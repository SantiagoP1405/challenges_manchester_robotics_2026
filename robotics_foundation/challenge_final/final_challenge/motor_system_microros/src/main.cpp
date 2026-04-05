#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#define EncA 25
#define EncB 27
#define LED_PIN_P 23
#define LED_PIN_N 22
#define PWM_PIN  26
#define PWM_CHANNEL 0
#define PWM_RESOLUTION 8
#define PWM_FREQ 980 //980 hz



// ── Encoder ───────────────────────────────────────────────────
float resolution = 0.0858983536;  // degrees per pulse
int pulses = 4191;          // pulses per revolution at the gearbox output

// Timing variables used to estimate velocity
int32_t time_act   = 0;
int32_t time_ant   = 0;
int32_t delta_time = 2000000000;   // large initial value to avoid division by zero

// Encoder state variables
int32_t counter     = 0;
int32_t revolutions = 0;

// Estimated motion variables
float   position     = 0;
float   speed    = 0; // speed in RPM
float   speed_norm = 0;
float speed_rad_s = 0; // speed in rad/s

// Encoder direction flags
volatile bool BSet            = false;
volatile bool ASet            = false;
volatile bool encoderDirection = false;

int pwm_value = 0;

// micro-ROS entities
rcl_subscription_t subscriber_pwm;
std_msgs__msg__Float32 pwm_msg;

rcl_publisher_t publisher_speed;
std_msgs__msg__Float32 speed_msg;

rcl_publisher_t publisher_speed_rpm;
std_msgs__msg__Float32 speed_rpm_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Executes a statement (X) every N milliseconds
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

void error_loop() {
  while(1) {
    delay(100);
  }
}

// micro-ROS connection state machine
enum states {
  WAITING_AGENT,        // Waiting for ROS 2 agent connection
  AGENT_AVAILABLE,      // Agent detected
  AGENT_CONNECTED,      // Successfully connected
  AGENT_DISCONNECTED    // Connection lost
} state;

// ======== Function Prototypes ========
bool create_entities();
void destroy_entities();

// Encoder ISR
// Triggered on every change of encoder channel A.
// Direction is determined by comparing channels A and B.
void IRAM_ATTR Encoder()
{
  BSet = digitalRead(EncB);
  ASet = digitalRead(EncA);

  if (BSet == ASet)
  {
    counter++;
    encoderDirection = true;
  }
  else
  {
    counter--;
    encoderDirection = false;
  }

  // Measure time between pulses for velocity estimation
  time_act   = micros();
  delta_time = time_act - time_ant;
  time_ant   = time_act;
}
// Position and velocity estimation
void pose()
{
  if (encoderDirection) // Update angular position
  {
    position = counter * resolution;
    if (counter >= pulses)
    {
      revolutions++;
      counter = 0;
    }
  }
  else
  {
    position = counter * resolution;
    if (counter <= -pulses)
    {
      revolutions--;
      counter = 0;
    }
  }

  float speed_new = 60000000.0 / ((float)pulses * (float)delta_time);
  speed_new = abs(speed_new);
  if (!encoderDirection) speed_new = -speed_new;

   // Exponential Moving Average filter used to smooth the velocity estimate.
  // The instantaneous velocity computed from encoder pulse timing can be very noisy
  // due to quantization effects, jitter in pulse timing, and ISR latency.
  // The EMA filter reduces high-frequency noise by combining the new measurement
  // with the previous filtered value:
  //      v_filtered = α * v_new + (1 - α) * v_previous
  // where:
  //   α (alpha) is the smoothing factor in the range (0,1)
  // In this case α = 0.15 provides a good compromise between noise reduction
  // and responsiveness for motor speed measurements.
  float alpha = 0.15f;  
  speed = alpha * speed_new + (1.0f - alpha) * speed;

  // Convert RPM to rad/s (used by the ROS controller)
  speed_rad_s = (speed * 2.0f * PI) / 60.0f;
}

// Wrapper function used if feedback processing is needed
// void feedback_callback(){
//   pose();
// }

// PWM command callback 
// Receives the normalized control signal [-1,1] from ROS
// and converts it to an 8-bit PWM signal.
void pwm_callback(const void *msgin){
  const std_msgs__msg__Float32 *msg_in = (const std_msgs__msg__Float32 *)msgin;
  float value = msg_in->data;

  float mag = fabs(value);      
  uint8_t pwm_value = (uint8_t) roundf(mag * 255.0f);  // Scale normalized input to PWM range [0,255]
  if (pwm_value > 255) pwm_value = 255;  

  // Direction control using H-bridge logic
  if (value >= 0.0f) {
    digitalWrite(LED_PIN_P, HIGH);
    digitalWrite(LED_PIN_N, LOW);

  }
  else if (value < 0.0f) {
    digitalWrite(LED_PIN_P, LOW);
    digitalWrite(LED_PIN_N, HIGH);
  }
  ledcWrite(PWM_CHANNEL,pwm_value);
}

void setup(){
  pinMode(LED_PIN_P, OUTPUT);
  pinMode(LED_PIN_N, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);

  pinMode(EncA, INPUT_PULLUP);
  pinMode(EncB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EncA), Encoder, CHANGE);

  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

}

// Create ROS 2 entities
bool create_entities() {
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor", "", &support));

  // Subscriber for control input
  RCCHECK(rclc_subscription_init_default(
    &subscriber_pwm,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_input"));
  
  // Publisher for angular velocity in rad/s
  RCCHECK(rclc_publisher_init_default(
    &publisher_speed,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_w"));

  // Publisher for angular velocity in RPM (used for debugging)  
  RCCHECK(rclc_publisher_init_default(
    &publisher_speed_rpm,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_w_rpm"));

  std_msgs__msg__Float32__init(&pwm_msg);
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_pwm, &pwm_msg, &pwm_callback, ON_NEW_DATA));  


  return true;
}

// Destroy ROS 2 entities if connection is lost
void destroy_entities() {
  rcl_subscription_fini(&subscriber_pwm, &node);
  rcl_publisher_fini(&publisher_speed, &node);
  rcl_publisher_fini(&publisher_speed_rpm, &node);
  rcl_node_fini(&node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
}

// Main loop with connection state machine
void loop(){
  switch (state) {

    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        
        // Publish motor speed at 50 Hz
        EXECUTE_EVERY_N_MS(20, {
          pose();
          speed_msg.data = speed_rad_s;
          speed_rpm_msg.data = speed;
          RCSOFTCHECK(rcl_publish(&publisher_speed, &speed_msg, NULL));
          RCSOFTCHECK(rcl_publish(&publisher_speed_rpm, &speed_rpm_msg, NULL));
        });
      }
      break;
    
    // If connection is lost, destroy entities and restart
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
      
    default:
      break;
  }
}