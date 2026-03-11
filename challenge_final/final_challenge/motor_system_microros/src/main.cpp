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
float   resolucion  = 0.0858983536;  // grados por pulso
int     pulsos      = 4191;          // pulsos por vuelta a la salida

int32_t tiempo_act   = 0;
int32_t tiempo_ant   = 0;
int32_t delta_tiempo = 2000000000;   // valor inicial grande para evitar /0
int32_t contador     = 0;
int32_t revoluciones = 0;
float   posicion     = 0;
float   velocidad    = 0;
float   velocidad_norm = 0;
float velocidad_rad_s = 0;

volatile bool BSet            = false;
volatile bool ASet            = false;
volatile bool encoderDirection = false;

int pwm_value = 0;

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
void IRAM_ATTR Encoder()
{
  BSet = digitalRead(EncB);
  ASet = digitalRead(EncA);

  if (BSet == ASet)
  {
    contador++;
    encoderDirection = true;
  }
  else
  {
    contador--;
    encoderDirection = false;
  }

  tiempo_act   = micros();
  delta_tiempo = tiempo_act - tiempo_ant;
  tiempo_ant   = tiempo_act;
}
// Pose estimation
void pose()
{
  if (encoderDirection)
  {
    posicion = contador * resolucion;
    if (contador >= pulsos)
    {
      revoluciones++;
      contador = 0;
    }
  }
  else
  {
    posicion = contador * resolucion;
    if (contador <= -pulsos)
    {
      revoluciones--;
      contador = 0;
    }
  }

  // FIX 1: variable LOCAL "velocidad_nueva" para no ocultar la global "velocidad"
  float velocidad_nueva = 60000000.0 / ((float)pulsos * (float)delta_tiempo);
  velocidad_nueva = abs(velocidad_nueva);
  if (!encoderDirection) velocidad_nueva = -velocidad_nueva;

  // FIX 2: EMA correcto — mezcla el valor nuevo con el valor ANTERIOR de la global
  float alpha = 0.3f;  // más reactivo que 0.05 para señales sinusoidales
  velocidad = alpha * velocidad_nueva + (1.0f - alpha) * velocidad;

  // Conversión RPM → rad/s para publicar en motor_w
  velocidad_rad_s = (velocidad * 2.0f * PI) / 60.0f;
}

void feedback_callback(){
  pose();
}

void pwm_callback(const void *msgin){
  const std_msgs__msg__Float32 *msg_in = (const std_msgs__msg__Float32 *)msgin;
  float value = msg_in->data;

  float mag = fabs(value);      
  uint8_t pwm_value = (uint8_t) roundf(mag * 255.0f);  // necesita <math.h>
  if (pwm_value > 255) pwm_value = 255;  // Restringe el valor a  entre -1 y 1, y luego escala a 0-255

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

bool create_entities() {
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_pwm,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_input"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_speed,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_w"));

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

void destroy_entities() {
  rcl_subscription_fini(&subscriber_pwm, &node);
  rcl_publisher_fini(&publisher_speed, &node);
  rcl_publisher_fini(&publisher_speed_rpm, &node);
  rcl_node_fini(&node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
}

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
        
        EXECUTE_EVERY_N_MS(20, {
          pose();
          speed_msg.data = velocidad_rad_s;
          speed_rpm_msg.data = velocidad;
          RCSOFTCHECK(rcl_publish(&publisher_speed, &speed_msg, NULL));
          RCSOFTCHECK(rcl_publish(&publisher_speed_rpm, &speed_rpm_msg, NULL));
        });
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
      
    default:
      break;
  }
}