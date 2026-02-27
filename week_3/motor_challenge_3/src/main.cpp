#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#define LED_PIN_P 23
#define LED_PIN_N 22
#define PWM_PIN  26
#define PWM_CHANNEL 0
#define PWM_RESOLUTION 8
#define PWM_FREQ 980 //980 hz

int pwm_value = 0;

rcl_subscription_t subscriber_pwm;
std_msgs__msg__Float32 pwm_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    delay(100);
  }
}

void pwm_callback(const void *msgin){
  const std_msgs__msg__Float32 *msg_in = (const std_msgs__msg__Float32 *)msgin;
  float value = msg_in->data;

  float mag = fabs(value);      
  uint8_t pwm_value = (uint8_t) roundf(mag * 255.0f);  // necesita <math.h>
  if (pwm_value > 255) pwm_value = 255;  

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

  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_pwm,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "cmd_pwm"));

  std_msgs__msg__Float32__init(&pwm_msg);
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_pwm, &pwm_msg, &pwm_callback, ON_NEW_DATA));  
  
}

void loop(){
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}