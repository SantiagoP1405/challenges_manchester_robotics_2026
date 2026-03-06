#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>

// Pines y configuración 
#define variador 34
#define PWM_PIN 26
#define In1 23
#define In2 22
#define freq 5000
#define resolution 8
#define PWM1_Ch 0

// Variables globales 
float voltaje = 0;
float duty = 0;
int pot = 0;
int pwm = 0;
float Vcc = 3.3;
bool motorEnMovimiento = false;

// Objetos micro-ROS 
rcl_publisher_t pub_duty;      // publica duty cycle (%)
rcl_publisher_t pub_voltaje;   // publica voltaje (V)
rcl_publisher_t pub_pwm;       // publica valor PWM crudo 0-255
rcl_publisher_t pub_status;    // publica estado del motor
rcl_subscription_t sub_cmd;     // recibe comandos "D" / "I" / "S"

std_msgs__msg__Float32 msg_duty;
std_msgs__msg__Float32 msg_voltaje;
std_msgs__msg__Float32 msg_pwm;
std_msgs__msg__String msg_cmd;
std_msgs__msg__String status;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Macros de manejo de errores 
#define RCCHECK(fn)  { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK){ errorLoop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

void errorLoop() {
  while(true) { delay(100); }   // parpadeo o indicador de error
}

enum states {
  WAITING_AGENT,        // Waiting for ROS 2 agent connection
  AGENT_AVAILABLE,      // Agent detected
  AGENT_CONNECTED,      // Successfully connected
  AGENT_DISCONNECTED    // Connection lost
} state;

bool create_entities();
void destroy_entities();

// Funciones del motor
void detener() {
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  ledcWrite(PWM1_Ch, 0);
  status.data.data = "Detenido";
  RCSOFTCHECK(rcl_publish(&pub_status, &status, NULL));

}

void derecha() {
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  status.data.data = "Derecha";
  RCSOFTCHECK(rcl_publish(&pub_status, &status, NULL));
}

void izquierda() {
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  status.data.data = "Izquierda";
  RCSOFTCHECK(rcl_publish(&pub_status, &status, NULL));
}

void lecturaPWM() {
  pot     = analogRead(variador);
  voltaje = pot * (Vcc / 4095.0);
  duty    = 100.0 * voltaje / Vcc;
  pwm     = map(pot, 0, 4095, 0, 255);
  ledcWrite(PWM1_Ch, pwm);
}


// Callback del suscriptor de comandos 
void cmd_callback(const void* msgin) {
  const std_msgs__msg__String* msg =
      (const std_msgs__msg__String*)msgin;

  char opcion = msg->data.data[0];   // primer carácter del string

  switch (opcion) {
    case 'D':
      motorEnMovimiento = true;
      derecha();
      break;
    case 'I':
      motorEnMovimiento = true;
      izquierda();
      break;
    case 'S':
      motorEnMovimiento = false;
      detener();
      break;
  }
}

// Callback del timer (publicación periódica) 
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;

  if (motorEnMovimiento) {
    lecturaPWM();

    msg_duty.data = duty;
    msg_voltaje.data = voltaje;
    msg_pwm.data = (float)pwm;

    RCSOFTCHECK(rcl_publish(&pub_duty, &msg_duty, NULL));
    RCSOFTCHECK(rcl_publish(&pub_voltaje, &msg_voltaje, NULL));
    RCSOFTCHECK(rcl_publish(&pub_pwm, &msg_pwm, NULL));
  }
}

// Setup
void setup() {
  // Pines
  pinMode(variador,INPUT);
  pinMode(PWM_PIN,OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);

  // PWM
  ledcSetup(PWM1_Ch, freq, resolution);
  ledcAttachPin(PWM_PIN, PWM1_Ch);
  detener();

  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
  
}

bool create_entities() {
  // Inicializar micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor_pwm_node", "", &support));

  // Publishers
  RCCHECK(rclc_publisher_init_default(
      &pub_duty,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "motor/duty_cycle"));        

  RCCHECK(rclc_publisher_init_default(
      &pub_voltaje,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "motor/voltaje"));

  RCCHECK(rclc_publisher_init_default(
      &pub_pwm,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "motor/pwm_raw"));

  RCCHECK(rclc_publisher_init_default(
      &pub_status,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "motor/status"));

  // Subscriber
  RCCHECK(rclc_subscription_init_default(
      &sub_cmd,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "motor/cmd"));

  // Inicializar buffer del mensaje String
  msg_cmd.data.data = (char*)malloc(10 * sizeof(char));
  msg_cmd.data.size = 0;
  msg_cmd.data.capacity = 10;

  status.data.data = (char*)malloc(20 * sizeof(char));
  status.data.size = 0;
  status.data.capacity = 20;

  RCCHECK(rclc_timer_init_default(
      &timer, &support, 
      RCL_MS_TO_NS(100),
      timer_callback));

  // 2 handles: 1 timer + 1 suscriptor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_cmd, &msg_cmd,
      &cmd_callback, ON_NEW_DATA));
  return true;
}

void destroy_entities() {
  rclc_executor_fini(&executor);
  rcl_timer_fini(&timer);
  rcl_subscription_fini(&sub_cmd, &node);
  rcl_publisher_fini(&pub_duty, &node);
  rcl_publisher_fini(&pub_voltaje, &node);
  rcl_publisher_fini(&pub_pwm, &node);
  rcl_publisher_fini(&pub_status, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void loop() {
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