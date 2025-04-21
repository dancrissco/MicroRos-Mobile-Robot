#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#if !defined(ESP32)
#error This example is only for ESP32 boards
#endif

// Motor pins
#define LEFT_IN1     2
#define LEFT_IN2     4
#define LEFT_ENABLE  5

#define RIGHT_IN1    25
#define RIGHT_IN2    26
#define RIGHT_ENABLE 27

// ROS objects
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Status LED
#define LED_PIN 13

// Limits
const float MAX_LINEAR_SPEED = 0.5;   // meters per second
const float MAX_ANGULAR_SPEED = 1.0;  // radians per second
const int MIN_PWM = 100;              // threshold to avoid stalling

// Utility macros
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) {} }

void error_loop() {
  while (true) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Scale velocity to PWM with minimum threshold
int scaleToPWM(float speed, float max_speed) {
  int pwm = (int)(255.0 * speed / max_speed);

  // Apply minimum threshold if non-zero
  if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;
  if (pwm < 0 && pwm > -MIN_PWM) pwm = -MIN_PWM;

  return constrain(pwm, -255, 255);
}

void setMotor(int in1, int in2, int enable, int speed) {
  int pwm = abs(speed);
  analogWrite(enable, pwm);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  float linear = msg->linear.x;
  float angular = msg->angular.z;

  int left_pwm = scaleToPWM(linear - angular, MAX_LINEAR_SPEED);
  int right_pwm = scaleToPWM(linear + angular, MAX_LINEAR_SPEED);

  Serial.print("L_PWM: ");
  Serial.print(left_pwm);
  Serial.print(" R_PWM: ");
  Serial.println(right_pwm);

  setMotor(LEFT_IN1, LEFT_IN2, LEFT_ENABLE, left_pwm);
  setMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_ENABLE, right_pwm);
}

void setup() {
  set_microros_wifi_transports("SSID", "Password", "IP Address", 8888);

  Serial.begin(115200);
  delay(1000);
  Serial.println("✅ Teleop test with min PWM threshold");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_ENABLE, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_ENABLE, OUTPUT);

  delay(1000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_teleop_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(50);
}

