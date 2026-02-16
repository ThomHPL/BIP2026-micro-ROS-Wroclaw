#include <HCSR04.h>
#include <micro_ros_arduino.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float64.h>

#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

#define SONAR_TRIGGER_PIN 14
#define SONAR_ECHO_PIN 12
#define LED_PIN 2

rcl_node_t node;
rcl_publisher_t publisher;
rclc_support_t support;
rcl_allocator_t allocator;
std_msgs__msg__Float64 msg;

rclc_executor_t executor;
rcl_subscription_t subscriber;
std_msgs__msg__Bool sub_msg;

// Callback function when LED control message is received
void led_callback(const void * msgin) {
  bool data = ((const std_msgs__msg__Bool *)msgin)->data;
  digitalWrite(LED_PIN, data ? HIGH : LOW);
}

void setup() {
  // Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  HCSR04.begin(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN);
  
    // Initialize micro-ROS
  set_microros_transports();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create ROS node
  rclc_node_init_default(
    &node,
    "sonar_node",
    "",
    &support
  );

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "distance"
  );

  // Create subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "led_control"
  );

  // Create executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &led_callback, ON_NEW_DATA);
  
  // Serial.println("Micro-ROS initialized");
}

void loop() {
  double* distances = HCSR04.measureDistanceCm();
  
  msg.data = distances[0];
  rcl_publish(&publisher, &msg, NULL);
  
  // Serial.print("Distance: ");
  // Serial.print(distances[0]);
  // Serial.println(" cm");
  
  // Process ROS messages (check for incoming LED control)
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  
  delay(100);
}