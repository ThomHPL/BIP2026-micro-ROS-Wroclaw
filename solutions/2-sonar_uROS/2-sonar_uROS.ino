#include <HCSR04.h>
#include <micro_ros_arduino.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float64.h>

#define SONAR_TRIGGER_PIN 14
#define SONAR_ECHO_PIN 12

rcl_node_t node;
rcl_publisher_t publisher;
rclc_support_t support;
rcl_allocator_t allocator;
std_msgs__msg__Float64 msg;

void setup() {
  // Serial.begin(115200);
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
  
  // Serial.println("Micro-ROS initialized");
}

void loop() {
  double* distances = HCSR04.measureDistanceCm();
  
  msg.data = distances[0];
  rcl_publish(&publisher, &msg, NULL);
  
  // Serial.print("Distance: ");
  // Serial.print(distances[0]);
  // Serial.println(" cm");
  
  delay(100);
}