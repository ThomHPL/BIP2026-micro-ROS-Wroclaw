# Micro ROS on MCU Workshop
## Step 1 - Installation
### Install micro-ROS agent

#### Add user to dialout group

```
sudo usermod -a -G dialout $USER
```
**You need to log out and login to enable this!**

#### Install rosdep
```
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

#### Build and install micro-ROS agent
See https://micro.ros.org/docs/tutorials/core/first_application_linux/

```
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
sudo apt install colcon
colcon build
source install/local_setup.bash
```

##### Creating the micro-ROS agent
Let’s first of all create a micro-ROS agent:
```
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh
```
Now, let’s build the agent packages and, when this is done, source the installation:

```
# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

##### Add micro-ROS environment to bashrc (optional)

You can add the micro-ROS workspace setup files to your .bashrc so the files do not have to be sourced every time a new command line is opened.

```
echo source ~/microros_ws/install/local_setup.bash >> ~/.bashrc
```

##### Running micro-ROS agent
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

### Install micro-ROS Arduino Library

Download the latest Micro ROS Arduino library from:

https://github.com/micro-ROS/micro_ros_arduino/archive/refs/tags/v2.0.8-rolling.zip

Install the Micro ROS Arduino library in the Arduino IDE:

1. Open Arduino IDE
2. Go to **Sketch** → **Include Library** → **Add .ZIP Library...**
3. Select the ZIP file you just downloaded

## Step 2 - SONAR using Micro ROS

### Exercise 1: Read HC-SR04 Sensor
**Goal:** Get distance readings from the ultrasonic sensor

Start with this skeleton code:
```cpp
#include <HCSR04.h>

#define SONAR_TRIGGER_PIN 14
#define SONAR_ECHO_PIN 12

void setup() {
  Serial.begin(115200);
  HCSR04.begin(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN);
  Serial.println("Sensor initialized");
}

void loop() {
  double* distances = HCSR04.measureDistanceCm();
  
  Serial.print("Distance: ");
  Serial.print(distances[0]);
  Serial.println(" cm");
  
  delay(100);
}
```

**Checkpoint:** Run this code and verify distances appear in the Serial Monitor (0-400 cm).

---

### Exercise 2: Add Micro-ROS Includes and Variables
**Goal:** Set up ROS communication infrastructure

Add these includes at the top:
```arduino
#include <micro_ros_arduino.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float64.h>
```

Add these global variables (after the sensor pins):
```cpp
rcl_node_t node;
rcl_publisher_t publisher;
rclc_support_t support;
rcl_allocator_t allocator;
std_msgs__msg__Float64 msg;
```

**Checkpoint:** Code should compile without errors.

---

### Exercise 3: Initialize Micro-ROS
**Goal:** Connect to the ROS 2 system

Add this to your `setup()` function after `HCSR04.begin()`:
```cpp
  // Initialize micro-ROS
  set_microros_transports();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create ROS node
  rclc_node_init_default(
    &node,
    /* PUT YOUR NODE NAME HERE */,
    "",
    &support
  );

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    /* PUT YOUR NODE NAME HERE */
  );
  
  Serial.println("Micro-ROS initialized");
```

**Checkpoint:** Start the micro-ROS agent on your host machine, then:
```bash
ros2 node list
# You should see: /distance
```
---

### Exercise 4: Publish Distance to ROS Topic
**Goal:** Send sensor data to ROS

Replace the Serial print in `loop()` with:
```cpp
void loop() {
  double* distances = HCSR04.measureDistanceCm();
  
  // Create and publish message
  msg.data = distances[0];
  rcl_publish(&publisher, &msg, NULL);
  
  delay(100);
}
```

**Checkpoint:** Subscribe to the topic on your host:
```bash
ros2 topic echo /distance
# You should see distance values updating
```
---


### Exercise 5: Subscribe to a ROS Topic
**Goal:** Receive a boolean value to change the internal LED state

#### Add includes
```arduino
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
```

#### Add variables
```arduino
#define LED_PIN 2

rclc_executor_t executor;
rcl_subscription_t subscriber;
std_msgs__msg__Bool sub_msg;
```
#### In the setup(), add a subscriber and an executor
```arduino
// Create subscriber
rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    /* PUT THE TOPIC NAME HERE */
);

// Create executor
rclc_executor_init(&executor, &support.context, 1, &allocator);
rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &led_callback, ON_NEW_DATA);
```

`&led_callback` is a pointer to a callback function that is called each time a message is received on the topic.

#### led_callback function
```arduino
void led_callback(const void * msgin) {
  bool data = ((const std_msgs__msg__Bool *)msgin)->data;
  digitalWrite(LED_PIN, data ? HIGH : LOW);
}
```

#### Running the executor in the loop()
```arduino
rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
```

**Checkpoint:** Subscribe to the topic on your host:
```bash
ros2 topic echo /distance
# You should see distance values updating
```
---

<!-- 
### Full Complete Code

Here's the complete solution combining all exercises:

```cpp
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
  
  delay(100);
}
```
-->

## Step 2 - ESP-NOW

## Step 3 - And now?
