# Micro ROS on MCU Workshop
## Step 1 - Installation
### Install micro-ROS agent
<!--
#### Installing tools
##### Add user to dialout group

```bash
sudo usermod -a -G dialout $USER
```
**You need to log out and login to enable this!**

##### Install rosdep
```bash
sudo apt install python3-rosdep python3-pip colcon
sudo rosdep init
sudo apt update
rosdep update
```

#### Build and install micro-ROS agent
See https://micro.ros.org/docs/tutorials/core/first_application_linux/

```bash
# Create the environment variable (replace zith ros version on the computer)
export ROS_EXPORT=jazzy

# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Install from path
rosdep install --from-paths src --ignore-src -y

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

##### Creating the micro-ROS agent
Let’s first of all create a micro-ROS agent:
```bash
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh
```
Now, let’s build the agent packages and, when this is done, source the installation:

```bash
# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

-->

# Create a user-local rosdep directory

```bash
mkdir -p ~/.ros/rosdep/sources.list.d
```

# Create the default sources list

```bash
nano ~/.ros/rosdep/sources.list.d/20-default.list
```

Put this inside:
```ruby
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/jazzy/distribution.yaml jazzy
```

Save and exit.

# Tell rosdep to use your local config

Add this to your shell:
```bash
export ROSDEP_SOURCE_PATH=$HOME/.ros/rosdep/sources.list.d
```

To make it permanent:
```bash
echo 'export ROSDEP_SOURCE_PATH=$HOME/.ros/rosdep/sources.list.d' >> ~/.bashrc
source ~/.bashrc
```

# Update rosdep

```bash
rosdep update
```

# Install and build
```bash
# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep install --from-paths src --ignore-src -r -y --as-root apt:false
colcon build

# Add the microros agent qnd msgs sources
git clone https://github.com/ThomHPL/BIP2026-micro-ROS-Wroclaw.git
cp -R BIP2026-micro-ROS-Wroclaw/microros_ws/src/micro-ros-agent ~/microros_ws/src/
cp -R BIP2026-micro-ROS-Wroclaw/microros_ws/src/micro_ros_msgs ~/microros_ws/src/

cd ~/microros_ws
rosdep install --from-paths src --ignore-src -r -y --as-root apt:false
colcon build

source install/local_setup.bash
```

# Run agent
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```


##### Add micro-ROS environment to bashrc (optional)

You can add the micro-ROS workspace setup files to your .bashrc so the files do not have to be sourced every time a new command line is opened.

```bash
echo source ~/microros_ws/install/local_setup.bash >> ~/.bashrc
```

##### Running micro-ROS agent
```bash
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
```cpp
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
```cpp
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
```

#### Add variables
```cpp
#define LED_PIN 2

rclc_executor_t executor;
rcl_subscription_t subscriber;
std_msgs__msg__Bool sub_msg;
```
#### In the setup(), add a subscriber and an executor
```cpp
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
```cpp
void led_callback(const void * msgin) {
  bool data = ((const std_msgs__msg__Bool *)msgin)->data;
  digitalWrite(LED_PIN, data ? HIGH : LOW);
}
```

#### Running the executor in the loop()
```cpp
rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
```

**Checkpoint:** Try to publish to the topic to light up the LED:
```bash
ros2 topic pub /led_cpontrol std_msgs/Bool "data: 1"
# The LED should light up
```
---

## Step 3 - ESP-NOW

### Generic Gateway Code

### Exercise 1: Initialize WiFi and ESP-NOW
**Goal:** Set up the ESP32 module for ESP-NOW communication

Create a **new Arduino sketch** and start with this skeleton code:
```cpp
// ========== LIBRARIES ==========
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// ========== SETUP FUNCTION ==========
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Initialize WiFi
  WiFi.begin();
  // Set the Wi-Fi channel (e.g., channel 1-13), should match between boards
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  // Initialize ESP-NOW
  esp_now_init();

  Serial.println("Setup complete");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
}

// ========== MAIN LOOP ==========
void loop() {
  delay(1000);
}
```

**Checkpoint:** Upload and verify the MAC address prints in Serial Monitor (115200 baud).

**Register somewhere this MAC address for later!**

---

### Exercise 2: Add Function Prototypes and Peer Variables
**Goal:** Define the callback functions and peer information structure

Add these after the includes:
```cpp
// ========== FUNCTION PROTOTYPES ==========
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len);
esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo);

// Variables
uint8_t peerMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
```

**Note:** Update `peerMAC` with the actual MAC address of the **peer** ESP32 you want to communicate with.

**Checkpoint:** Code should compile without errors.

---

### Exercise 3: Register Callbacks
**Goal:** Set up callback functions to handle sent and received data

In `setup()`, after `esp_now_init()`, add:
```cpp
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
```

Create the callback function stubs before `setup()`:
```cpp
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) { 
  Serial.print("Last Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  Serial.print("Received ");
  Serial.print(len);
  Serial.println(" bytes");
}
```

**Checkpoint:** Code should compile and callbacks should trigger when data is sent/received (checked later).

---

### Exercise 4: Implement the addPeer Function
**Goal:** Create a function to add peer devices to ESP-NOW

Add this function before `setup()`:
```cpp
esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo){
  if(peerInfo == NULL){
    peerInfo = (esp_now_peer_info_t*)malloc(sizeof(esp_now_peer_info_t));
    memset(peerInfo, 0, sizeof(esp_now_peer_info_t));
  }
  memcpy(peerInfo->peer_addr, mac, 6);
  peerInfo->channel = 0;
  peerInfo->encrypt = false;
  return esp_now_add_peer(peerInfo);
}
```

In `setup()`, after `esp_now_register_recv_cb()`, add:
```cpp
  addPeer(peerMAC, NULL);
```

**Checkpoint:** Code should compile without errors.

---

### Exercise 5: Send Data via ESP-NOW
**Goal:** Transmit messages to the peer device

In `loop()`, replace the delay with:
```cpp
void loop() {
  // Wait before next measurement
  delay(1000);  // Send data every 1000ms
  char msg[16] = "Hello";
  esp_now_send(peerMAC, (uint8_t*)msg, strlen(msg));
}
```

**Checkpoint:** Upload to one ESP32. The `OnDataSent` callback should print "Delivery Success" if the peer MAC is reachable.

---

### Exercise 6: Receive and Display Data
**Goal:** Parse received messages and display them

Update the `OnDataRecv()` callback:
```cpp
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  char msg[len+1] = {0};
  memcpy((void*)incomingData, msg, len);

  Serial.print("Received data: ");
  Serial.println((char*) msg);
}
```

**Checkpoint:** Set up two ESP32 boards, upload gateway code with the second ESP32 MAC to the first one, and modified code with the first ESP32 MAC to the other. Messages should print in the gateway Serial Monitor.

---

### Sensor and micro-ROS boards 

One student has to follow file `0-Distant-Sensor.md`, the other follows `1-micro-ROS-Gateway.md`.
