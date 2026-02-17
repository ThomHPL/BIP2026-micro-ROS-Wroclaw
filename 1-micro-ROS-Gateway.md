### micro-ROS Gateway Node

### Exercise 7: Set Up micro-ROS Gateway Node
**Goal:** Modify the gateway code to work with micro-ROS

Create another **new Arduino sketch**, and copy the gateway code to it.
```cpp
// ========== LIBRARIES ==========
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// ========== FUNCTION PROTOTYPES ==========
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len);
esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo);

// Variables
uint8_t peerMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ========== SETUP FUNCTION ==========
void setup() {
  Serial.begin(115200);
  

  WiFi.begin();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  esp_now_init();

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  addPeer(peerMAC, NULL);
  
  Serial.println("Setup complete");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  set_microros_transports();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

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
}

// ========== MAIN LOOP ==========
void loop() {
  delay(100);
}

void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  // ...
}
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  return;
}

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

**Important:** Update `peerMAC` with the MAC address printed from the sensor node.

**Checkpoint:** Verify sensor initialization and MAC address display.

---

### Exercise 8: Repeat Sensor Data to micro-ROS
**Goal:** Transmit HC-SR04 distance readings to ROS

- Add includes for micro-ROS (see previous exercices)
- In `OnDataRecv()`, add:
```cpp
msg.data = *((double*)incomingData);
rcl_publish(&publisher, &msg, NULL);
```

**Checkpoint:** Upload to remote ESP32 and check gateway Serial Monitor for incoming distance data.

---

### Exercise 9: send micro-ROS subscribed topics to distant sensor board
**Goal:** Transmit ROS topics in which the micro-ROS Gateway is subscribed to the distant boar.

This Exercise is left as an exercise to the students.
