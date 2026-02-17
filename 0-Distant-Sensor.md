### Remote Sensor Node

### Exercise 7: Set Up Remote Sensor Node
**Goal:** Configure a second ESP32 to send data to the gateway

Create another **new Arduino sketch**, and use the same libraries and setup as Exercises 1-4:
```cpp
// ========== LIBRARIES ==========
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <HCSR04.h>

#define SONAR_TRIGGER_PIN 14
#define SONAR_ECHO_PIN 12

// ========== FUNCTION PROTOTYPES ==========
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len);
esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo);

// Variables - Update with gateway MAC address
uint8_t gatewayMAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void setup() {
  Serial.begin(115200);
  
  HCSR04.begin(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN);
  Serial.println("Sensor initialized");
  
  WiFi.begin();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_now_init();
  
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  addPeer(gatewayMAC, NULL);
  
  Serial.println("Remote sensor ready");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  delay(1000);
}
```

**Important:** Update `gatewayMAC` with the MAC address printed from the gateway in Exercise 1.

**Checkpoint:** Verify sensor initialization and MAC address display.

---

### Exercise 8: Send Sensor Data via ESP-NOW
**Goal:** Transmit HC-SR04 distance readings to the gateway

In `loop()`, add:
```cpp
void loop() {
  double* distances = HCSR04.measureDistanceCm();

  esp_now_send(gatewayMAC, (uint8_t*)distances, sizeof(double));
  
  delay(50);
}
```

Add the callback functions:
```cpp
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) { 
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  // Remote node can also receive feedback if needed
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

**Checkpoint:** Upload to remote ESP32 and check gateway Serial Monitor for incoming distance data.

---

### Exercise 9: Verify End-to-End Communication
**Goal:** Confirm complete data flow between nodes

1. **Gateway ESP32:** Upload Exercise 7 complete code
2. **Remote ESP32:** Upload Exercise 9 complete code
3. **Check Gateway Serial Monitor:** Should show received distance values like `Received data: D:12.45`

**Checkpoint:** If both devices display success messages and distance data flows correctly, the ESP-NOW gateway system is complete!