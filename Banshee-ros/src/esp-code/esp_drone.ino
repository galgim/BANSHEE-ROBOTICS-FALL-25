#include <esp_now.h>
#include <WiFi.h>


uint8_t masterMacAddress[] = {0xA0, 0xB7, 0x65, 0x25, 0xD4, 0xBC};
uint8_t gcsMacAddress[] = {0xA0, 0xB7, 0x65, 0x25, 0xC6, 0x7C};
const int button = GPIO_NUM_36;
uint8_t data[2];  // 2 bytes for arduino int


// Checks if data was sent successfully
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status:  ");
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("Delivery Success");
    } else {
        Serial.println("Delivery Fail");
    }
}

void onDataReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    uint8_t senderMac[6];
    memcpy(senderMac, info->src_addr, 6);

    Serial.flush();

    // Print received MAC address
    Serial.print("Received from: ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", senderMac[i]);
        if (i < 5) Serial.print(":");
    }
    Serial.println();

    if (memcmp(mac, masterMacAddress, 6) == 0) {
      Serial.println("Drone complete")
      }
    
}

void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200); // Initialize serial monitor for debugging
   WiFi.mode(WIFI_STA);  // Set ESP32 to station for ESP-NOW
   delay(1000);          // Delay for stability
   Serial.println(WiFi.macAddress()); // Print ESP32 MAC address

   // Testing
   pinMode(button, INPUT);

   if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
   } else {
        Serial.println("Success initializing ESP-NOW");
   }

  // Monitors send status
  esp_now_register_send_cb(onDataSent);

  // Add GCS as a peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, gcsMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add GCS peer");
    return;
  }
  
  /*
  // Used if need to send signal to Master
  esp_now_peer_info_t masterPeer = {};
  memcpy(masterPeer.peer_addr, masterMacAddress, 6);
  masterPeer.channel = 0;
  masterPeer.encrypt = false;

  if (esp_now_add_peer(&masterPeer) != ESP_OK) {
    Serial.println("Failed to add Master peer");
    return;
  }
  */
  
}

void loop() {
  
    int buttonState = digitalRead(button);

    if (buttonState == HIGH) {
      Serial.println("Pressed!");
      int signal = 0;
      memcpy(data, &signal, sizeof(signal));
      esp_err_t result = esp_now_send(gcsMacAddress, data, sizeof(data));
      delay(1000);
    }
}