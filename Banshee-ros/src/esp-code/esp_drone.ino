#include <esp_now.h>
#include <WiFi.h>


uint8_t masterMacAddress[] = {0xA0, 0xB7, 0x65, 0x25, 0xD4, 0xBC};
const int button = GPIO_NUM_35;


void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status: ");
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("Delivery Success");
    } else {
        Serial.println("Delivery Fail");
    }
}


void sendCommand(const char *command) {
    esp_err_t result = esp_now_send(masterMacAddress, (uint8_t *)command, strlen(command) + 1); // Send data
    if (result == ESP_OK) {
        Serial.println("Command forwarded successfully!");
    } else {
        Serial.println("Error sending command.");
    }
}


void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
   WiFi.mode(WIFI_STA);
   delay(1000);
   Serial.println(WiFi.macAddress());

   // Testing
   pinMode(button, INPUT);

   if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
   } else {
        Serial.println("Success initializing ESP-NOW");
   }

   esp_now_peer_info_t peerInfo = {};
   memcpy(peerInfo.peer_addr, masterMacAddress, 6);
   peerInfo.channel = 0;
   peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.print("Failed to add master peer: ");
      return;
  }

  esp_now_register_send_cb(onDataSent);


}

void loop() {
  // put your main code here, to run repeatedly:
  

    int buttonState = digitalRead(button);

    if (buttonState == HIGH) {
      Serial.println("Pressed!");
      sendCommand("done");
      delay(1000);
    }
}