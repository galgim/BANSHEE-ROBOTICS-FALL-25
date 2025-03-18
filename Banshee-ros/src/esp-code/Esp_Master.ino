#include <esp_now.h>
#include <WiFi.h>
#include <HTTPClient.h>

// Declarations
//void sendDataBvm(String tag, const void* data, size_t size);

const int LED = GPIO_NUM_32;


const int MAX_MINIONS = 4;
const int CELLS_PER_BATTERY = 4;  // Update if necessary

// 2D array to store minion ID and its total voltage
uint8_t minionsMacs[MAX_MINIONS][6] = {  // Master esp MAC {0xA0, 0xB7, 0x65, 0x25, 0xD4, 0xBC}
  {0xA0, 0xB7, 0x65, 0x25, 0xC2, 0x1C}, // Chamber 0
  {0xA0, 0xB7, 0x65, 0x25, 0x0A, 0x70}, // Chamber 1
  {0x34, 0x5F, 0x45, 0xE7, 0x33, 0x1C}, // Chamber 2
  {0xFC, 0xE8, 0xC0, 0x7C, 0x72, 0xBC}
  
  // Add more here when ready
  };

uint8_t gcs_esp[] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};
uint8_t drone_esp[] = {0xA0, 0xB7, 0x65, 0x26, 0xBE, 0x84}; 

float voltage[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
uint8_t data[2];  // 2 bytes for arduino int
int done = 0;
int chamber = -1; // -1 will indicate when no chamber is selected
char receivedCommand[32] = {0};

// Used for testing purposes
const int button = GPIO_NUM_35;
const int led0 = GPIO_NUM_32;
const int led1 = GPIO_NUM_33;
const int led2 = GPIO_NUM_25;

// Used to send byte data to UART serial port to communicate with BVM
// Tag indicates what kind of information it is, which will need to be coded on BVM python code
// Data is the byte data being send in a newline after the tag
void sendDataBvm(String tag, const void* data, size_t size) {
    Serial.println(tag);
    Serial.write((const uint8_t*)data, size);
    Serial.println();
  }

// Used to set the Voltage array in the correct order as the minion array is set.
void setVoltageFromMinion(const uint8_t *mac, float voltageValue) {
    for (int i = 0; i < MAX_MINIONS; i++) {
        if (memcmp(mac, minionsMacs[i], 6) == 0) {
            // Serial.println("Minion Address in List!");// Match found
            voltage[i] = voltageValue;
            return;
        }
    }
    Serial.println("Mac address not found in list");
}

// Reads from the UART serial port to get information from the BVM
// We first read the Tag, which we can decide what to do in the code based off this tag
// Then we read the data in the next line
void readFromBvm() {
    if (Serial.available()) {
        String tag = Serial.readStringUntil('\n');

        
        if (tag == "Unlock") {
          String message = Serial.readStringUntil('\n');
          chamber = message.toInt();
          int signal = 1;

          memcpy(data, &signal, sizeof(signal));
        
          esp_err_t result = esp_now_send(minionsMacs[chamber], data, sizeof(data)); // Sends corresponding minion signal 1 to unlock battery chamber
        }
        else if (tag == "Lock") {
          String message = Serial.readStringUntil('\n');
          chamber = message.toInt();
          int signal = 0;

          memcpy(data, &signal, sizeof(signal));

          esp_err_t result = esp_now_send(minionsMacs[chamber], data, sizeof(data)); // Sends corresponding minion signal 1 to lock battery chamber
          }
        else if (tag == "CycleComplete") {
          Serial.print("CycleComplete signal");
          }
        }
    else {
        return;
      }
}

// Triggers when master sends data through ESP_NOW
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status: ");
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("Delivery Success");
    } else {
        Serial.println("Delivery Fail");
    }
}

// Triggers when master receives data through ESP_NOW
void onDataReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    // if (len != 4) {
    //     Serial.println("Invalid data length!");
    //     return;
    // }

    uint8_t senderMac[6];
    memcpy(senderMac, info->src_addr, 6);  // Get sender's MAC

    Serial.flush();

    if (done == 0) {
      // Print received MAC address
      Serial.print("Received from: ");
      for (int i = 0; i < 6; i++) {
          Serial.printf("%02X", senderMac[i]);
          if (i < 5) Serial.print(":");
      }
      Serial.println();
    }

    // Debugging: Print stored minion MACs
//    Serial.println("Checking against stored minions:");
//    for (int i = 0; i < MAX_MINIONS; i++) {
//        Serial.print("Minion[" + String(i) + "]: ");
//        for (int j = 0; j < 6; j++) {
//            Serial.printf("%02X", minionsMacs[i][j]);
//            if (j < 5) Serial.print(":");
//        }
//        Serial.println();
//    }

    float totalVoltage;
    memcpy(&totalVoltage, incomingData, sizeof(totalVoltage));  // Extract Voltage

    setVoltageFromMinion(senderMac, totalVoltage);

    strncpy(receivedCommand, (const char *)incomingData, sizeof(receivedCommand) - 1);
    receivedCommand[sizeof(receivedCommand) - 1] = '\0';  // Null-terminate the string

    Serial.print("Received command: ");
    Serial.println(receivedCommand);

    if (strcmp(receivedCommand, "done") == 0) {
      Serial.println("Received 'done' command, triggering done = 0...");
      done = 0;  
    }
}

// // Function to send data to server
// void sendToServer() {
//     HTTPClient http;
//     String serverUrl = "http://your-server-ip:5000/api/rgs/voltages/";

//     StaticJsonDocument<2048> doc;
//     JsonArray batteryArray = doc.createNestedArray("batteries");

//     for (int i = 0; i < MAX_MINIONS; i++) {
//         // Only add active minions (where totalVoltage is not 0.0)
//         if (minions[i][1] != 0.0f) {
//             JsonObject battery = batteryArray.createNestedObject();
//             battery["id"] = (int)minions[i][0];  // Minion ID
//             battery["total"] = minions[i][1];  // Total Voltage
//             battery["last_update"] = millis();  // Use current time as last update
//         }
//     }

//     doc["timestamp"] = millis();

//     String requestBody;
//     serializeJson(doc, requestBody);

//     // Debug print
//     Serial.println("Sending to server:");
//     serializeJsonPretty(doc, Serial);

//     http.begin(serverUrl);
//     http.addHeader("Content-Type", "application/json");

//     int httpResponseCode = http.POST(requestBody);

//     if (httpResponseCode > 0) {
//         Serial.printf("Data sent to server. Response: %d\n", httpResponseCode);
//     } else {
//         Serial.printf("Failed to send data. Error: %s\n", http.errorToString(httpResponseCode).c_str());
//     }

//     http.end();
// }

// Attempts to add all peers in the minion array using their mac addresses
void addPeers() {
    esp_now_peer_info_t peerInfo = {};
    esp_now_peer_info_t peerInfoGCS = {};  // Create a separate peer info for GCS
    esp_now_peer_info_t peerInfoDrone = {}; // Create a separate peer info for drone
    for (int i = 0; i < MAX_MINIONS; i++) {
        memcpy(peerInfo.peer_addr, minionsMacs[i], 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;

        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
          Serial.print("Failed to add peer: ");
          Serial.println(i);
        }
    }

  // Set up the first peer (GCS)
  memcpy(peerInfoGCS.peer_addr, gcs_esp, 6);  // Use gcs_esp MAC address
  peerInfoGCS.channel = 0;
  peerInfoGCS.encrypt = false;

  // Set up the second peer (Drone)
  memcpy(peerInfoDrone.peer_addr, drone_esp, 6);  // Use drone_esp MAC address
  peerInfoDrone.channel = 0;
  peerInfoDrone.encrypt = false;

  // Add the peers
  if (esp_now_add_peer(&peerInfoGCS) != ESP_OK) {
      Serial.println("Failed to add GCS peer");
  }

  if (esp_now_add_peer(&peerInfoDrone) != ESP_OK) {
      Serial.println("Failed to add Drone peer");
  }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    // Testing
    pinMode(LED, OUTPUT);
  

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    addPeers();

    esp_now_register_recv_cb(onDataReceive);
    esp_now_register_send_cb(onDataSent);

}

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5000;  // 5 seconds interval

void loop() {
    unsigned long currentMillis = millis();
    
    // Send data to server every 5 seconds
    if (currentMillis - lastSendTime >= sendInterval) {
        lastSendTime = currentMillis;
        

        if(done == 0){
          Serial.println("signal not reached");
          int count = 0;
          for (int i = 0; i < 8; i++) {
            if (voltage[i] > 10.0) {
              count++;
              }
            }
          if (count > 0) {
            Serial.println("done signal reached");
            sendDataBvm("Voltage", voltage, sizeof(voltage));
            digitalWrite(LED, HIGH);

            done = 1;
            
          }
        }
        else {
          readFromBvm();
          digitalWrite(LED, LOW);
          }
        Serial.flush();
        // Check if we have any active minions

        // // Only send if we have data to send
        // if (hasActiveMinions) {
        //     sendToServer();
        // }
    }
}