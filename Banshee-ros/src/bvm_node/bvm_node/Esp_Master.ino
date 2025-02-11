/*
#include <esp_now.h>
#include <WiFi.h>
#include <HTTPClient.h>

const int MAX_MINIONS = 2;
const int CELLS_PER_BATTERY = 4;  // Update if necessary

// 2D array to store minion ID and its total voltage
uint8_t minionsMacs[MAX_MINIONS][6] = {
  {0xA0, 0xB7, 0x65, 0x25, 0x80, 0x68}, // Chamber 0
  {0xA0, 0xB7, 0x65, 0x25, 0x0A, 0x70} // Chamber 1
  
  // Add more here when ready
  };

float voltage[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

void sendDataBvm(String tag, const void* data, size_t size) {
    Serial.println(tag);
    Serial.write((const uint8_t*)data, size);
  }

void setVoltageFromMinion(const uint8_t *mac, float voltageValue) {
    for (int i = 0; i < MAX_MINIONS; i++) {
        if (memcmp(mac, minionsMacs[i], 6) == 0) {
            Serial.println("Minion Address in List!");// Match found
            voltage[i] = voltageValue;
            return;
        }
    }
    Serial.println("Mac address not found in list");
}

void onDataReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    if (len != 4) {
        Serial.println("Invalid data length!");
        return;
    }

    uint8_t senderMac[6];
    memcpy(senderMac, info->src_addr, 6);  // Get sender's MAC

    // Print received MAC address
    Serial.print("Received from: ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", senderMac[i]);
        if (i < 5) Serial.print(":");
    }
    Serial.println();

    // Debugging: Print stored minion MACs
    Serial.println("Checking against stored minions:");
    for (int i = 0; i < MAX_MINIONS; i++) {
        Serial.print("Minion[" + String(i) + "]: ");
        for (int j = 0; j < 6; j++) {
            Serial.printf("%02X", minionsMacs[i][j]);
            if (j < 5) Serial.print(":");
        }
        Serial.println();
    }

    float totalVoltage;
    memcpy(&totalVoltage, incomingData, sizeof(totalVoltage));  // Extract Voltage

    setVoltageFromMinion(senderMac, totalVoltage);

    sendDataBvm("Voltage", voltage, sizeof(voltage));
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

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(onDataReceive);

}

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5000;  // 5 seconds interval

void loop() {
    unsigned long currentMillis = millis();

    // Send data to server every 5 seconds
    if (currentMillis - lastSendTime >= sendInterval) {
        lastSendTime = currentMillis;

        // Check if we have any active minions

        // // Only send if we have data to send
        // if (hasActiveMinions) {
        //     sendToServer();
        // }
    }
}
*/