/*
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

const int MAX_MINIONS = 8;
const int CELLS_PER_BATTERY = 4;  // Update if necessary

// 2D array to store minion ID and its total voltage
float minions[MAX_MINIONS][7];

void onDataReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    if (len != 10) {
        Serial.println("Invalid data length!");
        return;
    }

    uint8_t senderMac[6];
    float totalVoltage;
    memcpy(senderMac, incomingData, 6);  // Extract MAC
    memcpy(&totalVoltage, incomingData + 6, sizeof(totalVoltage));  // Extract Voltage

    // Print received data
    Serial.print("Received data from: ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X:", senderMac[i]);
    }
    Serial.printf(" Total Voltage: %.3fV\n", totalVoltage);

    // Check if the minion is already stored
    bool found = false;
    for (int i = 0; i < MAX_MINIONS; i++) {
        bool macMatch = true;
        for (int j = 0; j < 6; j++) {
            if (memcmp(minions[i], senderMac, 6) == 0) {
                minions[i][6] = totalVoltage;
                found = true;
                break;
            }
        }

        if (macMatch) {
            minions[i][6] = totalVoltage;  // Update voltage
            found = true;
            break;
        }
    }

    // If not found, add new minion
    if (!found) {
        for (int i = 0; i < MAX_MINIONS; i++) {
            if (minions[i][0] == 0) {  // Empty slot
                for (int j = 0; j < 6; j++) {
                    minions[i][j] = senderMac[j];  // Store MAC
                }
                minions[i][6] = totalVoltage;  // Store Voltage
                break;
            }
        }
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

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(onDataReceive);

    // Initialize all minions as inactive (set voltage to 0.0)
    for (int i = 0; i < MAX_MINIONS; i++) {
        minions[i][0] = 0;  // ID (initialized as 0)
        minions[i][1] = 0.0f;  // Voltage (initialized as 0.0)
    }
}

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5000;  // 5 seconds interval

void loop() {
    unsigned long currentMillis = millis();

    // Send data to server every 5 seconds
    if (currentMillis - lastSendTime >= sendInterval) {
        lastSendTime = currentMillis;

        // Check if we have any active minions
        bool hasActiveMinions = false;
        for (int i = 0; i < MAX_MINIONS; i++) {
            if (minions[i][1] != 0.0f) {
                hasActiveMinions = true;
                break;
            }
        }

        // // Only send if we have data to send
        // if (hasActiveMinions) {
        //     sendToServer();
        // }
    }
}
*/