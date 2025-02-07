#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
// #include <Preferences.h>

const int NUM_BATTERIES = 8;
const int CELLS_PER_BATTERY = 4;
const int MAX_MINIONS = 8;

/*
void handleSerialRequest() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command == "voltages") {
            StaticJsonDocument<2048> doc;
            JsonArray batteryArray = doc.createNestedArray("batteries");

            for (int i = 0; i < MAX_MINIONS; i++) {
                if (minions[i].isActive) {
                    JsonObject battery = batteryArray.createNestedObject();
                    battery["id"] = i + 1;
                    
                    JsonArray cells = battery.createNestedArray("cells");
                    float totalVoltage = 0;
                    
                    for (int j = 0; j < CELLS_PER_BATTERY; j++) {
                        cells.add(minions[i].cellVoltages[j]);
                        totalVoltage += minions[i].cellVoltages[j];
                    }
                    
                    battery["total"] = totalVoltage;
                    battery["last_update"] = minions[i].lastUpdate;
                }
            }

            // Send JSON response over Serial
            serializeJson(doc, Serial);
            Serial.println(); // Add newline for easier parsing
        }
        // else if (command == "status") {
        //     // Send simple status response
        //     Serial.printf("Active Minions: ");
        //     for (int i = 0; i < MAX_MINIONS; i++) {
        //         if (minions[i].isActive) {
        //             Serial.printf("%d ", i + 1);
        //         }
        //     }
        //     Serial.println();
        // }
    }
}
*/

void sendToServer() {
    HTTPClient http;
    String serverUrl = "http://your-server-ip:5000/api/rgs/voltages/";
    
    StaticJsonDocument<2048> doc;  
    JsonArray batteryArray = doc.createNestedArray("batteries");

    for (int i = 0; i < MAX_MINIONS; i++) {
        if (minions[i].isActive) {
            JsonObject battery = batteryArray.createNestedObject();
            battery["id"] = i + 1;  // Battery ID matches minion ID

            JsonArray cells = battery.createNestedArray("cells");
            float totalVoltage = 0;
            bool hasInvalidCell = false;

            for (int j = 0; j < CELLS_PER_BATTERY; j++) {
                float cellVoltage = minions[i].cellVoltages[j];
                cells.add(cellVoltage);

                if (!isValidVoltage(cellVoltage)) {
                    hasInvalidCell = true;
                }
                totalVoltage += cellVoltage;
            }

            battery["total"] = totalVoltage;
            battery["last_update"] = minions[i].lastUpdate;
            
            if (hasInvalidCell) {
                battery["warning"] = "Abnormal cell voltage detected";
            }
        }
    }

    doc["timestamp"] = millis();

    String requestBody;
    serializeJson(doc, requestBody);

    // Debug print
    Serial.println("Sending to server:");
    serializeJsonPretty(doc, Serial);

    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");
    
    int httpResponseCode = http.POST(requestBody);
    
    if (httpResponseCode > 0) {
        Serial.printf("Data sent to server. Response: %d\n", httpResponseCode);
    } else {
        Serial.printf("Failed to send data. Error: %s\n", http.errorToString(httpResponseCode).c_str());
    }
    
    http.end();
}

void onDataReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    
}

void setup() {
    Serial.begin(115200);
    
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(onDataReceive);

    for (int i = 0; i < MAX_MINIONS; i++) {
      minions[i].isActive = false;
      minions[i].lastUpdate = 0;
      memset(minions[i].cellVoltages, 0, sizeof(float) * CELLS_PER_BATTERY);
  } 
}

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5000;  // 5 seconds interval

void sendData(String tag, const void* data, size_t size) {
    Serial.flush();
    Serial.println(tag);
    Serial.write((const uint8_t*)data, size);
}

void loop() {

    unsigned long currentMillis = millis();

    // Send data to server every 5 seconds
    if (currentMillis - lastSendTime >= sendInterval) {
        lastSendTime = currentMillis;

        // Check if we have any active minions
        bool hasActiveMinions = false;
        for (int i = 0; i < MAX_MINIONS; i++) {
            if (minions[i].isActive) {
                hasActiveMinions = true;
                break;
            }
        }

        // Only send if we have data to send
        if (hasActiveMinions) {
            sendToServer();
        }
    }
}