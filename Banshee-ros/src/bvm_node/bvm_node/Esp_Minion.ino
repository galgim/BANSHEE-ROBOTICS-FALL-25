/*
#include <WiFi.h>
#include <esp_now.h>
#include <Arduino.h>

// Define the GPIO pin to control the solenoid
const int solenoidPin = GPIO_NUM_1;
const int solenoidPin2 = GPIO_NUM_3;

const int CELLS_PER_BATTERY = 4;

uint8_t masterMacAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  

uint8_t data[10];  // 6 bytes for MAC, 4 bytes for voltage

// Callback when data is sent to master
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status: ");
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("Delivery Success");
    } else {
        Serial.println("Delivery Fail");
    }
}

// Generate dummy voltage data for simplicity
float generateDummyVoltage() {
    return 3.7 + (random(0, 100) / 1000.0);  // Random voltage between 3.7V and 3.8V
}

// Initialize solenoid pins and ESP-NOW
void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Failed to initialize ESP-NOW");
        return;
    }

    esp_now_register_send_cb(OnDataSent);

    // Register master ESP as peer
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, masterMacAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    Serial.println("Sender ESP Ready to send data");

    // Initialize solenoid pins
    pinMode(solenoidPin, OUTPUT);
    pinMode(solenoidPin2, OUTPUT);
}

void loop() {
    // Turn on solenoid for 0.5s
    digitalWrite(solenoidPin, HIGH);
    digitalWrite(solenoidPin2, HIGH);
    delay(500); // Keep solenoid active
    digitalWrite(solenoidPin, LOW);
    digitalWrite(solenoidPin2, LOW);

    // Get Minion MAC Address
    uint8_t minionMac[6];
    WiFi.macAddress(minionMac);

    // Calculate total voltage
    float totalVoltage = 0;
    for (int i = 0; i < CELLS_PER_BATTERY; i++) {
        float cellVoltage = generateDummyVoltage();
        totalVoltage += cellVoltage;
        Serial.printf("Cell %d Voltage: %.3fV\n", i + 1, cellVoltage);
    }

    Serial.printf("Total Voltage: %.3fV\n", totalVoltage);
    memcpy(data, minionMac, 6);
    memcpy(&data[6], &totalVoltage, sizeof(totalVoltage));  

    // Send data to master ESP
    esp_err_t result = esp_now_send(masterMacAddress, data, sizeof(data));

    Serial.println(digitalRead(solenoidPin));
    Serial.println(digitalRead(solenoidPin2));
    Serial.println("------------------------");

    delay(1000); // Wait before sending the next packet
}
*/