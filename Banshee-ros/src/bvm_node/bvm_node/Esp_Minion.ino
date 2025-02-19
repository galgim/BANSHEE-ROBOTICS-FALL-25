/*
#include <WiFi.h>
#include <esp_now.h>
#include <Arduino.h>

// Define the GPIO pin to control the solenoid
const int solenoidPin = GPIO_NUM_2;
const int solenoidPin2 = GPIO_NUM_4;

const int CELLS_PER_BATTERY = 4;

uint8_t masterMacAddress[] = {0xA0, 0xB7, 0x65, 0x25, 0xD4, 0xBC}; // A0:B7:65:25:D4:BC

uint8_t data[4];  // 4 bytes for voltage

// Callback when data is sent to master
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status: ");
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("Delivery Success");
    } else {
        Serial.println("Delivery Fail");
    }
}

void onDataReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    if (len != 2) {
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

    float chamber;
    memcpy(&chamber, incomingData, sizeof(chamber));

    Serial.println("Received Unlock signal");
    Serial.println("Received Unlock signal");
    Serial.println("Received Unlock signal");
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
    esp_now_register_recv_cb(onDataReceive);

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
    // Calculate total voltage
    float totalVoltage = 0;
    for (int i = 0; i < CELLS_PER_BATTERY; i++) {
        float cellVoltage = generateDummyVoltage();
        totalVoltage += cellVoltage;
        Serial.printf("Cell %d Voltage: %.3fV\n", i + 1, cellVoltage);
    }

    Serial.printf("Total Voltage: %.3fV\n", totalVoltage);
    memcpy(data, &totalVoltage, sizeof(totalVoltage));  

    // Send data to master ESP
    esp_err_t result = esp_now_send(masterMacAddress, data, sizeof(data));

    Serial.println(WiFi.macAddress());
    Serial.println("This esp is for chamber 1.");

    delay(10000); // Wait before sending the next packet
}
*/