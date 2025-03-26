VoidMango
voidmango
Online

VoidMango — 10:17 AM
what the heck
Gimgal — 10:17 AM
that was so random
Gimgal — 11:09 AM
Image
Image
Image
VoidMango — 11:11 AM
i dont think that cube has any extra ports
for electronics
hmmm
Gimgal — 11:12 AM
honestly
battery swapping looks impossible
without outside mounting
VoidMango — 11:12 AM
it is impossible
i told you the bs needs to go
lol
Gimgal — 11:12 AM
LOL
i see these guys really want this to work
VoidMango — 11:12 AM
no theyre scope too big rn we havent even worked on the sinch drone yet
so i would really emphasize on the sinch drone getting done first before moving on
Gimgal — 11:14 AM
ill try to be clear with that
VoidMango — 11:14 AM
dang taking initiative already
lol
Gimgal — 11:14 AM
i gots to
i met the new aero lead
VoidMango — 11:15 AM
ill get you guys cuaght up on ee stuff coming next months
who
Gimgal — 11:15 AM
nathan
VoidMango — 11:15 AM
oh i see
what you think of him
Gimgal — 11:15 AM
hes gonna be aero next year
VoidMango — 11:15 AM
?
Gimgal — 11:15 AM
i thjnk
he just like me LOL
VoidMango — 11:15 AM
lol
Gimgal — 11:15 AM
we both kida clueless
VoidMango — 11:15 AM
wdym by that
ahahaha
thats usually how it goes
Gimgal — 11:15 AM
but he seems chill
have to see how he works
also is christian a new lead too?
VoidMango — 11:16 AM
most of the time you wont be working with aero but we shall see
yeah so you should get to know him more
Gimgal — 11:16 AM
will do
he ee lead?
VoidMango — 11:16 AM
no hes comms lead
Gimgal — 11:16 AM
o rught
VoidMango — 11:17 AM
ee lead is some aero guy thats also clue less
Gimgal — 11:17 AM
LOL
at least i have josh
VoidMango — 11:19 AM
you have josh so thats good
hes smarter than me so you should be fine 
just make sure he doenst do everything by himself 
gotta split work up
Gimgal — 11:20 AM
got it
VoidMango — 1:15 PM
can you send your minion over
code
Gimgal — 1:15 PM
#include <WiFi.h>
#include <esp_now.h>
#include <Arduino.h> 
#include "esp_adc_cal.h"

// Define the GPIO pin to control the solenoid
Expand
message.txt
5 KB
﻿
#include <WiFi.h>
#include <esp_now.h>
#include <Arduino.h> 
#include "esp_adc_cal.h"

// Define the GPIO pin to control the solenoid
const int solenoidPin = GPIO_NUM_2;
const int solenoidPin2 = GPIO_NUM_4;

const int CELLS_PER_BATTERY = 4;

uint8_t masterMacAddress[] = {0xA0, 0xB7, 0x65, 0x25, 0xD4, 0xBC}; // A0:B7:65:25:D4:BC
uint8_t data[4];  // 4 bytes for voltage

const int batteryPin0 = GPIO_NUM_32;
const int batteryPin1 = GPIO_NUM_33;
const int batteryPin2 = GPIO_NUM_34;
const int batteryPin3 = GPIO_NUM_35;

float voltages[4] = {0, 0, 0, 0};

esp_adc_cal_characteristics_t adc_chars;

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

    int command;
    memcpy(&command, incomingData, sizeof(command));

    if (command == 1) {
      digitalWrite(solenoidPin, HIGH);
      digitalWrite(solenoidPin2, HIGH);

      Serial.println("Received unlock signal");
      }
    else if (command == 0) {
      digitalWrite(solenoidPin, LOW);
      digitalWrite(solenoidPin2, LOW);

      Serial.println("Received lock signal");
      }
}

double ReadVoltage(byte pin){  
  uint32_t adc_reading = analogRead(pin);  // Raw ADC value
  uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);
  
  return (voltage_mv / 1000.0); // Convert from mV to V
}

// Initialize solenoid pins and ESP-NOW
void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    analogSetPinAttenuation(batteryPin0, ADC_11db);
    analogSetPinAttenuation(batteryPin1, ADC_11db);
    analogSetPinAttenuation(batteryPin2, ADC_11db);
    analogSetPinAttenuation(batteryPin3, ADC_11db);

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    
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
    voltages[0] = (ReadVoltage(batteryPin0) * 4.2)/(56.0/19.0);
    voltages[1] = (ReadVoltage(batteryPin1) * 4.2)/(126.0/43.0);
    voltages[2] = (ReadVoltage(batteryPin2)* 4.2)/(3.0);
    voltages[3] = (ReadVoltage(batteryPin3) * 4.2)/(420.0/133.0);

  // Check if voltages between 0 and 4.2 volts, meaning its being charged. 
    for(int i=0; i < 4; i++){
      if(voltages[i] > 0.0 && voltages[i] < 4.3){
        voltages[i] -= 0.09;
      }
        totalVoltage += voltages[i];
    }

    if(totalVoltage > 10) {
      digitalWrite(solenoidPin, LOW);
      digitalWrite(solenoidPin2, LOW);
      Serial.println("Received lock signal");

    }

    Serial.printf("Cell 1: %.3fV\n", voltages[0]);
    Serial.printf("Cell 2: %.3fV\n", voltages[1]);
    Serial.printf("Cell 3: %.3fV\n", voltages[2]);
    Serial.printf("Cell 4: %.3fV\n", voltages[3]);


    Serial.printf("Total Voltage: %.3fV\n", totalVoltage);
    memcpy(data, &totalVoltage, sizeof(totalVoltage));  

    // Send data to master ESP
    esp_err_t result = esp_now_send(masterMacAddress, data, sizeof(data));

    Serial.println(WiFi.macAddress());
    Serial.println("This esp is for chamber 3.");

    delay(5000); // Wait before sending the next packet
}