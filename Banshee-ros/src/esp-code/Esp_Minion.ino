#include <WiFi.h>
#include <esp_now.h>
#include <Arduino.h> 
#include <esp_adc_cal.h>

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
float totalVoltage = 0;

esp_adc_cal_characteristics_t adc_chars;
int mode = 0;
// Callback when data is sent to master
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    (void)info;  // avoid "unused parameter" warning, or use info->peer_addr if you want

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
  Serial.println("Received unlock signal");
  Serial.println("Received unlock signal");
  Serial.println("Received unlock signal");
  Serial.println("Received unlock signal");
  Serial.println("Received unlock signal");
  if (totalVoltage > 10.0) {
    mode = 1;
    }
  else {
    mode = 2;
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

    digitalWrite(solenoidPin, LOW);
    digitalWrite(solenoidPin2, LOW);
}

void loop() {
  // Calculate total voltage
  voltages[0] = (ReadVoltage(batteryPin0) * 4.2)/(56.0/19.0);
  voltages[1] = (ReadVoltage(batteryPin1) * 4.2)/(126.0/43.0);
  voltages[2] = (ReadVoltage(batteryPin2)* 4.2)/(3.0);
  voltages[3] = (ReadVoltage(batteryPin3) * 4.2)/(420.0/133.0);

  // Check if voltages between 0 and 4.2 volts, meaning its being charged.
  totalVoltage = 0; 
  for(int i=0; i < 4; i++){
    if(voltages[i] > 0.0 && voltages[i] < 4.3){
      voltages[i] -= 0.09;
    }
      totalVoltage += voltages[i];
  }
  Serial.printf("Cell 1: %.3fV\n", voltages[0]);
  Serial.printf("Cell 2: %.3fV\n", voltages[1]);
  Serial.printf("Cell 3: %.3fV\n", voltages[2]);
  Serial.printf("Cell 4: %.3fV\n", voltages[3]);
  Serial.printf("Total Voltage: %.3fV\n", totalVoltage);
  memcpy(data, &totalVoltage, sizeof(totalVoltage));  
  
  if (mode == 1) {
    if (totalVoltage < 10.0) {
      delay(3000);
      digitalWrite(solenoidPin, LOW);
      digitalWrite(solenoidPin2, LOW);
      mode = 0;
      }
    }
  else if (mode == 2) {
    if (totalVoltage > 10.0) {
      delay(1000);
      digitalWrite(solenoidPin, LOW);
      digitalWrite(solenoidPin2, LOW);
      mode = 0;
      }
    }
  // Send data to master ESP
  esp_err_t result = esp_now_send(masterMacAddress, data, sizeof(data));

  Serial.println(WiFi.macAddress());
  Serial.println("This esp is for chamber 3.");

  delay(5000); // Wait before sending the next packet
}
