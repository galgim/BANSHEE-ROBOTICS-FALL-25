#include <esp_now.h>
#include <WiFi.h>

#define Relayin1 12
#define Relayout1 14
#define Relayin2 27
#define Relayout2 26

int RelayState = -1; // default state "don't move"
uint8_t data[2];

// MAC Addresses
// GCS: A0:B7:65:25:C6:7C
uint8_t masterMacAddress[] = {0xA0, 0xB7, 0x65, 0x25, 0xD4, 0xBC};
uint8_t droneMacAddress[] = {0xA0, 0xB7, 0x65, 0x26, 0xBE, 0x84};
int extended = 0;


// triggered every time ESP sends data via ESP-NOW
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Send status:");
    if (status == ESP_NOW_SEND_SUCCESS) {
      Serial.println("Delivery Success");
    } else {
      Serial.println("Delivery Fail");
    }
}

// Set relays based on state
void relaystate(int state) {
  RelayState = state;
  switch (RelayState) {
    case 3: // Retrack side
      digitalWrite(Relayin1, LOW);
      digitalWrite(Relayout1, HIGH);
      Serial.println("Retract side");
      break;
    case 2: // Retract back
      digitalWrite(Relayin2, LOW);
      digitalWrite(Relayout2, HIGH);
      Serial.println("Retract back");
      break;
    case 1: // Extend side
      digitalWrite(Relayin1, HIGH);
      digitalWrite(Relayout1, LOW);
      Serial.println("Extend side");
      break;
    case 0: // Extend back
      digitalWrite(Relayin2, HIGH);
      digitalWrite(Relayout2, LOW);
      Serial.println("Extend back");
      break;
    case -1: // Don't move
      digitalWrite(Relayin1, HIGH);
      digitalWrite(Relayout1, HIGH);
      digitalWrite(Relayin2, HIGH);
      digitalWrite(Relayout2, HIGH);
      Serial.println("Don't move");
      break;
    default:
      break;
  }
}

// Triggers when gcs esp receives command from esp
void onDataReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  Serial.println("Data received");

  // checks number of bytes received
  if (len != 2) {
    Serial.println("Invalid data length");
    return;
  }

  // copies 2 bytes from incomingData into command variable
  int command;
  memcpy(&command, incomingData, sizeof(command));

  if (command == 1) {
    Serial.println("Received: Retract Command");
    relaystate(2);    // Retract Back
    relaystate(3);      //Retract Side
    delay(23000);

    // send integer signal (0 = done) to Master
    int doneSignal = 0;
    esp_err_t result = esp_now_send(masterMacAddress, (uint8_t *)&doneSignal, sizeof(doneSignal));
    if (result == ESP_OK) {
      Serial.println("Sent DONE signal to master.");
    } else {
      Serial.println("Failed to send DONE signal.");
    }

    extended = 0;
    relaystate(-1);   // Set relay to no movement

  } else if (command == 0 && extended == 0) {
      Serial.println("Received: Extend Command");
      extended = 1;
      for (int i = 0; i < 9100; i++) {
        relaystate(1);
        delay(1);
        relaystate(-1);
        delay(1);
        }    // Wait 17.5 seconds
        relaystate(0);
        delay(4000);
//       for (int i = 0; i < 2000; i++) {
//        relaystate(0);
//        delay(1);
//        relaystate(-1);
//        delay(1);
//        }    // Wait 4 seconds

    // send integer signal (0 = done) to Master
    int doneSignal = 0;
    esp_err_t result = esp_now_send(masterMacAddress, (uint8_t *)&doneSignal, sizeof(doneSignal));
    if (result == ESP_OK) {
      Serial.println("Sent DONE signal to master.");
    } else {
      Serial.println("Failed to send DONE signal.");
    }

    relaystate(-1);   // Set relay to no movement

  } else if (command == -2) { // Reset
      relaystate(3);    // Retract
      relaystate(2);
      delay(23000);      // Wait 23 seconds
      relaystate(-1);     // Don't move
  } else {
      Serial.println("Unknown command");
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(Relayin1, OUTPUT);
  pinMode(Relayout1, OUTPUT);
  pinMode(Relayin2, OUTPUT);
  pinMode(Relayout2, OUTPUT);

  // default relay state (all off)
  relaystate(-1);
  relaystate(3);    // Retract
  relaystate(2);
  delay(23000);
  relaystate(-1);

  /*
  The circuit is an active low
  Default state for pins will be high so relays
  have no power floating through them
   */

  WiFi.mode(WIFI_STA); // Set ESP to station mode

  if (esp_now_init() != ESP_OK){
    Serial.println("Error initializing ESP-NOW");
  } else {
    Serial.println("ESP-NOW initialized successfully");
  }

  esp_now_register_recv_cb(onDataReceive);
  esp_now_register_send_cb(onDataSent);

  // Add peers
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add master peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add master peer: ");
  }

  // Add drone peer
  memcpy(peerInfo.peer_addr, droneMacAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add drone peer: ");
  }
  
}

void loop() {
  
}