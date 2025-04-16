#include <esp_now.h>
#include <WiFi.h>

#define Relayin1 12
#define Relayout1 14
#define Relayin2 27
#define Relayout2 26

String RelayState = "0";

uint8_t masterMacAddress[] = {0xA0, 0xB7, 0x65, 0x25, 0xD4, 0xBC};

// Triggers when gcs esp receives command from drone esp
void onDataReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    char receivedCommand[32] = {0};
    memcpy(receivedCommand, incomingData, len);
    receivedCommand[len] = '\0'; 

    Serial.println(receivedCommand);

    if (strcmp(receivedCommand, "done") == 0) {
        Serial.println("Received signal from drone");
        RelayState = 1;
        sendCommand("done");
    } else {
        Serial.println("Unknown command received.");
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
  Serial.begin(9600);
  //Serial2.begin(115200);
 

  pinMode(Relayin1, OUTPUT);
  pinMode(Relayout1, OUTPUT);
  pinMode(Relayin2, OUTPUT);
  pinMode(Relayout2, OUTPUT);
  /*
  The circuit is an active low
  Default state for pins will be high so relays
  have no power floating through them
   */
  digitalWrite(Relayin1, HIGH);
  digitalWrite(Relayout1, HIGH);
  digitalWrite(Relayin2, HIGH);
  digitalWrite(Relayout2, HIGH);

  esp_now_register_recv_cb(onDataReceive);

  memcpy(peerInfo.peer_addr, masterMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.print("Failed to add peer: ");
  }

}

void loop() {
  // put your main code here, to run repeatedly
  if (Serial.available())
  {
    RelayState = Serial.readString();
  }

//Serial.println("dont move");
Serial.println(RelayState);
relaystate();

}


void relaystate()
{
  //relay retract code
  if(RelayState == "1")
  {


    //linear actuator start moving here
    digitalWrite(Relayin1, LOW);
    digitalWrite(Relayout1, HIGH);

    digitalWrite(Relayin2, LOW);
    digitalWrite(Relayout2, HIGH);
    Serial.print("retract");
  }


  // relay extand code
  if(RelayState == "0")
  {


    //linear actuator start moving here
    digitalWrite(Relayin1, HIGH);
    digitalWrite(Relayout1, LOW);

    digitalWrite(Relayin2, HIGH);
    digitalWrite(Relayout2, LOW);
    Serial.println("extand");

  }

  if(RelayState == "-1")
  {


    //linear actuator start moving here
    digitalWrite(Relayin1, HIGH);
    digitalWrite(Relayout1, HIGH);

    digitalWrite(Relayin2, HIGH);
    digitalWrite(Relayout2, HIGH);
    Serial.println("dont move");

  }