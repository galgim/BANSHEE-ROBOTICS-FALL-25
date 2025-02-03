/*
void setup() {
  // Start the serial communication at 115200 baud rate
  Serial.begin(115200);
}

void loop() {
  // Check if data is available to read from the serial buffer
  if (Serial.available() > 0) {
    // Read the incoming byte:
    String incomingData = Serial.readStringUntil('\n');  // Read until newline
//    Serial.print("Received: ");
//    Serial.println(incomingData);  // Print received data

    // Respond back to the sender (e.g., your Python script)
    byte response[8];
    response[0] = 1.2;
    Serial.write(response, 8);  // Send a response back to the serial
  }

  delay(1000);
}

*/
void setup() {
  Serial.begin(115200);  // Start serial communication
}

void loop() {
  if (Serial.available() > 0) {
    String incomingData = Serial.readStringUntil('\n');  // Read incoming data

    // Define an array of values to send
    int values[] = {10, 20, 30, 40, 50};  
    int size = sizeof(values) / sizeof(values[0]);

    // Send the array over Serial
    Serial.write((byte*)values, size * sizeof(int));  // Send array as raw bytes
  }

  delay(1000);  // Avoid flooding the serial port
}
