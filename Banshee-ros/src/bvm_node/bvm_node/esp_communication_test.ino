/*

void setup() {
  // Start the serial communication at 9600 baud rate
  Serial.begin(115200);
}

void loop() {
  // Check if data is available to read from the serial buffer
  if (Serial.available() > 0) {
    // Read the incoming byte:
    String incomingData = Serial.readStringUntil('\n');  // Read until newline
    Serial.print("Received: ");
    Serial.println(incomingData);  // Print received data

    // Respond back to the sender (e.g., your Python script)
    String response = "Message received: " + incomingData;
    Serial.println(response);  // Send a response back to the serial
  }

  delay(1000);
}

*/