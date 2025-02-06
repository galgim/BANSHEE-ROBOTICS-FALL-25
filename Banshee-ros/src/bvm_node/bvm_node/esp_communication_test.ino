
/*
void sendData(String tag, const void* data, size_t dataSize) {
    Serial.println(tag);
    Serial.write((const uint8_t*)data, dataSize);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(10000);
}

void loop() {
    float data[] = {1.24, 4.01, 3.99, 2.65, 1.24, 4.01, 3.99, 2.65};
    sendData("Voltage", data, sizeof(data));
    delay(1000);
    float data[] = {5.22, 3.99, 6.98};
    sendData("OtherInfo", data, sizeof(data));
    delay(1000);
}
*/