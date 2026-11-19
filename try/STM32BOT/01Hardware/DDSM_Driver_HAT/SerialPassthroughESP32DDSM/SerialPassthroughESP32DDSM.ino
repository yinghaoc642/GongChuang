#define DDSM_RX 18
#define DDSM_TX 19
void setup() {
  Serial.begin(115200);
  	Serial1.begin(115200, SERIAL_8N1, DDSM_RX, DDSM_TX);
}

void loop() {
  if (Serial.available()) {        // If anything comes in Serial (USB),
    Serial1.write(Serial.read());  // read it and send it out Serial1
  }

  if (Serial1.available()) {       // If anything comes in Serial1
    Serial.write(Serial1.read());  // read it and send it out Serial (USB)
  }
}
