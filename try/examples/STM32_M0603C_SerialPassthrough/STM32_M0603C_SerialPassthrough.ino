#include <DDSM210.h>


DDSM_CTRL dc;

// device settings.
#define M0603C_RX PA3
#define M0603C_TX PA2

// device settings.
#define USB_RX PA11
#define USB_TX PA12

//                            RX          TX
HardwareSerial Serial_M0603C(M0603C_RX, M0603C_TX);
//                          RX          TX
HardwareSerial Serial_USB(USB_RX, USB_TX);
void setup() {
  Serial_USB.begin(115200);
  Serial_M0603C.begin(DDSM_BAUDRATE);
}

void loop() {
  if (Serial_USB.available()) {              // If anything comes in Serial (USB),
    Serial_M0603C.write(Serial_USB.read());  // read it and send it out Serial_M0603C
  }

  if (Serial_M0603C.available()) {           // If anything comes in Serial_USB1
    Serial_USB.write(Serial_M0603C.read());  // read it and send it out Serial (USB)
  }
}
