#include <Arduino.h>
#include <String.h>

#define OPEN_RST 18
#define OPEN_TX 16
#define OPEN_RX 17

// TX, RX
UART Openlogger(16, 17);

void setup() {
  // Serial.begin(9600);

  // while(!Serial);

  // Serial.println("Initializing");

  pinMode(LED_BUILTIN, OUTPUT);

  // put your setup code here, to run once:
  Openlogger.begin(9600);

  while(!Openlogger);

  // Serial.println("Openlogger serial line ready");

  // Reset and begin comms with Openlog
  digitalWrite(OPEN_RST, LOW);
  delay(100);
  digitalWrite(OPEN_RST, HIGH);

  // Openlog is writing to a file and needs to be put into command mode
  // Serial.write("Openlog is writing to file.");
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println("Openlogger: Writing data");

  Openlogger.println("Fuck it we ball");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
}
