#include <Arduino.h>
#include <String.h>

// Define an Alias
#define Openlogger Serial1

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

unsigned long currentMillis = 0;

void loop() {
  currentMillis = millis();

  Openlogger.println(currentMillis);
  if(Openlogger.available()) {
    Serial.println(Openlogger.read());
  }
  Serial.print("Wrote to Openloger:");
  Serial.println(currentMillis);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
}
