#define IDLE_PIN 15

void setup() {
  // Built in LED
  pinMode(LED_BUILTIN, OUTPUT);
  // Status LED
  pinMode(IDLE_PIN, OUTPUT);

  // Start as idle
  digitalWrite(IDLE_PIN, HIGH);

  // Configure USB serial
  // Value provided is ignored in favor for the USB default
  Serial.begin(115200);
  // Sleep for one second
  delay(100);

  // Turn LED off after serial initialization
  digitalWrite(LED_BUILTIN, HIGH);

  delay(500);

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  bool idleStatus = digitalRead(IDLE_PIN) == HIGH;

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "led_on") {
      digitalWrite(LED_BUILTIN, HIGH);
    } else if (command == "led_off") {
      digitalWrite(LED_BUILTIN, LOW);
    } else if (command == "ping") {
      Serial.println("pong");
    } else if (command == "time") {
      Serial.println(millis());
    } else if (command == "idle") {
      if (!idleStatus) {
        Serial.println("idling");
        digitalWrite(IDLE_PIN, HIGH);
      } else {
        Serial.println("stopping idle");
        digitalWrite(IDLE_PIN, LOW);
      }
    }
  }

  if (idleStatus) {
    bool idleLEDStatus = digitalRead(LED_BUILTIN) == HIGH;
    if (!idleLEDStatus) {
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }
    delay(500);
  }
}
