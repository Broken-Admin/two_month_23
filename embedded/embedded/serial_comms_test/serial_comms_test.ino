#define ACTIVE_IDLE_PIN 15

void setup() {
  // Built in LED
  pinMode(LED_BUILTIN, OUTPUT);
  // Status LED
  pinMode(ACTIVE_IDLE_PIN, OUTPUT);

  // Start as idle
  digitalWrite(ACTIVE_IDLE_PIN, HIGH);

  // Configure USB serial
  // Value provided is ignored in favor for the USB default
  Serial.begin(115200);
  // Sleep for one second
  delay(1000);

  // Halt until the Serial line is ready
  
  /* while(!Serial.availableForWrite()) {
    // Leave idle LED off until the serial is open
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);
  } */
  
  // Turn LED off after serial initialization
  digitalWrite(LED_BUILTIN, HIGH);

  delay(500);

  digitalWrite(ACTIVE_IDLE_PIN, LOW);

  while(!Serial);

  // Write back
  delay(1000);
  Serial.println("Initalized.");
  delay(1000);
}

void loop() {

  bool idleStatus = digitalRead(ACTIVE_IDLE_PIN) == HIGH;

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    Serial.println(command);

    if (command == "led_on") {
      digitalWrite(LED_BUILTIN, HIGH);
    } else if (command == "led_off") {
      digitalWrite(LED_BUILTIN, LOW);
    } else if (command == "ping") {
      Serial.println("pong");
    } else if (command == "time") {
      Serial.println(millis());
    } else if (command == "idle") {
      // Idle - blink the built-in LED every 500 milliseconds
      if (!idleStatus) {
        Serial.println("idling");
        digitalWrite(ACTIVE_IDLE_PIN, HIGH);
      } else {
        Serial.println("stopping idle");
        digitalWrite(ACTIVE_IDLE_PIN, LOW);
      }
    }
  }

  // Perform idle blink every 500 milliseconds
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
