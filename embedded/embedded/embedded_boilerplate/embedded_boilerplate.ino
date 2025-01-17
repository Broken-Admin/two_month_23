#define LED_PIN 25

void setup() {
  // Initialize LED_PIN as an output
  pinMode(LED_PIN, OUTPUT);
  // Turn LED on for initialization
  digitalWrite(LED_PIN, HIGH);

  // Configure USB serial
  // Value provided is ignored in favor for the USB default
  Serial.begin(115200);
  // Sleep for one second
  delay(100);

  // Turn LED off after serial initialization
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "led_on") {
      digitalWrite(LED_PIN, HIGH);
    } else if (command == "led_off") {
      digitalWrite(LED_PIN, LOW);
    } else if (command == "ping") {
      Serial.println("pong");
    } else if (command == "time") {
      Serial.println(millis());
    }
  }
}
