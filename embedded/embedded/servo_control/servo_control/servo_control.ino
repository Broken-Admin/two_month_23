#include <Servo.h>

#define IDLE_PIN 15
#define END_EFFECTOR_PIN 28

#define METAL_GEAR_ZERO 0

Servo end_effector;

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
  delay(1000);

  // Power on the LED when serial is seen
  digitalWrite(IDLE_PIN, HIGH);

  // Turn LED off after serial initialization
  digitalWrite(LED_BUILTIN, HIGH);

  delay(500);

  // PWM pin
  // pinMode(END_EFFECTOR_PIN, OUTPUT);

  // Set up the end effector servo
  end_effector.attach(END_EFFECTOR_PIN, 500, 2500);

  end_effector.write(-90);
  delay(1000);
  end_effector.write(90);

  // Write back
  delay(1000);
  Serial.println("Initalized.");
  delay(1000);
}

void loop() {
  bool idleStatus = digitalRead(IDLE_PIN) == HIGH;

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
        digitalWrite(IDLE_PIN, HIGH);
      } else {
        Serial.println("stopping idle");
        digitalWrite(IDLE_PIN, LOW);
      }
    } else if (command == "pwm") {
      Serial.println("Write what angle value?");
      while (!Serial.available()) {}

      // duty cycle, between 0 (always off) and 255 (always on)
      String serial_angle = Serial.readStringUntil('\n');
      int angle = serial_angle.toInt();

      end_effector.write(angle+METAL_GEAR_ZERO);

      Serial.print("PWM changing to angle value ");
      Serial.println(angle+METAL_GEAR_ZERO);
      // char *pwm_data_char = strtok(command_char, " ");
      // String pwm_data = String(data_char);
      // String pwm_freq_str = command.substring(4);
      // Serial.println(pwm_freq_str);
      // Serial.println(pwm_data);
      // Process the data, returns 0 if data is invalid
      // int pwm_freq = pwm_freq_str.toInt();
      // Print out the data
      // Serial.println(pwm_freq);

      // Actually write the data
      // analogWrite(END_EFFECTOR_PIN, 200);
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
