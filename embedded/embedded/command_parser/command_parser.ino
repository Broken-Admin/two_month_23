#include <Servo.h>

#define IDLE_PIN 15

#define CONT_ZERO 95

// FIRST SERVO PIN
#define J1 6
#define J2 7
#define J3 8
#define J4 9
#define J5 10
#define J6 11

class ContinuousServo : public Servo {
  ContinuousServo(int ) 
}


Servo Joint1; // Cont
Servo Joint2; // 270
Servo Joint3; // Cont
Servo Joint4; // 180
Servo Joint5; // Cont
Servo Joint6; // 180

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

  // cont servo
  Joint1.attach(J1);
  Joint5.attach(J5);

  // 180 servo
  Joint3.attach(J3, 500, 2500);
  Joint4.attach(J4, 500, 2500);
  Joint6.attach(J6, 500, 2500); // Stalls at 90deg

  // 270 servo
  Joint2.attach(J2, 500, 2500);
}

void loop() {
  bool idleStatus = digitalRead(IDLE_PIN) == HIGH;

  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
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
    } 
    /* 180 SERVO */
    else if (command == "180") {
      Serial.println("Write what angle value?");
      while (!Serial.available()) {}

      // duty cycle, between 0 (always off) and 255 (always on)
      String serial_angle = Serial.readStringUntil('\n');
      int angle = serial_angle.toInt();


      Serial.println("What 180 servo? [2,4,6]");
      while (!Serial.available()) {}

      String servo_num = Serial.readStringUntil('\n');
      int servo = servo_num.toInt();

      Serial.println(servo);

      switch(servo) {
        case 2:
          Joint2.write(angle);
          break;
        case 4:
          Joint4.write(angle);
          break;
        case 6:
          Joint6.write(angle);
          break;
        default:
          Serial.println("Tried to update invalid 180 degree servo");
          break;
      }
    }
    /* Cont SERVO */
    else if (command == "cont") {
      Serial.println("Write what continuous value?");
      while (!Serial.available()) {}


      // duty cycle, between 0 (always off) and 255 (always on)
      String serial_angle = Serial.readStringUntil('\n');
      int angle = serial_angle.toInt() + CONT_ZERO;

      Serial.println("What continuous servo? [1,3,5]");
      while (!Serial.available()) {}

      String servo_num = Serial.readStringUntil('\n');
      int servo = servo_num.toInt();
      
      Serial.println(servo);

      switch(servo) {
        case 1:
          Joint1.write(angle);
          break;
        case 3:
          Joint3.write(angle);
          break;
        case 5:
          Joint5.write(angle);
          break;
      }

      Serial.print("PWM changing to angle value ");
      Serial.println(angle);
  
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
