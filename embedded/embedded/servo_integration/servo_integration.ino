#include <Servo.h>

// #define GREEN_LED 2
// #define RED_LED 3
// #define BLUE_LED 4

// FIRST SERVO PIN
#define J1 20
#define J2 21
#define J3 22
#define J4 26
#define J5 27
#define J6 28

// Continuous Servo Min and Max angle (speeds), and zero point
#define CONT_MIN 72 // Clockwise
#define CONT_MAX 113 // Counterclockwise
#define CONT_ZERO 95 // Zero angle

// High Torque 270 Servo Min and Max angle
#define HT_MIN 0
#define HT_MAX 75

// Low Torque 180 Servo angles
#define LT_TOP 180 // 12:00
#define LT_SIDE 90 // 3:00
#define LT_DOWN 0 // 6:00

// Moving UP the arm
Servo Joint1; // Cont
Servo Joint2; // 270
Servo Joint3; // Cont
Servo Joint4; // 180
Servo Joint5; // Cont
Servo Joint6; // 180

char *delim = " \n\t";

void setup() {
    // Built in LED
    pinMode(LED_BUILTIN, OUTPUT);

    // Configure USB serial
    // Value provided is ignored in favor for the USB default
    Serial.begin(115200);
    // Sleep for one second
    while(!Serial);

    Joint1.attach(J1);
    Joint2.attach(J2);
    Joint3.attach(J3);
    Joint4.attach(J4);
    Joint5.attach(J5);
    Joint6.attach(J6);
    // Status LEDs
    // pinMode(GREEN_LED, OUTPUT);
    // pinMode(RED_LED, OUTPUT);
    // pinMode(BLUE_LED, OUTPUT);

    // End Effector
    Joint6.write(90);
}

void loop() {
    Serial.println("Pending Input");

    if (Serial.available()) {
        String serialData = Serial.readStringUntil('\n');
        char charData[serialData.length()];
        serialData.toCharArray(charData, serialData.length());
        // Parse the command
        char *command = strtok(charData, delim);

        String arduCommand(command);
        Serial.println(arduCommand);

        if (strcmp(command, "led_on") == 0) {
            digitalWrite(LED_BUILTIN, HIGH);
        } else if (strcmp(command, "led_off") == 0) {
            digitalWrite(LED_BUILTIN, LOW);
        } else if (strcmp(command, "ping") == 0) {
            Serial.println("pong");
        } else if (strcmp(command, "servo") == 0) {
        
            char *servo_str = strtok(NULL, delim);
            char *angle_str = strtok(NULL, delim);

            int servo_n = strtol(servo_str, NULL, 10);
            int angle_n = strtol(angle_str, NULL, 10);

            // Serial.println(servo_n);
            // Serial.println(angle_n);
            if(servo_n < 1 || servo_n > 6) {
                Serial.print("[WARNING] Attempted to move unknown servo number ");
            } else {
                Serial.print("[INFO] Attemping to move servo number ");
            }
            Serial.println(servo_n);

            switch(servo_n) {
                case 1: // Base: Continuous
                    // Serial.println("BASE 1");
                    if(angle_n == 0) angle_n = 95;
                    angle_n = constrain(angle_n, CONT_MIN, CONT_MAX);
                    Joint1.write(angle_n);
                    break;
                case 2: // Joint: High Torque 270
                    // Serial.println("HT 2");
                    // angle_n = constrain(angle_n, HT_MIN, HT_MAX);
                    Joint2.write(angle_n);
                    break;
                case 3: // Joint: LT 180
                    // Serial.println("LT 3");
                    Joint3.write(angle_n);
                    break;
                case 4: // Joint: LT 180
                    // Serial.println("LT 4");
                    Joint4.write(angle_n);
                    break;
                case 5: // Joint: Continuous
                    // Serial.println("CONT 5");
                    if(angle_n == 0) angle_n = 95;
                    angle_n = constrain(angle_n, CONT_MIN, CONT_MAX);
                    Joint5.write(angle_n);
                    break;
                case 6:
                    // Serial.println("LT 6");
                    Joint6.write(angle_n);
                    break;
                default:
                    break;
            }
        }
    }

    digitalWrite(GREEN_LED, HIGH);
    delay(250);
    digitalWrite(RED_LED, HIGH);
    delay(250);
    digitalWrite(BLUE_LED, HIGH);
    delay(250);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    delay(250);
}