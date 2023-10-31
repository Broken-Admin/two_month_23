// Servo Control
#include <Servo.h>
// I2C
#include <Wire.h>
// Sensor Libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
// Sensor-associated libraries  
#include <utility/imumaths.h>

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

// Interval definitions
#define BNO055_CHECK_INTERVAL 2500
#define BMP388_CHECK_INTERVAL 2000
#define READY_INTERVAL 1800

// BMP constant definitions
#define SEALEVELPRESSURE_HPA (1013.25)

// Openlogger
#define Openlogger Serial1

// Moving UP the arm
Servo Joint1; // Cont
Servo Joint2; // 270
Servo Joint3; // Cont
Servo Joint4; // 180
Servo Joint5; // Cont
Servo Joint6; // 180

char delim[] = " \n\t";

// BNO055 definitions
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_A, &Wire);
bool bnoAttached = false;

// BMP388 definitions
Adafruit_BMP3XX bmp;
bool bmpAttached = false;


// Delay definitions
unsigned long previousBNOMillis = 0;
unsigned long previousBMPMillis = 0;
unsigned long previousReadyMillis = 0;
unsigned long currentMillis = 0;
int polledSensor;

// Output String
String dataout = "";

void setup() {
    // Built in LED
    pinMode(LED_BUILTIN, OUTPUT);

    // Configure USB serial
    // Value provided is ignored in favor for the USB default
    Serial.begin(115200);

    /* Openlogger */
    // Configure Openlogger serial
    Openlogger.begin(9600);
    // Add header to Openlogger
    Openlogger.println("Orientation X,Orientation Y,Orientation Z,Lin Accel X, Lin Accel Y, Lin Accel Z,Temperature (*C),Pressure (hPa),Approx Altitude (m)");

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
    // Joint1.write(95); // Continuous
    Joint2.write(0); // HT 
    Joint3.write(0); // LT
    Joint4.write(0); // LT
    // Joint5.write(95); // Continuous
    Joint6.write(0); // LT

    // Wait for a ping
    while(true) {
        if(Serial.available()) {
            String command = Serial.readStringUntil('\n');
            command.trim();
            if(command == "ping") {
                // Return the message 
                Serial.println("pong");
                // Break out of the catch loop
                break;
            }
        }
    }

    // BNO055 Integration
    /* Initialise the sensor */
    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.println("[ERROR] No BNO055 detected");
    } else {
        bnoAttached = true;
        bno.setExtCrystalUse(true);
    }

    if(!bmp.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &Wire)) {
        Serial.println("[ERROR] No BMP380 detected");
    } else {
        bmpAttached = true;
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    }
}

void loop() {
    polledSensor = 0;

    // Update interval
    currentMillis = millis();

    // Reset data string, if within a time interval
    if(currentMillis - previousBNOMillis >= BNO055_CHECK_INTERVAL || currentMillis - previousBMPMillis >= BMP388_CHECK_INTERVAL) {
        dataout = "";
    }

    // Let user know dataready
    if(currentMillis - previousReadyMillis >= READY_INTERVAL) {
        Serial.println("[GOOD] Ready for Input");
        previousReadyMillis = currentMillis;
    }


    // Command processing
    if (Serial.available()) {
        String serialData = Serial.readStringUntil('\n');
        char charData[serialData.length()];
        serialData.toCharArray(charData, serialData.length());
        // Parse the command, only properly parses if using crlf (\n\r) or a trailing space
        char *command = strtok(charData, delim);

        // Bring the command to lower case
        String arduCommand = String(command);
        arduCommand.toLowerCase();
        arduCommand.toCharArray(command, arduCommand.length()+1);

        // Halt until "ping"
        // while(strcmp(command, "ping") != 0);

        if (strcmp(command, "led_on") == 0) {
            digitalWrite(LED_BUILTIN, HIGH);
        } else if (strcmp(command, "led_off") == 0) {
            digitalWrite(LED_BUILTIN, LOW);
        } else if (strcmp(command, "ping") == 0) {
            Serial.println("pong");
        } else if (strcmp(command, "hcf") == 0) {
            // Halt and Catch Fire
            Serial.println("[WARNING] HALT");
            while(1);
        } else if (strcmp(command, "servo") == 0) {
        
            char *servo_str = strtok(NULL, delim);
            char *angle_str = strtok(NULL, delim);

            int servo_n = strtol(servo_str, NULL, 10);
            int angle_n = strtol(angle_str, NULL, 10);
            // Servo Number
            if(servo_n < 1 || servo_n > 6) {
                Serial.print("[WARNING] Attempted to move unknown servo number ");
            } else {
                Serial.print("[GOOD] Attemping to move servo number ");
            }
            Serial.print(servo_n);
            // Angle
            Serial.print(" to an angle of ");
            Serial.println(angle_n);

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

        // If the command does not control a servo
        // output
        if(strcmp(command, "servo") != 0) {
            Serial.println(command);
        }
    }
    
    // Regularly check for data and send a message
    if(bnoAttached && currentMillis - previousBNOMillis >= BNO055_CHECK_INTERVAL) {
        //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
        sensors_event_t orientationData, linearAccelData;
        // getEvent always returns true, there's no needs 
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

        dataout += String(orientationData.orientation.x) + ",";
        dataout += String(orientationData.orientation.y) + ",";
        dataout += String(orientationData.orientation.z) + ",";

        // Serial.println(String(orientationData.orientation.x) + String(orientationData.orientation.y) + String(orientationData.orientation.z));

        dataout += String(linearAccelData.acceleration.x) + ",";
        dataout += String(linearAccelData.acceleration.y) + ",";
        dataout += String(linearAccelData.acceleration.z) + ",";

        // Serial.println(String(linearAccelData.acceleration.x) + String(linearAccelData.acceleration.y) + String(linearAccelData.acceleration.z));

        printEvent(&orientationData);
        printEvent(&linearAccelData);

        // int8_t boardTemp = bno.getTemp();
        // Serial.print(F("[INFO] Temperature (deg Celcius): "));
        // Serial.println(boardTemp);

        // Reset the interval
        previousBNOMillis = currentMillis;

        // Update openlogger write flag
        polledSensor++;
    } else if(currentMillis - previousBNOMillis >= BNO055_CHECK_INTERVAL) {
        // Empty columns when the BNO is not attached
        dataout += ",,,,,,";
        // Reset the interval
        previousBNOMillis = currentMillis;
        // Don't update the write flag
    }

    if (bmpAttached && currentMillis - previousBMPMillis >= BMP388_CHECK_INTERVAL) {
        // Attempt to get a reading
        if (!bmp.performReading()) {
            Serial.println("[ERROR] BMP388 failed to take reading");
            dataout += ",,";
        } else {
            Serial.print("[INFO] Temperature (*C): ");
            Serial.println(bmp.temperature);
            dataout += String(bmp.temperature) + ",";

            Serial.print("[INFO] Pressure (hPa): ");
            Serial.println(bmp.pressure / 100.0);
            dataout += String(bmp.pressure / 100.0) + ",";

            Serial.print("[INFO] Approx. Altitude (m): ");
            Serial.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));
            dataout += String(bmp.readAltitude(SEALEVELPRESSURE_HPA));
            
            // Update openlogger write flag
            polledSensor++;
        }
        
        // Reset the interval
        previousBMPMillis = currentMillis;
    } else if(currentMillis - previousBNOMillis >= BNO055_CHECK_INTERVAL) {
        // Empty columns when the BMP is not attached
        dataout += ",,";
        // Reset the interval
        previousBMPMillis = currentMillis;
        // Don't update the write flag
    }

    // These are not connected or utilized
    // digitalWrite(GREEN_LED, HIGH);
    // delay(250);
    // digitalWrite(RED_LED, HIGH);
    // delay(250);
    // digitalWrite(BLUE_LED, HIGH);
    // delay(250);
    // digitalWrite(GREEN_LED, LOW);
    // digitalWrite(RED_LED, LOW);
    // digitalWrite(BLUE_LED, LOW);
    // delay(250);

    // If data was updated, send it to the Openlogger
    // This is ONLY run if the sensors are successfully polled 
    if(polledSensor == 2) {
        Openlogger.println(dataout);
        // Serial.println(dataout);
    }

    // Process data and perform checks every quarter second
    delay(250);
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("[INFO] Orient: ");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("[INFO] Linear: ");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }

  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(z);
}