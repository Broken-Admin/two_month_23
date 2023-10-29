#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>    

// #define LED_BUILTIN 25

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;


void setup() {
  // Attempt to initialize serial with USB baud rate
  Serial.begin(115200);

  // Pause the program until serial is initialized
  while (!Serial);

  // Display debug informaiton
  Serial.println("Adafruit BMP380 / BMP390 test");

  delay(2000);

  // Attempt to begin communication
  if (!bmp.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &Wire)) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP380 sensor, check wiring!");
    // Pause the program, ideally blink some error LED
    // or communicate the error via ROS2 to the basestation
    pinMode(LED_BUILTIN, OUTPUT);
    while (1) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
    }
  }

  Serial.println("Preparing to read data.");

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop() {
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  delay(2000);
}