/***************************************************************************
  This is a library for the BMP3XX temperature & pressure sensor

  Designed specifically to work with the Adafruit adafruit_i2c Breakout
  ----> http://www.adafruit.com/products/3966

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

#define I2C_SDA 14
#define I2C_SCL 15

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

// https://github.com/arduino/ArduinoCore-mbed/blob/fee275d4c151ac16289b636944169c5a8e773a9b/libraries/Wire/Wire.h#L33y
MbedI2C adafruit_i2c(I2C_SDA , I2C_SCL);

void setup() {
  // Attempt to initialize serial with USB baud rate
  Serial.begin(115200);

  // Pause the program until serial is initialized
  while (!Serial);

  // Display debug informaiton
  Serial.println("Adafruit BMP380 / BMP390 test");

  // Attempt to begin communication
  if (!bmp.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &adafruit_i2c)) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    // Pause the program, ideally blink some error LED
    // or communicate the error via ROS2 to the basestation
    while (1);
  }

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