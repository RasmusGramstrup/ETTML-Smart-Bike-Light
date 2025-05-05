/*
  Using the BNO080 IMU
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  This example shows how to output the parts of the calibrated gyro.

  It takes about 1ms at 400kHz I2C to read a record from the sensor, but we are polling the sensor continually
  between updates from the sensor. Use the interrupt pin on the BNO080 breakout to avoid polling.

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 115200 baud to serial monitor.
*/


#include "Particle.h"

SYSTEM_MODE(MANUAL);

SerialLogHandler logHandler;

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

BNO080 myIMU;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO080 Read Example");

  Wire.begin();

  if (myIMU.begin() == false)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
   // while (1);
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableAccelerometer(10); //Send data update every 10ms / 100Hz
  myIMU.enableGyro(10);         //Send data update every 10ms / 100Hz
  myIMU.enableMagnetometer(10); //Send data update every 10ms / 100Hz

  Serial.println(F("Accelerometer enabled"));
  Serial.println(F("Output in form x, y, z, in m/s^2"));
}

void loop()
{
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    float x = myIMU.getAccelX();
    float y = myIMU.getAccelY();
    float z = myIMU.getAccelZ();

    Serial.println("Accelerometer:");
    Serial.print(x, 2);
    Serial.print(F(","));
    Serial.print(y, 2);
    Serial.print(F(","));
    Serial.print(z, 2);
    Serial.print(F(","));

    Serial.println();

    x = myIMU.getGyroX();
    y = myIMU.getGyroY();
    z = myIMU.getGyroZ();

    Serial.println("Gyroscope:");
    Serial.print(x, 2);
    Serial.print(F(","));
    Serial.print(y, 2);
    Serial.print(F(","));
    Serial.print(z, 2);
    Serial.print(F(","));

    Serial.println();

    x = myIMU.getMagX();
    y = myIMU.getMagY();
    z = myIMU.getAccelZ();

    Serial.println("Magnetometer:");
    Serial.print(x, 2);
    Serial.print(F(","));
    Serial.print(y, 2);
    Serial.print(F(","));
    Serial.print(z, 2);
    Serial.print(F(","));

    Serial.println();
  }
}
