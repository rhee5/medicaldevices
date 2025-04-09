#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

#include <stdint.h>
#include <stddef.h>

#include "WString.h"
#include "Printable.h"


// Create sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_MPU6050 mpu;

// arduino to mpu6050 pins (4 pins)
// ground to ground
// 5V to vcc
// SCL to SCL
// SDA to SDA

// arduino to bno055 pins (4 pins)
// ground to ground
// 5V to vcc
// SCL to A5
// SDA to A4


void setup() {
  Serial.begin(9600);
  while (!Serial) { // Wait for serial
    delay(100);
  }; 

  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }
  if (!mpu.begin()) {
    Serial.println("Initialization failed");
    return;
  }
  Serial.println("Sensors is here...");


  delay(1000);
  bno.setExtCrystalUse(true);  // Use external crystal for better accuracy
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.println();

}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  Serial.print("BNO055 Sensor: ");
  Serial.print("X=");
  Serial.print(euler.x());
  Serial.print(" Y=");
  Serial.print(euler.y());
  Serial.print(" Z=");
  Serial.println(euler.z());

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);


  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  Serial.println("MPU6050 Sensor: ");
  Serial.print("acceleration ");
  Serial.print(a.acceleration.x);
  Serial.print(" ");
  Serial.print(a.acceleration.y);
  Serial.print(" ");
  Serial.print(a.acceleration.z);
  Serial.print("\n");

  Serial.print("gyro ");
  Serial.print(g.gyro.x);
  Serial.print(" ");
  Serial.print(g.gyro.y);
  Serial.print(" ");
  Serial.print(g.gyro.z);
  Serial.print("\n");

  Serial.print("temperature ");
  Serial.print(t.temperature);
  Serial.println("\n");
  Serial.println("\n");

  delay(1000);
}
