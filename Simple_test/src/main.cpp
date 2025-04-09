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
// Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno2 = Adafruit_BNO055(56, 0x29);

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

  if (!bno1.begin()) {
    Serial.println("BNO055 #1 not detected!");
    while (1);
  }
  if (!bno2.begin()) {
    Serial.println("BNO055 #2 not detected at 0x29!");
    while (1);
  }

  if (!mpu.begin()) {
    Serial.println("Initialization failed");
    return;
  }
  Serial.println("Sensors is here...");


  delay(1000);
  bno1.setExtCrystalUse(true);  // Use external crystal for better accuracy
  bno2.setExtCrystalUse(true);

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.println();

}

void loop() {
  imu::Vector<3> euler1 = bno1.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> euler2 = bno2.getVector(Adafruit_BNO055::VECTOR_EULER);

  Serial.print("BNO055 Sensor 1: ");
  Serial.print("X=");
  Serial.print(euler1.x());
  Serial.print(" Y=");
  Serial.print(euler1.y());
  Serial.print(" Z=");
  Serial.println(euler1.z());

  uint8_t system1, gyro1, accel1, mag1 = 0;
  bno1.getCalibration(&system1, &gyro1, &accel1, &mag1);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system1, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro1, DEC);
  Serial.print(" Accel=");
  Serial.print(accel1, DEC);
  Serial.print(" Mag=");
  Serial.println(mag1, DEC);


  Serial.print("BNO055 Sensor 2: ");
  Serial.print("X=");
  Serial.print(euler2.x());
  Serial.print(" Y=");
  Serial.print(euler2.y());
  Serial.print(" Z=");
  Serial.println(euler2.z());

  uint8_t system2, gyro2, accel2, mag2 = 0;
  bno2.getCalibration(&system2, &gyro2, &accel2, &mag2);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system2, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro2, DEC);
  Serial.print(" Accel=");
  Serial.print(accel2, DEC);
  Serial.print(" Mag=");
  Serial.println(mag2, DEC);

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
