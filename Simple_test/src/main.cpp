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

Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;

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

  if (!mpu1.begin(0x68)) {
    Serial.println("mpu1 Initialization failed");
    return;
  }
  if (!mpu2.begin(0x69)) {
    Serial.println("mpu2 Initialization failed");
    return;
  }
  Serial.println("Sensors is here...");


  delay(1000);
  bno1.setExtCrystalUse(true);  // Use external crystal for better accuracy
  bno2.setExtCrystalUse(true);

  mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu1.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu1.setFilterBandwidth(MPU6050_BAND_5_HZ);

  mpu2.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu2.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu2.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.println();

}

void loop() {
  imu::Vector<3> euler1 = bno1.getVector(Adafruit_BNO055::VECTOR_EULER);

  Serial.println("=== BNO055 Sensor 1 ===");
  Serial.print("Euler Angles (deg) -> X: ");
  Serial.print(euler1.x());
  Serial.print(" | Y: ");
  Serial.print(euler1.y());
  Serial.print(" | Z: ");
  Serial.println(euler1.z());

  uint8_t system1=0, gyro1=0, accel1=0, mag1=0;
  bno1.getCalibration(&system1, &gyro1, &accel1, &mag1);
  Serial.print("Calibration -> Sys: ");
  Serial.print(system1);
  Serial.print(" | Gyro: ");
  Serial.print(gyro1);
  Serial.print(" | Accel: ");
  Serial.print(accel1);
  Serial.print(" | Mag: ");
  Serial.println(mag1);
  Serial.println();

  imu::Vector<3> euler2 = bno2.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.println("=== BNO055 Sensor 2 ===");
  Serial.print("Euler Angles (deg) -> X: ");
  Serial.print(euler2.x());
  Serial.print(" | Y: ");
  Serial.print(euler2.y());
  Serial.print(" | Z: ");
  Serial.println(euler2.z());

  uint8_t system2, gyro2, accel2, mag2 = 0;
  bno2.getCalibration(&system2, &gyro2, &accel2, &mag2);
  Serial.print("Calibration -> Sys: ");
  Serial.print(system2);
  Serial.print(" | Gyro: ");
  Serial.print(gyro2);
  Serial.print(" | Accel: ");
  Serial.print(accel2);
  Serial.print(" | Mag: ");
  Serial.println(mag2);
  Serial.println();

  sensors_event_t a1, g1, t1;
  mpu1.getEvent(&a1, &g1, &t1);

  Serial.println("=== MPU6050 Sensor 1 ===");
  Serial.print("Acceleration (m/s^2) -> X: ");
  Serial.print(a1.acceleration.x);
  Serial.print(" | Y: ");
  Serial.print(a1.acceleration.y);
  Serial.print(" | Z: ");
  Serial.println(a1.acceleration.z);

  Serial.print("Gyro (rad/s) -> X: ");
  Serial.print(g1.gyro.x);
  Serial.print(" | Y: ");
  Serial.print(g1.gyro.y);
  Serial.print(" | Z: ");
  Serial.println(g1.gyro.z);

  Serial.print("Temperature (°C): ");
  Serial.println(t1.temperature);
  Serial.println();


  sensors_event_t a2, g2, t2;
  mpu1.getEvent(&a2, &g2, &t2);
  Serial.println("=== MPU6050 Sensor 2 ===");
  Serial.print("Acceleration (m/s^2) -> X: ");
  Serial.print(a2.acceleration.x);
  Serial.print(" | Y: ");
  Serial.print(a2.acceleration.y);
  Serial.print(" | Z: ");
  Serial.println(a2.acceleration.z);

  Serial.print("Gyro (rad/s) -> X: ");
  Serial.print(g2.gyro.x);
  Serial.print(" | Y: ");
  Serial.print(g2.gyro.y);
  Serial.print(" | Z: ");
  Serial.println(g2.gyro.z);

  Serial.print("Temperature (°C): ");
  Serial.println(t2.temperature);
  Serial.println();
  Serial.println("========================================\n");


  delay(1000);
}
