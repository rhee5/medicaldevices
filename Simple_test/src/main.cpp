#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


#include <stdint.h>
#include <stddef.h>

#include "WString.h"
#include "Printable.h"

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(100);
  }
  if (!mpu.begin()) { //sda scl pins ourself???
    Serial.println("Initialization failed");
    return;
  }
  Serial.println("Sensor is here...");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.println();
}

void loop() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  // Serial.println(a.acceleration.x);
  // Serial.println(a.acceleration.y);
  // Serial.println(a.acceleration.z);
  // Serial.println("\n");
  Serial.println(g.gyro.x);
  Serial.println(g.gyro.y);
  Serial.println(g.gyro.z);
  Serial.println("\n");
  // Serial.println(t.temperature);
  // Serial.println("\n");
  delay(1000);

}