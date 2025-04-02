#include <MPU6050.h>
MPU6050 mpu;

#include "DHT.h"
#define DHTPIN 3
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
float t;
float tt;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initializing I2C devices...");
  mpu.initialize();
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  dht.begin();
}

void loop() {
  t = mpu.getTemperature();
  tt = dht.readTemperature();
  mpu.CalibrateAccel(10);
  mpu.CalibrateGyro(10);
  Serial.print(t / 340 + 35);
  Serial.print(",\t");
  Serial.print(tt);
  Serial.print(",\t");
  mpu.PrintActiveOffsets();
  delay(120000);  //Ein Delay wurde nur im zweiten und dritten Versuch genutzt
}
