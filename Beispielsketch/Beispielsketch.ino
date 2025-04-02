#define MPU
#define OLEDDISPLAY
#define DHTSENSOR
#define MAGNETOMETER

#include "IMU.h"

void setup() {
  setup_mpu();
  setup_display();
  setup_dht();
  setup_compass();
}

void loop() {

  if (Serial.available()) {
    switch (Serial.read()) {
      case 'm':  //Gebe m ein, um die MPU zu kalibrieren
        loop_kalibrierungMPU(); 
        while (Serial.available() && Serial.read());
        break;
      case 'k':  //Gebe k ein, um den Magnetometer zu kalibrieren
        loop_kalibrierungMAG();
        while (Serial.available() && Serial.read());
        break;
      default:
        Serial.print(F("Unbekannte Eingabe:"));
        Serial.println(Serial.read());
    }
  }
  //Wähle einen dieser Funktionen aus:
  //loop_output_MAG(); //Benötigt Magnetometer | Gibt x,y und z Werte von Magnetometer aus
  //loop_output_on_display(); //Benötigt MPU, Magnetomer, DHT und Display | Gibt alle Sensorwerte als Zahlenwerte auf Display aus
  //loop_kuenstlicher_horizont(); //Benötigt MPU und Display | Gibt einen Künstlichen Horizont auf Display aus
  loop_compass(); //Benötigt Magnetometer, MPU und Display | Gibt Kompass mit Wasserwaag in der Mitte aus
}
