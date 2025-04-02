#include <EEPROM.h>

void writeWordEEPROM(uint16_t adress, int16_t value);
int16_t readWordEEPROM(int16_t adress);
void saveActiveOffsets(int16_t xa, int16_t ya, int16_t za, int16_t xg, int16_t yg, int16_t zg);
void printEEPROMOffsets();


void writeWordEEPROM(uint16_t adress, int16_t value) {
  EEPROM.write(adress, value >> 8);
  EEPROM.write(adress + 1, value);
}

int16_t readWordEEPROM(int16_t adress) {
  return (EEPROM.read(adress) << 8 | EEPROM.read(adress + 1));
}

void saveActiveOffsets(int16_t xa, int16_t ya, int16_t za, int16_t xg, int16_t yg, int16_t zg) {
  writeWordEEPROM(2, ya);
  writeWordEEPROM(4, za);
  writeWordEEPROM(6, xg);
  writeWordEEPROM(8, yg);
  writeWordEEPROM(10, zg);
}

void printEEPROMOffsets() {
  Serial.print(readWordEEPROM(0));
  Serial.print("\t");
  Serial.print(readWordEEPROM(2));
  Serial.print("\t");
  Serial.print(readWordEEPROM(4));
  Serial.print("\t");
  Serial.print(readWordEEPROM(6));
  Serial.print("\t");
  Serial.print(readWordEEPROM(8));
  Serial.print("\t");
  Serial.println(readWordEEPROM(10));
}

/*_________________________________________________________________________________________________________________*/

// Inline Funktionen werden direkt in den Code kopiert und sind im Betrieb einige Takte schneller
inline void loop_output_MPU() __attribute__((always_inline));
inline void loop_output_on_display() __attribute__((always_inline));
inline void loop_compass() __attribute__((always_inline));
inline void loop_kuenstlicher_horizont() __attribute__((always_inline));


#ifdef MPU

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float vx, vy;           // Geschwindigkeiten

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
#endif

#ifdef OLEDDISPLAY
#include "U8g2lib.h"
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#endif

#ifdef DHTSENSOR
#include "DHT.h"
#define DHTPIN 3
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
#endif

#ifdef MAGNETOMETER
#include "QMC5883LCompass.h"
QMC5883LCompass compass;
int x, y, z;
float xk, yk;

//Für die Kalibrierung:
int calibrationData[3][2];
bool done = false;
#endif

// ================================================================
// ===             INITIALISIERUNGEN IM SETUP-BLOCK             ===
// ================================================================

void setup_mpu() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.begin(115200);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply offsets from EEPROM
  //  mpu.setXAccelOffset(readWordEEPROM(0));
  //  mpu.setYAccelOffset(readWordEEPROM(2));
  //  mpu.setZAccelOffset(readWordEEPROM(4));
  //  mpu.setXGyroOffset(readWordEEPROM(6));
  //  mpu.setYGyroOffset(readWordEEPROM(8));
  //  mpu.setZGyroOffset(readWordEEPROM(10));

  float t = mpu.getTemperature() / 340 + 35;  //Nutze enweder Temperatur vom MPU Sensor
  //  float t = dht.readTemperature();        //oder Temperatur vom DHT Sensor
  mpu.setXAccelOffset(-0.0041 * t - 918);
  mpu.setYAccelOffset(0.0072 * t * t - 0.74 * t + 534);
  mpu.setZAccelOffset(2 * t - 1110);
  mpu.setXGyroOffset(0.036 * t * t - 5.5 * t + 175);
  mpu.setYGyroOffset(-65);
  mpu.setZGyroOffset(0.0031 * t * t - 0.69602 * t + 1.1712);
  Serial.print(t);
  Serial.print(",");
  Serial.print("\t");
  mpu.PrintActiveOffsets();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    Serial.println();
    mpu.PrintActiveOffsets();
    printEEPROMOffsets();

    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void setup_display() {
  u8g2.begin();
}

void setup_dht() {
  dht.begin();
}

void setup_compass() {
  compass.init();
  compass.setCalibration(readWordEEPROM(12), readWordEEPROM(14), readWordEEPROM(16), readWordEEPROM(18), readWordEEPROM(20), readWordEEPROM(22));
  //compass.setSmoothing(5, false);
}

// ================================================================
// ===               FUNKTIONEN FÜR DEN LOOP-BLOCK              ===
// ================================================================


void loop_kalibrierungMPU() {
  Serial.println(F("Kalibriere MPU neu"));
  mpu.CalibrateAccel(6);  //Ermittelt Offsetwerte und speichert diese in
  mpu.CalibrateGyro(6);   //das Offsetregister der MPU
  //mpu.PrintActiveOffsets();  //Gibt das Offsetregister aus
  writeWordEEPROM(0, mpu.getXAccelOffset());
  writeWordEEPROM(2, mpu.getYAccelOffset());
  writeWordEEPROM(4, mpu.getZAccelOffset());
  writeWordEEPROM(6, mpu.getXGyroOffset());
  writeWordEEPROM(8, mpu.getYGyroOffset());
  writeWordEEPROM(10, mpu.getZGyroOffset());
}

void loop_kalibrierungMAG() {
  Serial.println(F("Kalibriere Kompass neu"));
  Serial.println("Kalibrierung beginnt in 3 Sekunden. Bewegegen Sie den Kompass solange, bis keine neuen Extremwerte mehr ermittel werden.");
  delay(3000);
  compass.setCalibration(-1, 1, -1, 1, -1, 1);  //Verhindert vorherige Kalibrationen
  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
  calibrationData[0][0] = x;
  calibrationData[0][1] = x;
  calibrationData[1][0] = y;
  calibrationData[1][1] = y;
  calibrationData[2][0] = z;
  calibrationData[2][1] = z;
  Serial.print(x);
  Serial.print("\t");
  Serial.print(calibrationData[0][0]);
  Serial.print("\t");
  Serial.print(calibrationData[0][1]);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(y);
  Serial.print("\t");
  Serial.print(calibrationData[1][0]);
  Serial.print("\t");
  Serial.print(calibrationData[1][1]);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(z);
  Serial.print("\t");
  Serial.print(calibrationData[2][0]);
  Serial.print("\t");
  Serial.println(calibrationData[2][1]);
  while (!done) {
    compass.read();
    x = compass.getX();
    y = compass.getY();
    z = compass.getZ();

    if (x < calibrationData[0][0]) {
      calibrationData[0][0] = x;
    }
    if (x > calibrationData[0][1]) {
      calibrationData[0][1] = x;
    }

    if (y < calibrationData[1][0]) {
      calibrationData[1][0] = y;
    }
    if (y > calibrationData[1][1]) {
      calibrationData[1][1] = y;
    }

    if (z < calibrationData[2][0]) {
      calibrationData[2][0] = z;
    }
    if (z > calibrationData[2][1]) {
      calibrationData[2][1] = z;
    }

    if (!done) {
      Serial.print(x);
      Serial.print("\t");
      Serial.print(calibrationData[0][0]);
      Serial.print("\t");
      Serial.print(calibrationData[0][1]);
      Serial.print("\t");
      Serial.print("\t");
      Serial.print(y);
      Serial.print("\t");
      Serial.print(calibrationData[1][0]);
      Serial.print("\t");
      Serial.print(calibrationData[1][1]);
      Serial.print("\t");
      Serial.print("\t");
      Serial.print(z);
      Serial.print("\t");
      Serial.print(calibrationData[2][0]);
      Serial.print("\t");
      Serial.println(calibrationData[2][1]);
    }

    if (Serial.available() > 0) {
      compass.setCalibration(calibrationData[0][0], calibrationData[0][1], calibrationData[1][0], calibrationData[1][1], calibrationData[2][0], calibrationData[2][1]);
      writeWordEEPROM(12, calibrationData[0][0]);
      writeWordEEPROM(14, calibrationData[0][1]);
      writeWordEEPROM(16, calibrationData[1][0]);
      writeWordEEPROM(18, calibrationData[1][1]);
      writeWordEEPROM(20, calibrationData[2][0]);
      writeWordEEPROM(22, calibrationData[2][1]);
      done = true;
      while (Serial.available() && Serial.read());
    }
  }
  done = false;
}

void loop_output_MAG() {
  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  Serial.print(x);
  Serial.print("\t");
  Serial.print(y);
  Serial.print("\t");
  Serial.println(z);
}

void loop_output_on_display() {
  if (!dmpReady) return;
  /* read a packet from FIFO*/
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { /* Get the Latest packet*/

    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGyro(&gy, fifoBuffer);
    compass.read();

    Serial.println("Displayprogramm läuft");
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_haxrcorp4089_tr);
    u8g2.drawStr(0, 10, "Acc X:");
    u8g2.drawStr(0, 20, "Acc Y:");
    u8g2.drawStr(0, 30, "Acc Z:");
    u8g2.drawStr(0, 40, "Gyro X:");
    u8g2.drawStr(0, 50, "Gyro Y:");
    u8g2.drawStr(0, 60, "Gyro Z:");


    u8g2.setCursor(35, 10);
    u8g2.print((float)aa.x / 16384);
    u8g2.setCursor(35, 20);
    u8g2.print((float)aa.y / 16384);
    u8g2.setCursor(35, 30);
    u8g2.print((float)aa.z / 16384);

    u8g2.setCursor(35, 40);
    u8g2.print((float)gy.x / 131);
    u8g2.setCursor(35, 50);
    u8g2.print((float)gy.y / 131);
    u8g2.setCursor(35, 60);
    u8g2.print((float)gy.z / 131);

    u8g2.drawStr(64, 10, "T MPU:");
    u8g2.drawStr(64, 20, "T DHT:");
    u8g2.drawStr(64, 30, "Hum:");
    u8g2.drawStr(64, 40, "Azi:");

    u8g2.setCursor(96, 10);
    u8g2.print((float)mpu.getTemperature() / 340 + 35);
    u8g2.setCursor(96, 20);
    u8g2.print(dht.readTemperature());
    u8g2.setCursor(96, 30);
    u8g2.print(dht.readHumidity());
    u8g2.setCursor(90, 40);
    u8g2.print(compass.getAzimuth());

    u8g2.sendBuffer();
  }
}



void loop_kuenstlicher_horizont() {
  if (!dmpReady) return;
  /* read a packet from FIFO*/
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { /* Get the Latest packet*/

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);

    u8g2.clearBuffer();

    //Neutrale Lage Markierung
    u8g2.drawLine(36, 32, 55, 32);
    u8g2.drawLine(74, 32, 93, 32);

    //Künstlicher Horizont
    u8g2.drawLine(-(32 * cos(-ypr[2])) + 64, (32 * sin(ypr[2])) + 32 + (32 * sin(ypr[1])), (32 * cos(-ypr[2])) + 64, (32 * sin(-ypr[2])) + 32 + (32 * sin(ypr[1])));

    u8g2.sendBuffer();
  }
}

void loop_compass() {
  if (!dmpReady) return;
  /* read a packet from FIFO*/
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { /* Get the Latest packet*/

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    compass.read();
    x = compass.getX();
    y = compass.getY();
    z = compass.getZ();

    xk = cos(-ypr[1]) * x + sin(ypr[2]) * sin(-ypr[1]) * y + cos(ypr[2]) * sin(-ypr[1]) * z;
    yk = cos(ypr[2]) * y - sin(ypr[2]) * z;

    float a = atan2(y, x); //Nicht Korrigierter Azimut zum Vergleich
    float c = atan2(yk, xk);

    Serial.print(a * 180 / PI);
    Serial.print("\t");
    //        Serial.print(b * 180 / PI);
    //        Serial.print("\t");
    Serial.println(c * 180 / PI);


    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_haxrcorp4089_tr);

    //Nordmarkierung
    u8g2.drawLine(64, 1, 64, 10);
    u8g2.drawStr(62, 20, "N");

    //Ostmarkierung
    u8g2.drawLine(87, 32, 94, 32);

    //Südmarkierung
    u8g2.drawLine(64, 54, 64, 64);

    //Westmarkierung
    u8g2.drawLine(34, 32, 41, 32);

    //Azimutstrich mit Neigungsausgleich
    u8g2.drawLine(23 * sin(-c) + 64, -23 * cos(-c) + 32, 30 * sin(-c) + 64, -30 * cos(-c) + 32);
    //Azimutstrich ohne Neigungsausgleich (Ist etwas länger)
    //u8g2.drawLine(20 * sin(-a) + 64, -20 * cos(-a) + 32, 30 * sin(-a) + 64, -30 * cos(-a) + 32);

    //Ring
    u8g2.drawCircle(64, 32, 31);
    u8g2.drawCircle(64, 32, 22);

    //Kreuz in Mitte
    u8g2.drawLine(61, 32, 67, 32);
    u8g2.drawLine(64, 29, 64, 35);

    //"Wasserwaage"
    u8g2.drawCircle(19 * sin(-ypr[2]) + 64, 19 * sin(-ypr[1]) + 32, 3);

    u8g2.sendBuffer();
  }
}
