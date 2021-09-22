#include <Adafruit_LSM6DS33.h>
#include <Adafruit_NeoPixel.h>
/*
#define SCK 25
#define MOSI 24
#define MISO 23
#define CS1 13
*/
#define hallPin 12
#define IR1 5
#define IR2 6
#define modeSwitchPin 13
#define trig1 4
#define echo1 9

Adafruit_NeoPixel strip(1, 8, NEO_GRB + NEO_KHZ400);
Adafruit_LSM6DS33 lsm6ds33_A, lsm6ds33_B;

uint32_t ran = 0x00000001;
uint64_t pastMillis = 0;

float AX, AY, AZ, BX, BY, BZ, inch, ft;
int pulseWidth;

void setup() {
  strip.begin();
  strip.clear();
  strip.setBrightness(64);
  strip.show(); // Initialize all pixels to 'off'
  delay(10);
  pinMode(hallPin, INPUT);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(modeSwitchPin, INPUT);
  pinMode(trig1, OUTPUT);
  digitalWrite(trig1, LOW);
  pinMode(echo1, INPUT);
  delay(10);

  lsm6ds33_A.begin_I2C(0x6A);
  lsm6ds33_B.begin_I2C(0x6B);
  //lsm6ds33.begin_SPI(CS1, SCK, MISO, MOSI);

  lsm6ds33_A.setAccelRange(LSM6DS_ACCEL_RANGE_2_G); // 2G, 4G, 8G, 16G
  lsm6ds33_A.setAccelDataRate(LSM6DS_RATE_1_66K_HZ); // 12.5Hz, 26Hz, 52Hz, 104Hz, 208Hz, 416Hz,
                                                  // 833Hz, 1.66kHz, 3.33kHz, 6.66kHz
  lsm6ds33_B.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds33_B.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
  
  lsm6ds33_A.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS); // 125DPS, 250DPS, 1000DPS, 2000DPS, 4000DPS
  lsm6ds33_A.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);    // 12.5Hz, 26Hz, 52Hz, 104Hz, 208Hz, 416Hz,
                                                    // 833Hz, 1.66kHz, 3.33kHz, 6.66kHz
  lsm6ds33_B.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);
  lsm6ds33_B.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
  
  lsm6ds33_A.configInt1(false, false, false); // dissable INT1 for temp, gyro, accel
  lsm6ds33_A.configInt2(false, false, false); // dissable INT2 for temp, gyro, accel
  lsm6ds33_B.configInt1(false, false, false);
  lsm6ds33_B.configInt2(false, false, false);
  delay(10);
  Serial.begin(115200);
  delay(10);
}

void loop() {

  sensors_event_t accel_A, accel_B;
  sensors_event_t gyro_A, gyro_B;
  sensors_event_t temp_A, temp_B;
  lsm6ds33_A.getEvent(&accel_A, &gyro_A, &temp_A);
  lsm6ds33_B.getEvent(&accel_B, &gyro_B, &temp_B);
/*
  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");
*/
  AX = accel_A.acceleration.x;
  AY = accel_A.acceleration.y; 
  AZ = accel_A.acceleration.z;
  
  BX = accel_B.acceleration.x;
  BY = accel_B.acceleration.y;
  BZ = accel_B.acceleration.z;

  digitalWrite(trig1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig1, LOW);
  pulseWidth = pulseIn(echo1, HIGH, 23200) + 3; // 4,294,967,293 ? unsiged 4 bytes (-3)
  inch = pulseWidth / 148.0;
  ft = inch / 12;

switch (digitalRead(modeSwitchPin)) {
  case 0:
  if (millis() - pastMillis > 100) {
    pastMillis = millis();
    strip.setPixelColor(0, ran);
    strip.show();
    (ran < 0x00800000 ? ran*=2 : ran = 0x01);
  //  Display the results (acceleration is measured in m/s^2) 
    Serial.print("\n\tAccel_A X: ");
    Serial.print(AX);
    Serial.print(" \tY: ");
    Serial.print(AY);
    Serial.print(" \tZ: ");
    Serial.print(AZ);
    Serial.print(" m/s^2 \t\t");

    Serial.print("\tAccel_B X: ");
    Serial.print(BX);
    Serial.print(" \tY: ");
    Serial.print(BY);
    Serial.print(" \tZ: ");
    Serial.print(BZ);
    Serial.println(" m/s^2 ");
  
  // Display the results (rotation is measured in rad/s) 
    Serial.print("\tGyro_A  X: ");
    Serial.print(gyro_A.gyro.x);
    Serial.print(" \tY: ");
    Serial.print(gyro_A.gyro.y);
    Serial.print(" \tZ: ");
    Serial.print(gyro_A.gyro.z);
    Serial.print(" radians/s \t");
  
    Serial.print("\tGyro_B  X: ");
    Serial.print(gyro_B.gyro.x);
    Serial.print(" \tY: ");
    Serial.print(gyro_B.gyro.y);
    Serial.print(" \tZ: ");
    Serial.print(gyro_B.gyro.z);
    Serial.println(" radians/s\n");
  
    Serial.print("\tH-Field: ");
    Serial.print(digitalRead(hallPin));
    Serial.print("\t\tIR1: ");
    Serial.print(digitalRead(IR1));
    Serial.print("\t\tIR2: ");
    Serial.print(digitalRead(IR2));
    Serial.print("\t\t\t\tµSonic Dist (in): ");
    Serial.println(inch);
    Serial.println();
  }
  break;

  case 1:
  Serial.print(AX);
  Serial.print(','); Serial.print(AY);
  Serial.print(','); Serial.print(AZ);

  Serial.print(','); Serial.print(BX);
  Serial.print(','); Serial.print(BY);
  Serial.print(','); Serial.print(BZ);
  /*Serial.print(",");
  Serial.print(gyro_A.gyro.x);
  Serial.print(","); Serial.print(gyro_A.gyro.y);
  Serial.print(","); Serial.print(gyro_A.gyro.z);
  */
  Serial.print(','); Serial.print(ft);
  Serial.println();
  strip.clear();
  strip.show();
  break;
}
}
