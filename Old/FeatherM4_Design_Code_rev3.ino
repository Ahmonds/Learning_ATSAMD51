#include <Adafruit_LSM6DS33.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

#define n 25               // number of values to average
#define halPin 9        // hall effect sensor pin
uint64_t pastMillis = 0, ran = 0x00000001;
float angleAX, angleAY, errX, errY, angularAX;
byte tick = 0;

Adafruit_NeoPixel strip(1, 8, NEO_GRB + NEO_KHZ400);
Adafruit_LSM6DS33 lsm6ds33_A; // LSM6DS33 Gyro/Accel object create (add lsm6ds33_B again later)

#define levelSetVal 0.7 // degrees
#define levelMAxDeg 10
#define levelingPER 2399            // TCC3 PER reg val for frequency 
int levelingDUT = 1199;                          // TCC3 CC[0] reg val for DUT
#define levelingAggression 180             // 90  maps ≈ [ 0° - 45° ] to [ 50% - 100% ]
float levelingErr = 0;                             // 180 maps ≈ [ 0° - 80° ] to [ 50% - 100% ]

void ControlledLevel(float now) {

   levelingErr = (now - levelSetVal) / levelSetVal;
   levelingDUT = (levelingPER/2) + levelingErr*levelingPER/levelingAggression; // levelingPER/2 so aggression is twice as high
   (levelingDUT >= levelingPER ? levelingDUT = levelingPER : ++tick);
   (levelingDUT <= 0 ? levelingDUT = 0 : ++tick);

   TCC3->CCBUF[0].reg = levelingDUT;
   while (!TCC3->STATUS.bit.CCBUFV0);
}

#define speedSetVal 250
#define speedPER 2399           // TCC2 PER reg val for frequency
int speedDUT = 1199;                             // TCC2 CC[0] reg val for DUT
#define speedAggression 100
float speedErr = 0, speedPrevErr = 0, speedIntegral = 0, SPD = 0;

void ControlledSpeed(float now) {

   speedErr = now - speedSetVal;
   speedIntegral += speedErr;
   SPD = speedErr + (speedErr - speedPrevErr) + speedIntegral;

   speedDUT = SPD / ( speedAggression*( speedPER / 2 ) );
   
   TCC2->CCBUF[0].reg = speedDUT;
}

void TCC3BeginTimer() { // Leveling Actuator - 10kHz - 50% Dut -  PWM output on pin 1 (TX), WO[0]  -  TCC3->PERB.reg = val;
  // Set up the generic clock (GCLK5) to clock timer TCC3 
  GCLK->GENCTRL[5].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 48MHz clock source by divisor 3
                         GCLK_GENCTRL_IDC |            // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK5
                         GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL5); 

  GCLK->PCHCTRL[29].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK5; 

  // Enable the peripheral multiplexer on pin 1 (TX)
  PORT->Group[g_APinDescription[1].ulPort].PINCFG[g_APinDescription[1].ulPin].bit.PMUXEN = 1;
  // Set the PB16 peripheral multiplexer to peripheral (even port number) F(5): TCC3, Channel 0
  PORT->Group[g_APinDescription[1].ulPort].PMUX[g_APinDescription[1].ulPin >> 1].reg |= PORT_PMUX_PMUXE(5);

  TCC3->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV2 | TCC_CTRLA_PRESCSYNC_PRESC;

  TCC3->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;   // Set-up TCC3 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC3->SYNCBUSY.bit.WAVE);

/*TCC4/3/2 are only 16-bit timers, 
    therefore the PER/CC registers must be a value between 0 and 65535. 
    Only TCC timers TCC0 and TCC1 are 24-bits. */
    
/* (48,000,000/1)/(2*(2400)) = 10kHz
       log(2400)/log(2) = 11.2288 bit resolution
       1199/2399 = 50% Dut */

  TCC3->PER.reg = levelingPER;               // Set-up the PER (period) register
  while (TCC3->SYNCBUSY.bit.PER); 
  
  TCC3->CC[0].reg = levelingDUT;                   // Set-up the CC (counter compare), channel 0 register
  while (TCC3->SYNCBUSY.bit.CC0);

  TCC3->CTRLA.bit.ENABLE = 1;                       // Enable timer TCC3
  while (TCC3->SYNCBUSY.bit.ENABLE);
}

void TCC2BeginTimer() { // Breaking Actuator - 10kHz - 50% Dut -  PWM output on pin D4, WO[0]  -  TCC2->CCBUF[0].reg = val;
  // Set up the generic clock (GCLK4) to clock timer TCC2 
  GCLK->GENCTRL[4].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 48MHz clock source by divisor 1: 48MHz/1 = 48MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK5
                         GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL4); 

  GCLK->PCHCTRL[29].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK4; 

  // Enable the peripheral multiplexer on pin D4
  PORT->Group[g_APinDescription[4].ulPort].PINCFG[g_APinDescription[4].ulPin].bit.PMUXEN = 1;
  // Set the 4 (PA14) peripheral multiplexer to peripheral (even port number) F(5): TCC2, Channel 0
  PORT->Group[g_APinDescription[4].ulPort].PMUX[g_APinDescription[4].ulPin >> 1].reg |= PORT_PMUX_PMUXE(5);

  TCC2->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV2 | TCC_CTRLA_PRESCSYNC_PRESC;

  TCC2->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;   // Set-up TCC2 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC2->SYNCBUSY.bit.WAVE);

/*TCC2 is only a 16-bit timer, 
    therefore the PER/CC registers must be a value between 0 and 65535. 
*/
    
/*    Duty cycle, frequency. and resolution
       (48,000,000/1)/(2*(2400)) = 10kHz
       log(2400)/log(2) = 11.2288 bit resolution
       1199/2399 = 50% DUT
*/

  // you shouldn't need to change these, but you can before compiling
  TCC2->PER.reg = speedPER;                            // Set-up the PER (period) register
  while (TCC2->SYNCBUSY.bit.PER); 
  
  TCC2->CC[0].reg = speedDUT;                           // Set-up the CC (counter compare), channel 0 register
  while (TCC2->SYNCBUSY.bit.CC0);

  TCC2->CTRLA.bit.ENABLE = 1;                       // Enable timer TCC4
  while (TCC2->SYNCBUSY.bit.ENABLE);
}

class SMA {
   float readings[n], total = 0;
   byte Pos = 0;
  public :
    float average (float reading) {
    total -= readings[Pos];             //subtract old reading from total
    readings[Pos] = reading;          //place new reading into the array
    total += readings[Pos];             //add the new reading to the total
    (Pos < n-1 ? Pos++ : Pos = 0);     //Incrament Pos until the end is reached
    return total / n;                       //take the current average
  }
  float averaged () { return total / n; }
  SMA () {                                    //Constrctor to clear/set reading array
    for (byte t = 0; t < n; t++) {
    readings[t] = 0;
    }
  }
}avgX, avgY, avgZ, avgGX;

void AccelGyro_A_Begin() {
  lsm6ds33_A.begin_I2C(0x6A);

  lsm6ds33_A.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);   // 2G, 4G, 8G, 16G
  lsm6ds33_A.setAccelDataRate(LSM6DS_RATE_6_66K_HZ); // 12.5Hz, 26Hz, 52Hz, 104Hz, 208Hz, 416Hz,
                                                                                             // 833Hz, 1.66kHz, 3.33kHz, 6.66kHz
  
  lsm6ds33_A.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS); // 125DPS, 250DPS, 1000DPS, 2000DPS, 4000DPS
  lsm6ds33_A.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);     // 12.5Hz, 26Hz, 52Hz, 104Hz, 208Hz, 416Hz,
                                                                                                // 833Hz, 1.66kHz, 3.33kHz, 6.66kHz
  
  lsm6ds33_A.configInt1(false, false, false); // dissable INT1 for temp, gyro, accel
  lsm6ds33_A.configInt2(false, false, false); // dissable INT2 for temp, gyro, accel
}

/*void AccelGyro_B_Begin() {
  lsm6ds33_B.begin_I2C(0x6B); // DO/AD0 pin pulled high changes address to 0x6B

  lsm6ds33_B.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds33_B.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
 
  lsm6ds33_B.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);
  lsm6ds33_B.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);

  lsm6ds33_B.configInt1(false, false, false);
  lsm6ds33_B.configInt2(false, false, false);
}*/

sensors_event_t accel_A, gyro_A, temp_A;
//sensors_event_t accel_B, gyro_B, temp_B;

void setup() {
   __disable_irq();
   
   Serial.begin(115200);
   strip.begin();
   strip.setBrightness(64);
   strip.clear();
   strip.show(); 
   TCC3BeginTimer();
   TCC2BeginTimer();
   AccelGyro_A_Begin();
   pastMillis = millis();
   
    __enable_irq();
}

void loop() {
//digitalWrite(12, HIGH);
/*
   if (digitalRead(hallPin)) { // digitalRead(hallPin) returns high (true) or low (false)
      pastMillis = millis(); // will probably need this and an array of differences between each magnet time?

      // Wheel Size to velocity calculation and setting
      
      TCC2->CCBUF[0].reg = value;   // Setting this 'value' will change the duty cycle (check bottom of TCC2BeginTimer function)
      
      (ran < 0x00800000 ? ran*=2 : ran = 0x01); // this is just to show it works, will probably be removed later
      strip.setPixelColor(0, ran);
      strip.show();
   } */

   lsm6ds33_A.getEvent(&accel_A, &gyro_A, &temp_A);
   //lsm6ds33_B.getEvent(&accel_B, &gyro_B, &temp_B);

   avgX.average(accel_A.acceleration.x);
   avgZ.average(accel_A.acceleration.z);
   //angularAX = avgGX.average(gyro_A.gyro.x);
   //angleAY = atan(accel_A.acceleration.y/accel_A.acceleration.z)*180/PI;

   if (millis() - pastMillis > 100) {
      pastMillis = millis();

      angleAX = atan( avgX.averaged()/avgZ.averaged() )*180/PI;
      //(angleAX < 0 ? angleAX= -angleAX : ++tick);

      //angleAX = atan( avgX.averaged()/avgZ.averaged() )*180/PI;
      ControlledLevel(angleAX);
      
      Serial.print("Angle X: ");
      Serial.print( angleAX );
      Serial.print(",    ");
      Serial.print("levelPER: ");
      Serial.print( levelingDUT );
      Serial.print(",  ");
      Serial.printf( "%.4f\n", levelingErr);
      
      (ran < 0x00800000 ? ran*=2 : ran = 0x01);
      strip.setPixelColor(0, ran);
      strip.show();
   }
//digitalWrite(12, LOW);
}
