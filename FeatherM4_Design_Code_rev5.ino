#include <Adafruit_LSM6DS33.h>
//#include <Adafruit_NeoPixel.h>
#include <math.h>

//Adafruit_NeoPixel strip(1, 8, NEO_GRB + NEO_KHZ400);
Adafruit_LSM6DS33 lsm6ds33_A, lsm6ds33_B;

#define hallPin 9       // hall effect sensors pin
#define IR1 10          // IR touch sensor 1
#define IR2 11          // IR touch sensor 2
#define battPin A2    // Analog battery pin measrement (1.2M and 4.3M -> 3.2727V)
#define ADC_RES  12           // 12bits = 4095 -> 3.27V/3.3V = 
#define battCuttOff 4060 //3.2727/3.30*(1 << ADC_RES)
uint64_t pastMillis = 0, ran = 0x00000001;
float angleAY = 0, angleBY = 0;
byte tick = 0, mode = 0;

#define n 50               // accel readings
#define m 5                // Integral error
#define o 16               // wheel sensor

class SMA {
    byte Pos = 0;
    float readings[n], total = 0, avg = 0;
  public :
    SMA () {                                    //Constrctor to clear/set reading array
      for (byte t = 0; t < n; t++) {
      readings[t] = 0;
      }
    }
    float average (float reading) {
    total -= readings[Pos];             //subtract old reading from total
    readings[Pos] = reading;          //place new reading into the array
    total += readings[Pos];             //add the new reading to the total
    (Pos < n-1 ? Pos++ : Pos = 0);     //Incrament Pos until the end is reached
    avg = total / n;                        //take the current average
    return avg;
  }
    float averaged () { return avg; }
} avgAY, avgAZ, avgBY, avgBZ;

class SMA2 {
    byte Pos = 0;
    float readings[m], total = 0, avg = 0;
  public :
    SMA2 () {                                    //Constrctor to clear/set reading array
      for (byte t = 0; t < n; t++) {
      readings[t] = 0;
      }
    }
    float average (float reading) {
    total -= readings[Pos];             //subtract old reading from total
    readings[Pos] = reading;          //place new reading into the array
    total += readings[Pos];             //add the new reading to the total
    (Pos < m-1 ? Pos++ : Pos = 0);     //Incrament Pos until the end is reached
    avg = total / m;                        //take the current average
    return avg;
  }
    float averaged () { return avg; }
} pastLevelingErr, pastSpeedErr, battCharge;

class SMA3 {
    byte Pos = 0;
    float readings[o], total = 0, avg = 0;
  public :
    SMA3 () {                                    //Constrctor to clear/set reading array
      for (byte t = 0; t < n; t++) {
      readings[t] = 0;
      }
    }
    float average (float reading) {
    total -= readings[Pos];             //subtract old reading from total
    readings[Pos] = reading;          //place new reading into the array
    total += readings[Pos];             //add the new reading to the total
    (Pos < o-1 ? Pos++ : Pos = 0);     //Incrament Pos until the end is reached
    avg = total / o;                        //take the current average
    return avg;
  }
    float averaged () { return avg; }
} speedy_boi;

#define levelSetVal 0.6                // degrees
#define levelingPER 2399            // TCC3 PER reg val for frequency 
int levelingDUT = 1199;                 // TCC3 CC[0] reg val for DUT
#define levelingAggression 10             // 90  maps ≈ [ 0° - 45° ] to [ 50% - 100% ], 180 maps ≈ [ 0° - 80° ] to [ 50% - 100% ]
#define Kp 0.6
#define Ki 0.7
float levelingErr = 0;

void ControlledLevel(float now) {

   levelingErr = now - levelSetVal;
   levelingDUT = (levelingPER/2) + (Kp*levelingErr + Ki*pastLevelingErr.average(levelingErr))*levelingPER/levelingAggression; 
                                                                                                                        // levelingPER/2 so aggression is twice as high
   (levelingDUT >= levelingPER ? levelingDUT = levelingPER : ++tick);
   (levelingDUT <= 0 ? levelingDUT = 0 : ++tick);

   TCC3->CCBUF[0].reg = levelingDUT;
   //while (!TCC3->STATUS.bit.CCBUFV0);
}

void TCC3BeginTimer() { // Leveling Actuator - 10kHz - 50% Dut -  PWM output on pin 1 (TX), WO[0]  -  TCC3->PERB.reg = val;
  GCLK->GENCTRL[5].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 48MHz clock source by divisor 3
                         GCLK_GENCTRL_IDC |            // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK5
                         GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL5); 

  GCLK->PCHCTRL[29].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK5; 

  PORT->Group[g_APinDescription[1].ulPort].PINCFG[g_APinDescription[1].ulPin].bit.PMUXEN = 1;
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

#define speedSetVal 205              // milliseconds between magnets at 3mph
#define speedPER 2399                // TCC3 PER reg val for frequency 
int speedDUT = 1199;                     // TCC3 CC[0] reg val for DUT
#define speedAggression 20             // 90  maps ≈ [ 0° - 45° ] to [ 50% - 100% ], 180 maps ≈ [ 0° - 80° ] to [ 50% - 100% ]
#define Kp 0.5
#define Ki 0.5
float speedErr = 0, speedy = 0;
bool speedyMode = true, speedMode = false;
uint64_t speedyTime = 0;

void ControlledSpeed(float now) {

   speedErr = now - speedSetVal;
   speedDUT = (speedPER/2) + (Kp*speedErr + Ki*pastSpeedErr.average(speedErr))*speedPER/speedAggression; 
                                                                                                                        // levelingPER/2 so aggression is twice as high
   (speedDUT >= speedPER ? speedDUT = speedPER : ++tick);
   (speedDUT <= 0 ? speedDUT = 0 : ++tick);

   TCC2->CCBUF[0].reg = speedDUT;
   //while (!TCC2->STATUS.bit.CCBUFV0);
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

void AccelGyro_B_Begin() {
  lsm6ds33_B.begin_I2C(0x6B); // DO/AD0 pin pulled high changes address to 0x6B

  lsm6ds33_B.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds33_B.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
 
  lsm6ds33_B.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);
  lsm6ds33_B.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);

  lsm6ds33_B.configInt1(false, false, false);
  lsm6ds33_B.configInt2(false, false, false);
}

sensors_event_t accel_A, gyro_A, temp_A, accel_B, gyro_B, temp_B;

void setup() {
   
   //Serial.begin(9600);
   /*strip.begin();
   strip.setBrightness(64);
   strip.clear();
   strip.show(); */
   analogReadResolution(ADC_RES);
   delay(10);
   TCC2BeginTimer();
   delay(10);
   TCC3BeginTimer();
   delay(10);
   AccelGyro_A_Begin();
   delay(10);
   AccelGyro_B_Begin();
   delay(10);
   pastMillis = millis();
   speedyTime = millis();
}

void loop() {

   lsm6ds33_B.getEvent(&accel_B, &gyro_B, &temp_B);
   avgBY.average(accel_B.acceleration.y);
   avgBZ.average(accel_B.acceleration.z);

   lsm6ds33_A.getEvent(&accel_A, &gyro_A, &temp_A);
   avgAY.average(accel_A.acceleration.y);
   avgAZ.average(accel_A.acceleration.z);

   angleBY = atan( avgBY.averaged()/avgBZ.averaged() )*180/PI;
   ControlledLevel(angleBY);

   if ( battCharge.average(analogRead(battPin)) > battCuttOff) digitalWrite(13, HIGH);
   else digitalWrite(13, LOW);

//============================================================================================================
/*switch(mode) {
   case 0:        // PI Controllers*/
      speedMode = digitalRead(hallPin);
      if (  speedMode && speedyMode ) {
         speedy = millis() - speedyTime;
         speedyTime = millis();
         ControlledSpeed(speedy);
         speedyMode  = false;
      }
      else if (speedMode && !speedyMode) speedyMode = true;
   
      angleAY = atan( avgAY.averaged()/avgAZ.averaged() )*180/PI;
      ControlledLevel(angleAY);

      if ( digitalRead(IR1) || digitalRead(IR2) ) mode = 2;    // Go to E-Brake
      else if ( angleBY < 1 || angleBY > -1 ) mode = 1;             // Go to Standby
      delay(10);
/*
      if (millis() - pastMillis > 100) {
         pastMillis = millis();
         (ran < 0x00800000 ? ran << 1 : ran = 0x01);
         strip.setPixelColor(0, ran);
         strip.show();
      }*//*
   break;
//============================================================================================================
   case 1:      // Standby (level ground)
      
      if ( angleBY < -1 ) mode = 0;                                         // Go to PI Controllers
   break;
//============================================================================================================
   case 2:      // E-Breaking 

      if ( angleBY < -1 ) mode = 0;                                         // Go to PI Controllers
      else if ( angleBY < 1 || angleBY > -1 ) mode = 1;             // Go to Standby
   break;
//============================================================================================================
   default:    // Retracting (Reset)

      if ( angleBY < -1 ) mode = 0;                                         // Go to PI Controllers
      else if ( angleBY < 1 || angleBY > -1 ) mode = 1;             // Go to Standby
   break;
//============================================================================================================
}*/
}
