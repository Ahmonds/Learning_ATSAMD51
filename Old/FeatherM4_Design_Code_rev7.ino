#include <Adafruit_LSM6DS33.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

Adafruit_NeoPixel strip(1, 8, NEO_GRB + NEO_KHZ400);
Adafruit_LSM6DS33 lsm6ds33_A, lsm6ds33_B;

#define hallPin 12                      // hall effect sensors pin
#define bLED 10                        // Battery LED
#define IR1 6                            // IR touch sensor 1 (light)
#define IR2 5                            // IR touch sensor 2 (right)
#define levelDirPin 11                // leveling actuator pin direction
#define speedDirPin 13              // Braking acutator pin direction
#define Speaker 9
#define battPin 16                     // Analog battery pin (A2) measrement (1.2M and 330K -> 3.17V)
#define speedPot 17                  // Analog Speed Pot (A3) measurment
#define ADC_RES  12                // 12bits -> 3.17V/3.3V*4095 = 3934 max
#define battCuttOff 2500         // 3772 full charge & 3449 considered dead
float angleAY = 0, angleBX = 0;
byte tick = 0, mode = 0;
boolean BattState = true;

#define n 100               // accel readings
#define m 10                // Integral error
#define o 16               // wheel sensor

class SMA { // avgAY, avgAZ, avgBX, avgBZ
    byte Pos = 0;
    float readings[n], total = 0, avg = 0;
  public :
    SMA () {                                    //Constrctor to clear/set reading array
      for (byte t = 0; t < n; ++t) {
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
    float sum () { return total; }
} avgAY, avgAZ, avgBX, avgBZ;

class SMA2 { // pastLevelingErr, pastSpeedErr, battCharge
    byte Pos = 0;
    float readings[m], total = 0, avg = 0;
  public :
    SMA2 () {                                    //Constrctor to clear/set reading array
      for (byte t = 0; t < m; ++t) {
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
    float sum() { return total; }
} pastLevelingErr, pastSpeedErr, battCharge;

class SMA3 { // speedy_boi
    byte Pos = 0;
    float readings[o], total = 0, avg = 0;
  public :
    SMA3 () {                                    //Constrctor to clear/set reading array
      for (byte t = 0; t < o; ++t) {
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
    float sum () { return total; }
} speedy_boi;
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

#define levelSetVal 0.6                // degrees
#define levelingPER 2399            // TCC3 PER reg val for frequency 
int levelingDUT = 0;                     // TCC3 CC[0] reg val for DUT
#define Kp 0.9
#define Ki 0.9

void ControlledLevel(float now) {

   boolean levelDir = (now > 0 ? HIGH : LOW);
   digitalWrite(levelDirPin, levelDir);

   now = fabs(now);                   // Downhill is negative angle
   levelingDUT = (Kp*now + Ki*pastLevelingErr.averaged())*levelingPER/8;
   pastLevelingErr.average(now);
   
   //if (levelingDUT <= 0) levelingDUT = -levelingDUT;                   // Downhill is negative angle
   if (levelingDUT < 960) levelingDUT += 960;
   if (levelingDUT >= levelingPER ) levelingDUT = levelingPER;
   
   TCC3->CCBUF[0].reg = levelingDUT;
   //while (!TCC3->STATUS.bit.CCBUFV0);
}

void TCC3BeginTimer() { // Leveling Actuator - 20kHz - 0% Dut -  PWM output on pin 1 (TX), WO[0]  -  TCC3->PERB.reg = val;
  GCLK->GENCTRL[5].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 48MHz clock source by divisor 1
                         GCLK_GENCTRL_IDC |            // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK5
                         GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL5); 

  GCLK->PCHCTRL[29].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK5; 

  PORT->Group[g_APinDescription[1].ulPort].PINCFG[g_APinDescription[1].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[1].ulPort].PMUX[g_APinDescription[1].ulPin >> 1].reg |= PORT_PMUX_PMUXE(5);

  TCC3->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_PRESCSYNC_PRESC;

  TCC3->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;   // Set-up TCC3 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC3->SYNCBUSY.bit.WAVE);

/*TCC4/3/2 are only 16-bit timers, 
    therefore the PER/CC registers must be a value between 0 and 65535. 
    Only TCC timers TCC0 and TCC1 are 24-bits. */
    
/* (48,000,000/1)/(*(2400)) = 20kHz
       log(2400)/log(2) = 11.2288 bit resolution
       1199/2399 = 50% Dut */

  TCC3->PER.reg = levelingPER;               // Set-up the PER (period) register
  while (TCC3->SYNCBUSY.bit.PER); 
  
  TCC3->CC[0].reg = levelingDUT;                   // Set-up the CC (counter compare), channel 0 register
  while (TCC3->SYNCBUSY.bit.CC0);
/*
  TCC3->CC[1].reg = LEDDUT;                         // Set-up the CC (counter compare), channel 1 register
  while (TCC3->SYNCBUSY.bit.CC0);
*/
  TCC3->CTRLA.bit.ENABLE = 1;                       // Enable timer TCC3
  while (TCC3->SYNCBUSY.bit.ENABLE);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

int speedSetVal = 60;                    // 205 milliseconds between magnets at 3mph?
#define speedPER 2399                // TCC3 PER reg val for frequency 
int speedDUT = 0;                     // TCC3 CC[0] reg val for DUT
#define Kp 1.0
#define Ki 1.0

void ControlledSpeed(float now, float target) {

   float speedErr = now - target;
   speedDUT = (Kp*speedErr + Ki*pastSpeedErr.average(speedErr))*speedPER/60;

   speedDUT = (speedDUT <= 0 ? -speedDUT : speedDUT);
   speedDUT = (speedDUT >= speedPER ? speedPER : speedDUT);
   speedDUT = (speedDUT < 240 ? 0 : speedDUT);

   boolean speedDir = (speedDUT > 0 ? HIGH : LOW);
   digitalWrite(speedDirPin, speedDir);

   //TCC2->CCBUF[0].reg = speedDUT;
   //while (!TCC2->STATUS.bit.CCBUFV0);
}

void TCC2BeginTimer() { // Breaking Actuator - 20kHz - 0% Dut -  PWM output on pin D4, WO[0]  -  TCC2->CCBUF[0].reg = val;
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

  TCC2->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_PRESCSYNC_PRESC;

  TCC2->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;   // Set-up TCC2 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC2->SYNCBUSY.bit.WAVE);

/*TCC2 is only a 16-bit timer, 
    therefore the PER/CC registers must be a value between 0 and 65535. 
*/
    
/*    Duty cycle, frequency. and resolution
       (48,000,000/1)/(1*(2400)) = 20kHz
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
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

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
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {

   pinMode(hallPin, INPUT);
   pinMode(speedPot, INPUT);
   pinMode(battPin, INPUT);
   pinMode(IR1, INPUT);
   pinMode(IR2, INPUT);
   pinMode(levelDirPin, OUTPUT);
   pinMode(speedDirPin, OUTPUT);
   pinMode(bLED, OUTPUT);

   int speedy = 300;
   uint64_t pastMillis = millis(), ran = 0x00000001, speedyTime = millis(), speedyTime = millis();
   bool speedyMode = true, speedMode = false;

   //Serial.begin(115200);
   strip.begin();
   strip.setBrightness(64);
   strip.clear();
   strip.show();
   analogReadResolution(ADC_RES);
   TCC2BeginTimer();
   TCC3BeginTimer();
   AccelGyro_A_Begin(); // 6.66kHz is 150us -> 1.66kHz is 602.4us
   AccelGyro_B_Begin();
}

void loop() {
   delayMicroseconds(140);

   lsm6ds33_B.getEvent(&accel_B, &gyro_B, &temp_B);
   avgBX.average(accel_B.acceleration.x);
   avgBZ.average(accel_B.acceleration.z);

   angleBX = atan2f( avgBX.averaged(), avgBZ.averaged() )*180/PI;
   ControlledLevel(angleBX);

   lsm6ds33_A.getEvent(&accel_A, &gyro_A, &temp_A);
   avgAY.average(accel_A.acceleration.y);
   avgAZ.average(accel_A.acceleration.z);
     
//============================================================================================================
   if ( millis() > timeNow) {
      timeNow = millis();
      
      switch(0) {
         case 0:        // PI Down Hill

            speedMode = digitalRead(hallPin);
            if (  !speedMode && speedyMode ) {
               speedy = (timeNow - speedyTime);
               speedyTime = timeNow;
               speedy = (speedy > 300 ? 300 : speedy);
               //ControlledSpeed(speedy, speedSetVal);
               speedyMode  = false;
            }
            else if (speedMode && !speedyMode) speedyMode = true;
            ControlledSpeed(speedy, speedSetVal);

            angleAY = atan2f( avgAY.averaged(), -avgAZ.averaged() )*180/PI;
            //if ( -0.1 < angleAY < 0.1) TCC3->CCBUF[0].reg = 0;
            //else ControlledLevel(angleAY);
            ControlledLevel(angleAY);
      
            if ( digitalRead(IR1) || digitalRead(IR2) ) mode = 2;     // Go to E-Brake
            else if ( angleBX < 1 || angleBX > 0 ) mode = 4;              // Go to Standby
      
         break;
//============================================================================================================
         case 1:      // PI Up Hill
            angleAY = atan2f( avgAY.averaged(), -avgAZ.averaged() )*180/PI;
            ControlledLevel(angleAY);
	  
            if ( digitalRead(IR1) || digitalRead(IR2) ) mode = 2;     // Go to E-Brake
            else if ( angleBX < 1 || angleBX > 0 ) mode = 4;              // Go to Standby
         break;
//============================================================================================================
         case 2:      // Retracting (reset brake)

            
         break;
//============================================================================================================
         case 3:      // E-Breaking 

            speedMode = digitalRead(hallPin);
            if (  !speedMode && speedyMode ) {
               speedy = (timeNow - speedyTime);
               speedyTime = timeNow;
               ControlledSpeed(speedy, 0);
               speedyMode  = false;
            }
            else if (speedMode && !speedyMode) speedyMode = true;
      
            if ( !digitalRead(IR1) && !digitalRead(IR2) ) mode = 0;    // Go back to PI (Up or Down)
            else if ( angleBX < 1 || angleBX > 0 ) mode = 4;                // Go to Standby
         break;
//============================================================================================================
         default:    // Standby
      
            if ( angleBX > 1 ) mode = 0;                                          // Go to PI Down Hill
      	  else if ( angleBX < 0 ) mode = 1;                                     // Go to PI Up Hill
         break;
//============================================================================================================
      }
      if (millis() - pastMillis > 100) {
         pastMillis = millis();

         if ( battCharge.average(analogRead(battPin)) < battCuttOff && BattState) BattState = false;
         else BattState = true;
         digitalWrite(bLED, BattState);

         speedSetVal = analogRead(speedPot)*0.024;

         Serial.print(angleAY);
         Serial.print("   ");
         Serial.println(levelingDUT);

         (ran < 0x00800000 ? ran*=2 : ran = 0x00000001);
         strip.setPixelColor(0, ran);
         strip.show();
         }
   } // 1ms timed runs
}
