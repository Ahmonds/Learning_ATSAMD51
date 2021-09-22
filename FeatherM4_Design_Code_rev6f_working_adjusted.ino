#include <Adafruit_LSM6DS33.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

Adafruit_NeoPixel strip(1, 8, NEO_GRB + NEO_KHZ400);
Adafruit_LSM6DS33 lsm6ds33_A, lsm6ds33_B;

#define hallPin 13                           // hall effect sensors pin
#define bLED 10                           // Battery LED
#define IR1 6                             // IR touch sensor 1 (left)
#define IR2 5                          // IR touch sensor 2 (right)
#define levelDirPin 11            // leveling actuator pin direction
#define speedDirPin 12        // Braking acutator pin direction
#define Speaker 9
#define battPin 16                           // Analog battery pin (A2) measrement (1.2M and 330K -> 3.17V)
#define speedPot 17                      // Analog Speed Pot (A3) measurment
#define ADC_RES  12                 // 12bits -> 3.17V/3.3V*4095 = 3934 max
#define battCuttOff 3440     	// 3772 full charge & 3449 considered dead
uint64_t pastMillis = 0, timeNow = 0, ran = 0x00000001;
float angleAY = 0, angleBX = 0;
byte mode = 2;
boolean BattState = true, levelDir = HIGH, speedDir = HIGH, BrakeState = true;

#define n 120               // accel readings
#define m 10              // Integral error
#define o 16            // wheel sensor

class SMA { // avgAY, avgAZ, avgBX, avgBZ
    byte Pos = 0;
    float readings[n], total = 0, avg = 0;
  public :
    SMA () { //Constrctor to clear/set reading array
      for (byte t = 0; t < n; ++t) {
      readings[t] = 0;
      }
    }
    float average (float reading) {
      total -= readings[Pos];                    //subtract old reading from total
      readings[Pos] = reading;               //place new reading into the array
      total += readings[Pos];                //add the new reading to the total
      (Pos < n-1 ? Pos++ : Pos = 0);      //Incrament Pos until the end is reached
      avg = total / n;                       //take the current average
      return avg;
  }
    float averaged () { return avg; }
    float sum () { return total; }
} avgAY, avgAZ, avgBZ, avgBX;

class SMA2 { // pastLevelingErr, pastSpeedErr, battCharge
    byte Pos = 0;
    float readings[m], total = 0, avg = 0;
  public :
    SMA2 () { //Constrctor to clear/set reading array
      for (byte t = 0; t < m; ++t) {
      readings[t] = 0;
      }
    }
    float average (float reading) {
      total -= readings[Pos];                    //subtract old reading from total
      readings[Pos] = reading;               //place new reading into the array
      total += readings[Pos];                //add the new reading to the total
      (Pos < m-1 ? Pos++ : Pos = 0);      //Incrament Pos until the end is reached
      avg = total / m;                       //take the current average
      return avg;
  }
    float averaged () { return avg; }
    float sum() { return total; }
} pastLevelingErr, pastSpeedErr, battCharge;

class SMA3 { // speedy_boi
    byte Pos = 0;
    float readings[o], total = 0, avg = 0;
  public :
    SMA3 () { //Constrctor to clear/set reading array
      for (byte t = 0; t < o; ++t) {
      readings[t] = 0;
      }
    }
    float average (float reading) {
      total -= readings[Pos];                    //subtract old reading from total
      readings[Pos] = reading;               //place new reading into the array
      total += readings[Pos];                //add the new reading to the total
      (Pos < o-1 ? Pos++ : Pos = 0);      //Incrament Pos until the end is reached
      avg = total / o;                       //take the current average
      return avg;
  }
    float averaged () { return avg; }
    float sum () { return total; }
} speedy_boi;
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

#define levelingPER 2399            // TCC3 PER reg val for frequency 
int levelingDUT = 0;                   // TCC3 CC[0] reg val for DUT
#define Kp 0.8
#define Ki 0.3

void ControlledLevel(float now) {

   levelingDUT = (Kp*now + Ki*pastLevelingErr.average(now))*levelingPER/8;  // calculate 

   levelDir = (levelingDUT > 0 ? HIGH : LOW);
   digitalWrite(levelDirPin, levelDir);
   
   levelingDUT = (levelingDUT <= 0 ? -levelingDUT : levelingDUT);                //Keep value within register value
   levelingDUT = (levelingDUT >= levelingPER ? levelingPER : levelingDUT);  // Keep value within register value
   //levelingDUT = (levelingDUT < 960 ? 960 : levelingDUT);
   
   TCC3->CCBUF[0].reg = levelingDUT;        // Write to buffered register
   //while (!TCC3->STATUS.bit.CCBUFV0); // unnecessary sync waiting
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

  TCC3->PER.reg = levelingPER;               		// Set-up the PER (period) register
  while (TCC3->SYNCBUSY.bit.PER); 
  
  TCC3->CC[0].reg = levelingDUT;                    // Set-up the CC (counter compare), channel 0 register
  while (TCC3->SYNCBUSY.bit.CC0);

  TCC3->CTRLA.bit.ENABLE = 1;                       // Enable timer TCC3
  while (TCC3->SYNCBUSY.bit.ENABLE);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

int speedSetVal = 60;                     // ≈60 milliseconds between magnets at 3mph
#define speedPER 2399             	// TCC2 PER reg val for frequency
int speedDUT = 0;                     // TCC2 CC[0] reg val for DUT
#define Kp 0.5
#define Ki 1.5
float speedErr = 0, speedy = 400;
bool speedyMode = true, speedMode = false;
uint64_t speedyTime = 0;

void ControlledSpeed(float now, float target) { // work in progress

   speedErr = target - now;
   speedDir = (speedErr > 0 ? HIGH : LOW);
   digitalWrite(speedDirPin, speedDir);
   speedErr = (speedErr < 0 ? -speedErr : speedErr);
   
   speedDUT = (Kp*speedErr + Ki*pastSpeedErr.average(speedErr))*speedPER/120;
   speedDUT = (speedDUT >= speedPER ? speedPER : speedDUT);

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

void AccelGyro_B_Begin() { // DO/AD0 pin pulled high changes address to 0x6B
  lsm6ds33_B.begin_I2C(0x6B); 

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

   //Serial.begin(115200);
   strip.begin();
   strip.setBrightness(32);
   strip.clear();
   strip.show();
   analogReadResolution(ADC_RES);
   TCC2BeginTimer();
   TCC3BeginTimer();
   AccelGyro_A_Begin();  // 6.66kHz is 150us -> 1.66kHz is 602.4us
   AccelGyro_B_Begin();
   speedyTime = millis();
   pastMillis = speedyTime;

}

void loop() {
   delayMicroseconds(120);       // Delay to read accelerameter readings at 6.66kHz

   lsm6ds33_B.getEvent(&accel_B, &gyro_B, &temp_B);
   avgBX.average(accel_B.acceleration.x);                      // Smooth data from accel B x-axis
   avgBZ.average(accel_B.acceleration.z);                     // Smooth data form accel B z-axis
   
   lsm6ds33_A.getEvent(&accel_A, &gyro_A, &temp_A);
   avgAY.average(accel_A.acceleration.y);                   // Smooth data from accel A y-axis
   avgAZ.average(accel_A.acceleration.z);                  // Smooth data from accel A z-axis
     
//============================================================================================================
   if ( millis() > timeNow) {    // Level every 1ms to avoid writing to registers before synchronization
      timeNow = millis();
      
      switch(mode) { // State Machine header
         case 0:        // Leveling Mode

            /*speedMode = digitalRead(hallPin);
            if ( !speedMode && speedyMode ) {
               speedy = (timeNow - speedyTime);
               speedyTime = timeNow;
               speedy = (speedy > 1000 ? 2000 : speedy);
               //ControlledSpeed(speedy, speedSetVal);
               speedyMode  = false;
            }
            else if (speedMode && !speedyMode) speedyMode = true;
            ControlledSpeed(speedy, speedSetVal);*/

            angleAY = atan2f( avgAY.averaged(), -avgAZ.averaged() )*180/PI;  // calculate shelf angle in degrees centered at 0°
            ControlledLevel(angleAY);  // Feed shelf angle to the PI controller

            if ( digitalRead(IR1) || digitalRead(IR2) ) mode = 1;     				// Go to E-Brake
            else if ( angleBX < 4 || angleBX > -3 ) mode = 3;                      // Go to Standby
      
         break;
//============================================================================================================
         case 1:      // Emergency Breaking 

            /*speedMode = digitalRead(hallPin);
            if (  !speedMode && speedyMode ) {
               speedy = (timeNow - speedyTime);
               speedyTime = timeNow;
               speedyMode  = false;
            }
            else if (speedMode && !speedyMode) speedyMode = true;
            ControlledSpeed(speedy, 0); */
			
			if (BrakeState) {
			   // Run emergency brake for static amount of time until braking PI controller works properly
				TCC3->CCBUF[0].reg = 0;
				TCC2->CCBUF[0].reg = speedPER;
				digitalWrite(speedDirPin, LOW);
				delay(1225);
            TCC2->CCBUF[0].reg = 0;
				BrakeState = false;
			}
			if (!digitalRead(IR1) && !digitalRead(IR2)) mode = 2;
         break;
//============================================================================================================
         case 2:      // Retracting (reset brake)
         
            // Retract emergency brake for static amount of time until braking PI controller works properly
            TCC3->CCBUF[0].reg = 0;
            TCC2->CCBUF[0].reg = speedPER;
            digitalWrite(speedDirPin, HIGH);
            delay(1600);
            TCC2->CCBUF[0].reg = 0;
            BrakeState = true;
            mode = 3;
            
         break;
//============================================================================================================
         default:    // Standby

            angleAY = atan2f( avgAY.averaged(), -avgAZ.averaged() )*180/PI;
            ControlledLevel(angleAY);
            
            TCC2->CCBUF[0].reg = 0;
			if ( angleBX > 6.5 || angleBX < -3.5 ) mode = 0;                      	// Go to Leveling
         break;
//============================================================================================================
      }
      if (millis() - pastMillis > 100) { // every 100ms update less important features
         pastMillis = millis();

         // Moniter battery and flash LED if low, else leave battery LED on when powered
         if ( battCharge.average(analogRead(battPin)) < battCuttOff && BattState) BattState = false;
         else BattState = true;
         digitalWrite(bLED, BattState);
		 
		   angleBX = atan2f( avgBX.averaged(), avgBZ.averaged() )*180/PI;    // Calculate cart angle in degrees centered on 0°

         speedSetVal = analogRead(speedPot)*0.024;    // Moniter handle potentiometer to set target speed for braking PI controller
         
         /*Serial.print(mode);
         Serial.print("    ");
         Serial.print(avgBZ.averaged());
         Serial.print("    ");
         Serial.print(avgBX.averaged());
         Serial.print("    ");
         Serial.print(angleBX);
         Serial.print("    ");
         Serial.print(angleAY);
         Serial.print("   ");
         Serial.print(digitalRead(IR1));
         Serial.print("   ");
         Serial.println(digitalRead(IR2));*/
         
         (ran < 0x00800000 ? ran*=2 : ran = 0x00000001);    // Cycle on board RGB LED to signal code has reached the end of loop
         strip.setPixelColor(0, ran);
         strip.show();
      }
   } // 1ms timed runs
}
