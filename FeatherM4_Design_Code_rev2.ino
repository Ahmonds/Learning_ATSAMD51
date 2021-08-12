/*
 * https://forum.arduino.cc/t/metro-m4-express-atsamd51-pwm-frequency-and-resolution/566491/24 
 * http://ww1.microchip.com/downloads/en/DeviceDoc/60001507E.pdf 
 *    PCHCTRL (pg 168)
 *    Timer interrupt flag handlers in section 10.2.2 (NVIC) Interrupt Line Mapping table 
 */

#define tccOutPin 25 // PA17 ( TCC0[5], TCC1[1], TC2[1] ) IOSET5 PINs      25 = SCK on Feather M4

uint64_t pastMillis = 0;
int pst =  1;

void TCC1BeginTimer() { // 24bit timer - 1MHz - 100% Dut default - pulse mode - output pin 25 (SCK)  (Feather M4)
  // Set up the generic clock (GCLK7) to clock timer TCC1
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL;
  while (GCLK->SYNCBUSY.bit.GENCTRL7);

  GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK7;    // Connect GCLK7 to GCLK_TCC1

  PORT->Group[g_APinDescription[25].ulPort].PINCFG[g_APinDescription[25].ulPin].bit.PMUXEN = 1;
  // Set the 25 (PA17) peripheral multiplexer to peripheral (odd port number) (F = 5) : TCC1 WO[1]
  PORT->Group[g_APinDescription[25].ulPort].PMUX[g_APinDescription[25].ulPin >> 1].reg |= PORT_PMUX_PMUXO(5); 
  
  TCC1->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV16 | TCC_CTRLA_PRESCSYNC_PRESC; // Set reset to trigger on prescaler clock

  TCC1->CTRLBSET.reg = TCC_CTRLBSET_ONESHOT;         // Enable one shot
  while (TCC1->SYNCBUSY.bit.CTRLB);

  TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;

   /* (48,000,000/1)/(16*(59+1)) = 50kHz
         log(50)/log(2) = 5.64 bit resolution
         19 / 59 = 33.3% Dut
   */
  TCC1->PER.reg = 59;                            // Set-up the PER (period) register
  while (TCC1->SYNCBUSY.bit.PER);
  
  TCC1->CC[1].reg = 19;                           // Set-up CC[1] register for both WO[1] and WO[5] 
  while (TCC1->SYNCBUSY.bit.CC1);

  TCC1->CTRLA.bit.ENABLE = 1;                  // Enable timer TCC1
  while (TCC1->SYNCBUSY.bit.ENABLE);
}

void TCC0BeginTimer() {

   NVIC_SetPriority(TCC0_0_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TCC0 to 0 (highest) 
   NVIC_EnableIRQ(TCC0_0_IRQn);         // Connect TCC0 to Nested Vector Interrupt Controller (NVIC)
   TCC0->INTENSET.reg |= TCC_INTENSET_OVF;  // Enable overflow interrupts
   // may also use TCC_INTENSET_MC1 or TCC_INTENSET_MC0 for match compare interrupts on channels 1 and 0
   
  // Set up the generic clock (GCLK7) to clock timer TCC0
  GCLK->GENCTRL[6].reg = GCLK_GENCTRL_DIV(3) |       // Divide the 48MHz clock source by divisor 1: 48MHz/1 = 48MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         GCLK_GENCTRL_SRC_DFLL;      // Select 100MHz DFLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL6);

  GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC0 peripheral channel
                          GCLK_PCHCTRL_GEN_GCLK6;    // Connect generic clock 7 to TCC0

  // Enable the peripheral multiplexer on pin RX
  PORT->Group[g_APinDescription[0].ulPort].PINCFG[g_APinDescription[9].ulPin].bit.PMUXEN = 1;
  
  // Set the RX (PB17) peripheral multiplexer to peripheral (even port number) G(7): TCC0, WO[5] (Channel 5)
  PORT->Group[g_APinDescription[0].ulPort].PMUX[g_APinDescription[9].ulPin >> 1].reg |= PORT_PMUX_PMUXO(7);

  TCC0->CTRLA.reg = TC_CTRLA_PRESCALER_DIV16 |        // Set prescaler to 8, 48MHz/3 = 16MHz
                    TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock                 

  TCC0->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;             // Set-up TCC0 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC0->SYNCBUSY.bit.WAVE);                    // Wait for synchronization

  TCC0->PER.reg = 3999;                            // 
  while (TCC0->SYNCBUSY.bit.PER);
  
  TCC0->CC[1].reg = 3999;                           // Set-up the CC (counter compare)
  while (TCC0->SYNCBUSY.bit.CC0);

  TCC0->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC0
  while (TCC0->SYNCBUSY.bit.ENABLE);
}

void TCC0_0_Handler() {
   TCC1->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;
   TCC0->INTFLAG.bit.OVF = 1;
}

void setup() {
  //Serial.begin(115200);
  //delay(10);
   __enable_irq();
   TCC1BeginTimer();
   TCC0BeginTimer();
   pastMillis = millis();
}

void loop() {
   
}
