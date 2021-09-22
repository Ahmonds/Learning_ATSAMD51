/*
 * https://forum.arduino.cc/t/metro-m4-express-atsamd51-pwm-frequency-and-resolution/566491/24 
 * http://ww1.microchip.com/downloads/en/DeviceDoc/60001507E.pdf 
 *    PCHCTRL (pg 168)
 *    Timer interrupt flag handlers in section 10.2.2 (NVIC) Interrupt Line Mapping table 
 */
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip(1, 8, NEO_GRB + NEO_KHZ400);

uint64_t pastMillis = 0, ran = 0x00000001;
int pst =  1;
byte flag = 0;

void TCC3BeginTimer() { // 1kHz - 50% Dut -  PWM output on pin 1 (TX), WO[0]
  // Set up the generic clock (GCLK5) to clock timer TCC3 
  GCLK->GENCTRL[5].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 48MHz clock source by divisor 3: 48MHz/3 = 16MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK5
                         GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL5); 

  GCLK->PCHCTRL[29].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK5; 

  // Enable the peripheral multiplexer on pin ##
  PORT->Group[g_APinDescription[1].ulPort].PINCFG[g_APinDescription[1].ulPin].bit.PMUXEN = 1;
  // Set the 1 (PB16) peripheral multiplexer to peripheral (even port number) F(5): TCC3, Channel 0
  PORT->Group[g_APinDescription[1].ulPort].PMUX[g_APinDescription[1].ulPin >> 1].reg |= PORT_PMUX_PMUXE(5);

  TCC3->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV8 | TCC_CTRLA_PRESCSYNC_PRESC;

  TCC3->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;   // Set-up TCC3 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC3->SYNCBUSY.bit.WAVE);

/*TCC4/3/2 are only 16-bit timers, 
    therefore the PER/CC registers must be a value between 0 and 65535. 
    Only TCC timers TCC0 and TCC1 are 24-bits. */
    
/* (48,000,000/1)/(8*(6000)) = 1kHz
       log(50)/log(2) = 12.55 bit resolution
       2999/5999 = 50% Dut */

  TCC3->PER.reg = 5999;                            // Set-up the PER (period) register
  while (TCC3->SYNCBUSY.bit.PER); 
  
  TCC3->CC[0].reg = 2999;                           // Set-up the CC (counter compare), channel 0 register
  while (TCC3->SYNCBUSY.bit.CC0);

  TCC3->CTRLA.bit.ENABLE = 1;                       // Enable timer TCC4
  while (TCC3->SYNCBUSY.bit.ENABLE);
}

void TCC2BeginTimer() { // 1kHz - 50% Dut -  PWM output on pin D4, WO[0]
  // Set up the generic clock (GCLK4) to clock timer TCC2 
  GCLK->GENCTRL[4].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 48MHz clock source by divisor 3: 48MHz/3 = 16MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK5
                         GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL4); 

  GCLK->PCHCTRL[29].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK4; 

  // Enable the peripheral multiplexer on pin ##
  PORT->Group[g_APinDescription[4].ulPort].PINCFG[g_APinDescription[4].ulPin].bit.PMUXEN = 1;
  // Set the 4 (PA14) peripheral multiplexer to peripheral (even port number) F(5): TCC2, Channel 0
  PORT->Group[g_APinDescription[4].ulPort].PMUX[g_APinDescription[4].ulPin >> 1].reg |= PORT_PMUX_PMUXE(5);

  TCC2->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV8 | TCC_CTRLA_PRESCSYNC_PRESC;

  TCC2->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;   // Set-up TCC2 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC2->SYNCBUSY.bit.WAVE);

/*TCC2 is only a 16-bit timer, 
    therefore the PER/CC registers must be a value between 0 and 65535. 
    Only TCC timers TCC0 and TCC1 are 24-bits. */
    
/* (48,000,000/1)/(8*(6000)) = 1kHz
       log(50)/log(2) = 12.55 bit resolution
       2999/5999 = 50% Dut */

  TCC2->PER.reg = 5999;                            // Set-up the PER (period) register
  while (TCC2->SYNCBUSY.bit.PER); 
  
  TCC2->CC[0].reg = 2999;                           // Set-up the CC (counter compare), channel 0 register
  while (TCC2->SYNCBUSY.bit.CC0);

  TCC2->CTRLA.bit.ENABLE = 1;                       // Enable timer TCC4
  while (TCC2->SYNCBUSY.bit.ENABLE);
}

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

void TCC0BeginTimer() { // Interrupt Timer on pin 0 (RX)

   NVIC_SetPriority(TCC0_0_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TCC0 to 0 (highest) 
   NVIC_EnableIRQ(TCC0_0_IRQn);         // Connect TCC0 to Nested Vector Interrupt Controller (NVIC)
   TCC0->INTENSET.reg |= TCC_INTENSET_OVF;  // Enable overflow interrupts
   // may also use TCC_INTENSET_MC1 or TCC_INTENSET_MC0 for match compare interrupts on channels 1 and 0
   
  // Set up the generic clock (GCLK6) to clock timer TCC0
  GCLK->GENCTRL[6].reg = GCLK_GENCTRL_DIV(3) | 
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL6);

  GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC0 peripheral channel
                          GCLK_PCHCTRL_GEN_GCLK6;    // Connect generic clock 6 to TCC0

  // Enable the peripheral multiplexer on pin RX
  PORT->Group[g_APinDescription[0].ulPort].PINCFG[g_APinDescription[9].ulPin].bit.PMUXEN = 1;
  
  // Set the RX (PB17) peripheral multiplexer to peripheral (even port number) G(7): TCC0, WO[5] (Channel 5)
  PORT->Group[g_APinDescription[0].ulPort].PMUX[g_APinDescription[9].ulPin >> 1].reg |= PORT_PMUX_PMUXO(7);

  TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV16 | TCC_CTRLA_PRESCSYNC_PRESC;  // Set the reset/reload to trigger on prescaler clock                 

  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;             // Set-up TCC0 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC0->SYNCBUSY.bit.WAVE);                    // Wait for synchronization

   // 48,000,000/3/(16*4000) = 250 Hz ≈ 4ms

  TCC0->PER.reg = 3999;                            // Setting how long it takes to overflow will determine interrupt frequency
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

#define EIC_NUM                   5
#define BUTTON_PORT          0
#define BUTTON_PIN            21         /* PA21 ->  D11? */
#define BUTTON_BITMASK   (1 << BUTTON_PIN)
#define BUTTON_EIC_MASK  (1 << 5)

void ExtInterrupt5Setup() { // external interrupt 5 (EIC5) pin D11
   
    GCLK->PCHCTRL[4].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN(0);     // Connect GCLK0 to IEC 
    while ((GCLK->PCHCTRL[4].reg & GCLK_PCHCTRL_CHEN) == 0);                        // Wait for Sync 

    PORT->Group[BUTTON_PORT].PINCFG[BUTTON_PIN].reg = PORT_PINCFG_PMUXEN | PORT_PINCFG_PULLEN | PORT_PINCFG_INEN;  // alternate function for PA21
    PORT->Group[BUTTON_PORT].PMUX[BUTTON_PIN].reg |= PORT_PMUX_PMUXO(0);             // PA21 = EIC/EXTINT[5] 
    //PORT->Group[BUTTON_PORT].PMUX[BUTTON_PIN].bit.PMUXO = PORT_PMUX_PMUXO(0);
    PORT->Group[BUTTON_PORT].OUTSET.reg = BUTTON_BITMASK;                                             // Make user button a pull-up

    EIC->CTRLA.reg = EIC_CTRLA_SWRST;                                   // reset the External Interrupt Controller
    while(EIC->SYNCBUSY.reg & EIC_SYNCBUSY_SWRST);         // wait for reset to complete
    EIC->CONFIG[0].reg |= EIC_CONFIG_SENSE5_FALL;              // trigger on falling edge of IEC5
    EIC->INTENSET.reg = BUTTON_EIC_MASK;
    EIC->INTFLAG.reg  = BUTTON_EIC_MASK;
    EIC->CTRLA.reg = EIC_CTRLA_ENABLE;                                   // enable EIC 
    while(EIC->SYNCBUSY.reg & EIC_SYNCBUSY_ENABLE);         // wait for enable to complete
    
    NVIC_EnableIRQ(EIC_5_IRQn);
}

void EIC_5_Handler() {
   flag++;
   EIC->INTENCLR.reg = 1 << EIC_NUM;  // Clear interrupt on external interrupt EIC_NUM (5)
    while (EIC->SYNCBUSY.bit.ENABLE);
    //EIC->INTFLAG.reg = EIC->INTFLAG.reg;               /* clear all EIC interrupt flags */
}

void setup() {
    //SystemInit();          // Initialize the SAM system -> crashes microcontroller
   __disable_irq();
   
   Serial.begin(115200);
   strip.begin();
   strip.setBrightness(64);
   strip.clear();
   strip.show(); 
   //TCC3BeginTimer();
   //TCC2BeginTimer();
   //TCC1BeginTimer();
   //TCC0BeginTimer();
   ExtInterrupt5Setup();
   pastMillis = millis();

    __enable_irq();
}

void loop() {
   
   if (millis() - pastMillis > 100) {
      pastMillis = millis();
      strip.setPixelColor(0, ran);
      strip.show();
      (ran < 0x00800000 ? ran*=2 : ran = 0x01);
      Serial.println(flag);
   }
}
