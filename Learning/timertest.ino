// SAMD51 (M4) Timer Examples from MartinL

#include <sam.h>

// Needed for the Feather M0 board
#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL) // Required for Serial on Zero based boards
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

const int Timer5 = 6;
const int Timer4 = 7;
const int L_PWM = 9;
const int D_PWM = 5;
const int P_PWM = 11;

int long PotValue_Led = 240;
int long PotValue_Drive = 150;
int long PotValue_Pump = 50;


void setup() {
  
  /* Unnecessary for ARM boards (SAMD51)
  pinMode(Timer5, OUTPUT);
  pinMode(Timer4, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(D_PWM, OUTPUT);
  pinMode(P_PWM, OUTPUT);
 */
 
  // Start Timers
  tc4StartTimer();                                    // startup TC4 timer
  tc5StartTimer();                                    // startup TC5 timer
  
  TCC0StartTimer();                                   // startup TCC0 timer for drive PWM output
  TCC4StartTimer();                                   // startup TCC4 timer
}

void loop() {
  
  //example to change the TCC0 duty-cycle between 25% and 75% every second:
  TCC0->CCBUF[0].reg = 29999;                        // Set-up the CCBUF (Counter Compare Buffered), channel 0 register for 25% duty-cycle
  while (!TCC0->STATUS.bit.CCBUFV0);                 // Wait for synchronization, using the STATUS register          
  delay(1000);                                       // Wait for 1 second
  TCC0->CCBUF[0].reg = 89999;                        // Set-up the CCBUF (Counter Compare Buffered), channel 0 register for 75% duty-cycle
  while (!TCC0->STATUS.bit.CCBUFV0);                 // Wait for synchronization, using the STATUS register
  delay(1000);
  //Use CCBUF and PERBUF registers to change duty-cycle and or frequency on the fly.
 
}


// TC5 16bit Timer (this timer works) 
void tc5StartTimer() {
  
  MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC5;           // Activate timer TC5 (TC5 works without activating?)
  
  // Set up the generic clock (GCLK7) used to clock timers
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(3) |       // Divide the 48MHz clock source by divisor 3: 48MHz/3 = 16MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         GCLK_GENCTRL_SRC_DFLL;      // Generate from 48MHz DFLL clock source
                         //GCLK_GENCTRL_SRC_DPLL1;     // Generate from 100MHz DPLL clock source
                         //GCLK_GENCTRL_SRC_DPLL0;     // Generate from 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL7);               // Wait for synchronization 

  GCLK->PCHCTRL[30].reg = GCLK_PCHCTRL_CHEN |        // Enable perhipheral channel
                         GCLK_PCHCTRL_GEN_GCLK7;     // Connect generic clock 7 to TC5, PCHCTRL[30]

  // Enable the peripheral multiplexer on pin D6
  //PORT->Group[g_APinDescription[A1].ulPort].PINCFG[g_APinDescription[A1].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[6].ulPort].PINCFG[g_APinDescription[6].ulPin].bit.PMUXEN = 1;
 
  // Set D6 (Port PB15) the peripheral multiplexer to peripheral (odd) O(4): TC5, Channel 1 TC5, Channel 1 (peripheral A=0, B=1, C=2, D=3, E=4, etc)
  PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg |= PORT_PMUX_PMUXO(4);
 
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV16 | // Set prescaler to 16, 16MHz/16 = 1MHz
                           TC_CTRLA_PRESCSYNC_PRESC | // Set the reset/reload to trigger on prescaler clock
                           TC_CTRLA_MODE_COUNT16;     // Set the counter to 16-bit mode

  TC5->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MPWM;       // Set-up TC5 timer for Match PWM mode (MPWM)
  
  // PWM Frequency = GCLK frequency / (N * (TOP + 1)
  // N = timer prescaler (1,2,4,8,16,64,256,1024)
  // TOP = the value of the CC) register (in MPWM mode)
  // The PWM resolution formula is:
  // PWM Resolution = log(CC) + 1) / log(2)
  // PWM frequency = 16MHz / 16 * (19999 + 1)) = 50Hz
  // PWM resolution = log(19999+1)/log(2) = 14bits
  // The CC1 register determines the PWM duty cycle from 0 (0%) upto the CC0 register value (100%)
  // To change dutycycle on the fly use the CCBUF1 register (TC5->COUNT16.CCBUF[1].reg = 4999; Set the duty cycle buffered
  
  TC5->COUNT16.CC[0].reg = 19999;                    // Use CC0 register as TOP value, set for 50Hz PWM
  while (TC5->COUNT16.SYNCBUSY.bit.CC0);             // Wait for synchronization

  TC5->COUNT16.CC[1].reg = 9999;                     // Set the duty cycle to 50% (CC1 half of CC0)
  while (TC5->COUNT16.SYNCBUSY.bit.CC1);             // Wait for synchronization

  TC5->COUNT16.CTRLA.bit.ENABLE = 1;                 // Enable timer TC5
  while (TC5->COUNT16.SYNCBUSY.bit.ENABLE);          // Wait for synchronization
}

// TC4 16bit Timer (this timer does not work PWM output not correct should be same as TC5 output?) 
void tc4StartTimer() {
  
  MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC4;           // Activate timer TC4 (TC5 works without activating?)
  
  // Set up the generic clock (GCLK7) used for clock timers
  GCLK->GENCTRL[6].reg = GCLK_GENCTRL_DIV(3) |       // Divide the 48MHz clock source by divisor 3: 48MHz/3 = 16MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK6
                         GCLK_GENCTRL_SRC_DFLL;      // Generate from 48MHz DFLL clock source
                         //GCLK_GENCTRL_SRC_DPLL1;     // Generate from 100MHz DPLL clock source
                         //GCLK_GENCTRL_SRC_DPLL0;     // Generate from 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL6);               // Wait for synchronization 

  GCLK->PCHCTRL[30].reg = GCLK_PCHCTRL_CHEN |        // Enable perhipheral channel
                         GCLK_PCHCTRL_GEN_GCLK6;     // Connect generic clock 6 to TC4, PCHCTRL[30]

  // Enable the peripheral multiplexer on pin D7
  PORT->Group[g_APinDescription[7].ulPort].PINCFG[g_APinDescription[7].ulPin].bit.PMUXEN = 1;
 
  // Set D7 (Port PB12) the peripheral multiplexer to peripheral (even) E(4): TC4, Channel 0 (peripheral A=0, B=1, C=2, D=3, E=4, etc)
  PORT->Group[g_APinDescription[7].ulPort].PMUX[g_APinDescription[7].ulPin >> 1].reg |= PORT_PMUX_PMUXE(4);
 
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV16 | // Set prescaler to 16, 16MHz/16 = 1MHz
                           TC_CTRLA_PRESCSYNC_PRESC | // Set the reset/reload to trigger on prescaler clock
                           TC_CTRLA_MODE_COUNT16;     // Set the counter to 16-bit mode

  TC4->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MPWM;       // Set-up TC4 timer for Match PWM mode (MPWM)
  
  // PWM Frequency = GCLK frequency / (N * (TOP + 1)
  // N = timer prescaler (1,2,4,8,16,64,256,1024)
  // TOP = the value of the CC) register (in MPWM mode)
  // The PWM resolution formula is:
  // PWM Resolution = log(CC) + 1) / log(2)
  // PWM frequency = 16MHz / 16 * (19999 + 1)) = 50Hz
  // PWM resolution = log(19999+1)/log(2) = 14bits
  // The CC1 register determines the PWM duty cycle from 0 (0%) upto the CC0 register value (100%)
  // To change dutycycle on the fly use the CCBUF1 register (TC5->COUNT16.CCBUF[1].reg = 4999; Set the duty cycle buffered
  
  TC4->COUNT16.CC[0].reg = 19999;                    // Use CC0 register as TOP value, set for 50Hz PWM
  while (TC4->COUNT16.SYNCBUSY.bit.CC0);             // Wait for synchronization

  TC4->COUNT16.CC[1].reg = 9999;                     // Set the duty cycle to 50% (CC1 half of CC0)
  while (TC4->COUNT16.SYNCBUSY.bit.CC1);             // Wait for synchronization

  TC4->COUNT16.CTRLA.bit.ENABLE = 1;                 // Enable timer TC4
  while (TC4->COUNT16.SYNCBUSY.bit.ENABLE);          // Wait for synchronization
}


// Adafruit Metro M4 Only: Set-up digital pin D7 to output 50Hz, single slope PWM with a 50% duty cycle
// This timer works
void TCC0StartTimer() {
  // Set up the generic clock (GCLK7) to clock timer TCC0 
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 48MHz clock source by divisor 1: 48MHz/1 = 48MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
                         //GCLK_GENCTRL_SRC_DPLL1;     // Select 100MHz DPLL clock source
                         //GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL7);               // Wait for synchronization  

  GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC0 peripheral channel
                          GCLK_PCHCTRL_GEN_GCLK7;    // Connect generic clock 7 to TCC0

  // Enable the peripheral multiplexer on pin D9
  PORT->Group[g_APinDescription[9].ulPort].PINCFG[g_APinDescription[9].ulPin].bit.PMUXEN = 1;
  
  // Set the D9 (PORT_PA20) peripheral multiplexer to peripheral (even port number) E(6): TCC0, Channel 0
  PORT->Group[g_APinDescription[9].ulPort].PMUX[g_APinDescription[9].ulPin >> 1].reg |= PORT_PMUX_PMUXE(6);
  
  TCC0->CTRLA.reg = TC_CTRLA_PRESCALER_DIV8 |        // Set prescaler to 8, 48MHz/8 = 6MHz
                    TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock                 

  TCC0->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;             // Set-up TCC0 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC0->SYNCBUSY.bit.WAVE)                    // Wait for synchronization

  TCC0->PER.reg = 119999;                            // Set-up the PER (period) register 50Hz PWM
  while (TCC0->SYNCBUSY.bit.PER);                    // Wait for synchronization
  
  TCC0->CC[0].reg = 59999;                           // Set-up the CC (counter compare), channel 0 register for 50% duty-cycle
  while (TCC0->SYNCBUSY.bit.CC0);                    // Wait for synchronization

  TCC0->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC0
  while (TCC0->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization
}

// This timer does not work - Also tried TCC1, TCC2, TCC3 all no output???
void TCC4StartTimer() {
  // Set up the generic clock (GCLK7) to clock timer TCC4 
  //MCLK->APBDMASK.reg |= MCLK_APBDMASK_TCC4;           // Activate timer TCC4 not needed?
  
  GCLK->GENCTRL[5].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 48MHz clock source by divisor 1: 48MHz/1 = 48MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
                         //GCLK_GENCTRL_SRC_DPLL1;     // Select 100MHz DPLL clock source
                         //GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL5);               // Wait for synchronization  

  GCLK->PCHCTRL[38].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC4 peripheral channel
                          GCLK_PCHCTRL_GEN_GCLK5;    // Connect generic clock 7 to TCC4

  // Enable the peripheral multiplexer on pin D5
  PORT->Group[g_APinDescription[5].ulPort].PINCFG[g_APinDescription[5].ulPin].bit.PMUXEN = 1;
  
  // Set the D5 (PORT_PB14) peripheral multiplexer to peripheral (even port number) E(5): TCC4, Channel 0
  PORT->Group[g_APinDescription[5].ulPort].PMUX[g_APinDescription[5].ulPin >> 1].reg |= PORT_PMUX_PMUXE(5);
  
  TCC4->CTRLA.reg = TC_CTRLA_PRESCALER_DIV8 |        // Set prescaler to 8, 48MHz/8 = 6MHz
                    TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock                 

  TCC4->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;             // Set-up TCC1 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC4->SYNCBUSY.bit.WAVE)                    // Wait for synchronization

  TCC4->PER.reg = 119999;                            // Set-up the PER (period) register 50Hz PWM
  while (TCC4->SYNCBUSY.bit.PER);                    // Wait for synchronization
  
  TCC4->CC[0].reg = 59999;                           // Set-up the CC (counter compare), channel 0 register for 50% duty-cycle
  while (TCC4->SYNCBUSY.bit.CC0);                    // Wait for synchronization

  TCC4->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC1
  while (TCC4->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization
}
