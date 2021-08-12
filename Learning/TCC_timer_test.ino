// SAMD51 (Adafruit Metro M4) Timer Examples from MartinL

#include <sam.h>

// Needed for the Feather M0 board
#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL) // Required for Serial on Zero based boards
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

const int Timer0 = 9;
const int Timer1 = 12;
const int Timer3 = 4;
const int Timer4 = 6;

int long PotValue_Led = 240;
int long PotValue_Drive = 150;
int long PotValue_Pump = 50;


void setup() {
  
  // Pin Definintions
  //pinMode(Timer5, OUTPUT);
  //pinMode(Timer4, OUTPUT);
  //pinMode(L_PWM, OUTPUT);
  //pinMode(D_PWM, OUTPUT);
  //pinMode(P_PWM, OUTPUT);
 
  // Start Timers
  
  TCC0StartTimer();                                     // startup TCC0 timer for drive PWM output
  TCC1StartTimer();                                     // startup TCC1 timer
  TCC3StartTimer();                                     // startup TCC3 timer
  TCC4StartTimer();                                     // startup TCC4 timer
}

void loop() {
  
  //example to change the TCC0 duty-cycle between 25% and 75% every second:
  //TCC0->CCBUF[0].reg = 29999;                        // Set-up the CCBUF (Counter Compare Buffered), channel 0 register for 25% duty-cycle
  //while (!TCC0->STATUS.bit.CCBUFV0);                 // Wait for synchronization, using the STATUS register          
  //delay(1000);                                       // Wait for 1 second
  //TCC0->CCBUF[0].reg = 89999;                        // Set-up the CCBUF (Counter Compare Buffered), channel 0 register for 75% duty-cycle
  //while (!TCC0->STATUS.bit.CCBUFV0);                 // Wait for synchronization, using the STATUS register
  //delay(1000);
  //Use CCBUF and PERBUF registers to change duty-cycle and or frequency on the fly.
 
}

void TCC0StartTimer() { // Automatically shares clock with TCC1
  // Set up the generic clock (GCLK7) to clock timer TCC0 
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |        // Divide the 48MHz clock source by divisor 1: 48MHz/1 = 48MHz
                         GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |         // Enable GCLK7
                         GCLK_GENCTRL_SRC_DFLL;       // Select 48MHz DFLL clock source
                         //GCLK_GENCTRL_SRC_DPLL1;     // Select 100MHz DPLL clock source
                         //GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL7);                // Wait for synchronization  

  GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN |         // Enable the TCC0 peripheral channel
                          GCLK_PCHCTRL_GEN_GCLK7;     // Connect generic clock 7 to TCC0

  // Enable the peripheral multiplexer on pin D9
  PORT->Group[g_APinDescription[9].ulPort].PINCFG[g_APinDescription[9].ulPin].bit.PMUXEN = 1;
  
  // Set the D9 (PORT_PA20) peripheral multiplexer to peripheral (even port number) E(6): TCC0, Channel 0
  PORT->Group[g_APinDescription[9].ulPort].PMUX[g_APinDescription[9].ulPin >> 1].reg |= PORT_PMUX_PMUXE(6);
  
  TCC0->CTRLA.reg = TC_CTRLA_PRESCALER_DIV8 |         // Set prescaler to 8, 48MHz/8 = 6MHz
                    TC_CTRLA_PRESCSYNC_PRESC;         // Set the reset/reload to trigger on prescaler clock                 

  TCC0->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;              // Set-up TCC0 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC0->SYNCBUSY.bit.WAVE)                     // Wait for synchronization

  TCC0->PER.reg = 119999;                             // Set-up the PER (period) register 50Hz PWM
  while (TCC0->SYNCBUSY.bit.PER);                     // Wait for synchronization
  
  TCC0->CC[0].reg = 59999;                            // Set-up the CC (counter compare), channel 0 register for 50% duty-cycle
  while (TCC0->SYNCBUSY.bit.CC0);                     // Wait for synchronization

  TCC0->CTRLA.bit.ENABLE = 1;                         // Enable timer TCC0
  while (TCC0->SYNCBUSY.bit.ENABLE);                  // Wait for synchronization
}

void TCC4StartTimer() {
  // Set up the generic clock (GCLK5) to clock timer TCC4 
  //MCLK->APBDMASK.reg |= MCLK_APBDMASK_TCC4;         // Activate timer TCC4 not needed?
  
  GCLK->GENCTRL[5].reg = GCLK_GENCTRL_DIV(1) |        // Divide the 48MHz clock source by divisor 1: 48MHz/1 = 48MHz
                         GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |         // Enable GCLK7
                         GCLK_GENCTRL_SRC_DFLL;       // Select 48MHz DFLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL5);

  GCLK->PCHCTRL[38].reg = GCLK_PCHCTRL_CHEN |         // Enable the TCC4 peripheral channel
                          GCLK_PCHCTRL_GEN_GCLK5;     // Connect generic clock 7 to TCC4

  // Enable the peripheral multiplexer on pin D6
  PORT->Group[g_APinDescription[6].ulPort].PINCFG[g_APinDescription[6].ulPin].bit.PMUXEN = 1;
  
  // Set the D6 (PORT_PB15) peripheral multiplexer to peripheral (even port number) E(5): TCC4, Channel 1
  PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg |= PORT_PMUX_PMUXO(5);

  TCC4->CTRLA.reg = TC_CTRLA_PRESCALER_DIV8 |         // Set prescaler to 8, 48MHz/8 = 6MHz
                    TC_CTRLA_PRESCSYNC_PRESC;         // Set the reset/reload to trigger on prescaler clock                 

  TCC4->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;              // Set-up TCC1 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC4->SYNCBUSY.bit.WAVE);

  TCC4->PER.reg = 1999;                               // Set-up the PER (period) register 3.2kHz? PWM
  while (TCC4->SYNCBUSY.bit.PER);
  
  TCC4->CC[1].reg = 599;                              // PB15 is on channel 1 so CC[1] must be used instead of CC[0]
  while (TCC4->SYNCBUSY.bit.CC1);            // SYNCBUSY.bit.CC1 not SYNCBUSY.bit.CC0

  TCC4->CTRLA.bit.ENABLE = 1;                         // Enable timer TCC1
  while (TCC4->SYNCBUSY.bit.ENABLE);
}

void TCC1StartTimer() { // Automatically shares cock with TCC0
  // Set up the generic clock (GCLK6) to clock timer TCC1
  // TCC1 is already set to GCLK7 with TCC0
   
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(4) |        // Divide the 48MHz clock source by divisor 1: 48MHz/1 = 48MHz
                         GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |         // Enable GCLK6
                         GCLK_GENCTRL_SRC_DFLL;       // Select 48MHz DFLL clock source
                         //GCLK_GENCTRL_SRC_DPLL1;     // Select 100MHz DPLL clock source
                         //GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL6);                // Wait for synchronization  

  GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN |         // Enable the TCC4 peripheral channel
                          GCLK_PCHCTRL_GEN_GCLK7;     // Connect generic clock 6 to TCC1

  // Enable the peripheral multiplexer on pin D12
  PORT->Group[g_APinDescription[12].ulPort].PINCFG[g_APinDescription[12].ulPin].bit.PMUXEN = 1;

  // Set the D12 (PORT_PA17) peripheral multiplexer to peripheral (even port number) O(5): TCC1, Channel 1
  PORT->Group[g_APinDescription[12].ulPort].PMUX[g_APinDescription[12].ulPin >> 1].reg |= PORT_PMUX_PMUXO(5);
  
  TCC1->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV8 |        // Set prescaler to 8, 48MHz/8 = 6MHz
                    TCC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock                 

  TCC1->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;              // Set-up TCC1 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC1->SYNCBUSY.bit.WAVE);

  TCC1->PER.reg = 199;                               // Set-up the PER (period) register 50Hz PWM
  while (TCC1->SYNCBUSY.bit.PER);
  
  TCC1->CC[1].reg = 99;                              // CC[1] used, not CC[0] because of odd peripheral?
  while (TCC1->SYNCBUSY.bit.CC1);

  TCC1->CTRLA.bit.ENABLE = 1;                         // Enable timer TCC1
  while (TCC1->SYNCBUSY.bit.ENABLE);
}

void TCC3StartTimer() {
  // Set up the generic clock (GCLK7) to clock timer TCC3
   
  GCLK->GENCTRL[6].reg = GCLK_GENCTRL_DIV(4) |        // Divide the 48MHz clock source by divisor 1: 48MHz/1 = 48MHz
                         GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |         // Enable GCLK6
                         GCLK_GENCTRL_SRC_DFLL;       // Select 48MHz DFLL clock source
                         //GCLK_GENCTRL_SRC_DPLL1;     // Select 100MHz DPLL clock source
                         //GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL6);                // Wait for synchronization  

  GCLK->PCHCTRL[29].reg = GCLK_PCHCTRL_CHEN |         // Enable the TCC3 peripheral channel
                          GCLK_PCHCTRL_GEN_GCLK6;     // Connect generic clock 6 to TCC3

  // Enable the peripheral multiplexer on pin D4
  PORT->Group[g_APinDescription[4].ulPort].PINCFG[g_APinDescription[4].ulPin].bit.PMUXEN = 1;
  
  // Set the D4 (PORT_PB13) peripheral multiplexer to peripheral (even port number) O(5): TCC3, Channel 1
  PORT->Group[g_APinDescription[4].ulPort].PMUX[g_APinDescription[4].ulPin >> 1].reg |= PORT_PMUX_PMUXO(5);
  
  TCC3->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV8 |        // Set prescaler to 8, 48MHz/8 = 6MHz
                    TCC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock                 

  TCC3->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;              // Set-up TCC3 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC3->SYNCBUSY.bit.WAVE)                     // Wait for synchronization

  TCC3->PER.reg = 1999;                               // Set-up the PER (period) register 50Hz PWM
  while (TCC3->SYNCBUSY.bit.PER);                     // Wait for synchronization
  
  TCC3->CC[0].reg = 999;                              // Set-up the CC (counter compare), channel 0 register for 50% duty-cycle
  while (TCC3->SYNCBUSY.bit.CC0);                     // Wait for synchronization

  TCC3->CTRLA.bit.ENABLE = 1;                         // Enable timer TCC3
  while (TCC3->SYNCBUSY.bit.ENABLE);                  // Wait for synchronization
}
