//SAMD51 Metro M4 Epress timer example  (MartinL)

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
  
  /* Unnecessary for ARM boards (SAMD51)?
  pinMode(Timer5, OUTPUT);
  pinMode(Timer4, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(D_PWM, OUTPUT);
  pinMode(P_PWM, OUTPUT);
 */
 
  // Start Timers
  tc4SetupTimer();                                    // startup TC4 timer
  tc5SetupTimer();                                    // startup TC5 timer
  
  TCC0SetupTimer();                                   // startup TCC0 timer for drive PWM output
  TCC4SetupTimer();                                   // startup TCC4 timer
}

void loop() {

   /*TCC0->CTRLA.bit.ENABLE = 1;           // Enable timer TCC0
   while (TCC0->SYNCBUSY.bit.ENABLE);    // Wait for synchronization */

   /*TCC0->CTRLA.bit.ENABLE = 0;           // Disable timer TCC0
   while (TCC0->SYNCBUSY.bit.ENABLE);    // Wait for synchronization */

   /*TCC0->CTRLBSET.reg = TCC_CTRLBSET_CMD_STOP;   // Stop timer TCC0
   while (TCC0->SYNCBUSY.bit.CTRLB);             // Wait for synchronization */

   /*TCC0->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;   // Start timer TCC0
   while (TCC0->SYNCBUSY.bit.CTRLB);                  // Wait for synchronization */

   /*TCC1->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;   // Start timer TCC1
   while (TCC1->SYNCBUSY.bit.CTRLB);                  // Wait for synchronization */

   /*TCC0->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;    // Initiate read synchronization of the COUNT register
   while (TCC0->SYNCBUSY.bit.CTRLB);                  // Wait for CTRLB register write synchronization
   while (TCC0->SYNCBUSY.bit.COUNT);                  // Wait for COUNT register read synchronization
   Serial.println(TCC0->COUNT.reg);                   // Read the COUNT register */
  
  /*Changes to the buffered registers (CCBUF(x) and PERB) only occur at the start of the timer cycle,
  where as changes to CC(x) and the PER register appear immediately at the PWM output. */
  TCC0->CCBUF[0].reg = 29999;                  // Set-up the TCC0 Counter Compare Buffered channel 0 register
  while (!TCC0->STATUS.bit.CCBUFV0);     
  delay(1000);
  TCC0->CCBUF[0].reg = 89999;
  while (!TCC0->STATUS.bit.CCBUFV0);
  delay(1000);

  /*TCC1->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;   // Retrigger the timer's One/Multi-Shot pulses
   while (TCC1->SYNCBUSY.bit.CTRLB);  */
 
}

/* Normal PWM (NPWM) allows indepenent output generation on both of its channels: CC0 and CC1,
 *    but offers no control of the waveforms' frequency.
 *  Match PWM (MPWM) sacrifices CC0 to provide frequency control (for PER) while CC1 determines the duty-cycle.
 *    The resulting waveform is output on CC1. 
 */

/*PB12 can also be connected either to TCC3/WO0 on switch F(5) or TCC0/WO[0] on switch G(6)
 * as shown in the I/O Multiplexing and Considerations table in the SAMD51 datasheet.
*/

/* On the SAMD51 some of the TCCx timers share the same generic clock:
 * TCC0/TCC1 share PHCTRL[25] 
 * TCC2/TCC3 share PHCTRL[29]
 * TCC4 has PHCTRL[38]
 */

/*PA16 - TC2/WO[0], TCC1/WO[0], TCC0/WO[4]      PA17 - TC2/WO[1], TCC1/WO[1], TCC0/WO[5]
 * If you're using a WO[4] to WO[7] on TCC1, this will also map to CC[0] to CC[3] respectively.
 */

/*Each timer has single period (PER) register, but a separate counter compare (CC(x)) register for each channel.
 * This means that each timer can be configured to operate at one frequency (PER), but with different duty-cycles (CC(x)) on each channel.
 * 
 * Each port pin has it's own pin configuration (PINCONFIG) registers, but shares the port multiplexer (PMUX) register with its neighbouring pin.
 * PMUX[0] sets the port pins PA00 and PA01, PMUX[1] sets PA02 and PA03 and so on. The port pairs can still be set to different peripherals though
*/

void tc5SetupTimer() {
  
  //MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC5;           // TC0, TC1, and TC2 are the only timers not active by default
  
  // Set up the generic clock (GCLK7) used to clock timers
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(3) |       // Divide the 48MHz clock source by divisor 3: 48MHz/3 = 16MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         GCLK_GENCTRL_SRC_DFLL;      // Generate from 48MHz DFLL clock source
                         //GCLK_GENCTRL_SRC_DPLL1;     // Generate from 100MHz DPLL clock source
                         //GCLK_GENCTRL_SRC_DPLL0;     // Generate from 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL7);               // Wait for synchronization 

  GCLK->PCHCTRL[30].reg = GCLK_PCHCTRL_CHEN |        // Enable perhipheral channel
                         GCLK_PCHCTRL_GEN_GCLK7;     // Connect generic clock 7 to TC5 (PCHCTRL[30])

  // Enable the peripheral multiplexer on pin D6
   PORT->Group[g_APinDescription[6].ulPort].PINCFG[g_APinDescription[6].ulPin].bit.PMUXEN = 1;
 
  // Set D6 (Port PB15) the peripheral multiplexer to peripheral (odd) O(4): TC5, Channel 1 TC5, Channel 1 (peripheral A=0, B=1, C=2, D=3, E=4, etc)
  PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg |= PORT_PMUX_PMUXO(4);
                                                                                                                                     //PORT_PMUX_PMUXO() for odd peripherals (PB15)

  TC5->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV16 | // Set prescaler to 16, 16MHz/16 = 1MHz
                           TC_CTRLA_PRESCSYNC_PRESC | // Set the reset/reload to trigger on prescaler clock
                           TC_CTRLA_MODE_COUNT16;     // Set the counter to 16-bit mode

  TC5->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MPWM;       // Set-up TC5 timer for Match PWM mode (MPWM)

  /* PWM Frequency = GCLK frequency / (N * (TOP + 1)
  // N = timer prescaler (1,2,4,8,16,64,256,1024)
  // TOP = the value of the CC0 register (in MPWM mode)
  // The PWM resolution formula is:
  // PWM Resolution = log(CC0 + 1) / log(2)
  // PWM frequency = 16MHz /( 16 * (19999 + 1)) = 50Hz
  // PWM resolution = log(19999+1)/log(2) = 14bits
  // The CC1 register determines the PWM duty cycle from 0 (0%) upto the CC0 register value (100%)
  // To change dutycycle on the fly use the CCBUF1 register (TC5->COUNT16.CCBUF[1].reg = 4999; Set the duty cycle buffered
  */
  
  TC5->COUNT16.CC[0].reg = 19999;                     // Use CC0 register as TOP value, set for 50Hz PWM
  while (TC5->COUNT16.SYNCBUSY.bit.CC0);      // Wait for synchronization
                                                                          // waiting for synch is really only necessary when accessing the peripherals in quick succession

  TC5->COUNT16.CC[1].reg = 9999;                     // Set the duty cycle to 50% (CC1 half of CC0)
  while (TC5->COUNT16.SYNCBUSY.bit.CC1);

  TC5->COUNT16.CTRLA.bit.ENABLE = 1;                 // Enable timer TC5
  while (TC5->COUNT16.SYNCBUSY.bit.ENABLE);
}

void tc4SetupTimer() {
  
  //MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC4;           // Activate timer TC4
  
  // Set up the generic clock (GCLK6) used to clock timers
  GCLK->GENCTRL[6].reg = GCLK_GENCTRL_DIV(3) |       // Divide the 48MHz clock source by divisor 3: 48MHz/3 = 16MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK6
                         GCLK_GENCTRL_SRC_DFLL;      // Generate from 48MHz DFLL clock source
                         //GCLK_GENCTRL_SRC_DPLL1;     // Generate from 100MHz DPLL clock source
                         //GCLK_GENCTRL_SRC_DPLL0;     // Generate from 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL6); 

  GCLK->PCHCTRL[30].reg = GCLK_PCHCTRL_CHEN |        // Enable perhipheral channel
                         GCLK_PCHCTRL_GEN_GCLK6;     // Connect generic clock 6 to TC4 (PCHCTRL[30] on Metro?)

  // Enable the peripheral multiplexer on pin D4
  PORT->Group[g_APinDescription[4].ulPort].PINCFG[g_APinDescription[4].ulPin].bit.PMUXEN = 1;
 
  // Set D4 (Port PB13) the peripheral multiplexer to peripheral (even) E(4): TC4, Channel 0 (peripheral A=0, B=1, C=2, D=3, E=4, etc)
  PORT->Group[g_APinDescription[4].ulPort].PMUX[g_APinDescription[4].ulPin >> 1].reg |= PORT_PMUX_PMUXO(4);
                                                                                                                                     //PORT_PMUX_PMUXO() for odd peripherals (PB13)

  TC4->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV16 | // Set prescaler to 16, 16MHz/16 = 1MHz
                           TC_CTRLA_PRESCSYNC_PRESC | // Set the reset/reload to trigger on prescaler clock
                           TC_CTRLA_MODE_COUNT16;     // Set the counter to 16-bit mode (COUNT16)

  TC4->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MPWM;       // Set-up TC4 timer for Match PWM mode (MPWM)
  
  /* PWM Frequency = GCLK frequency / (N * (TOP + 1)
  // N = timer prescaler (1,2,4,8,16,64,256,1024)
  // TOP = the value of the CC0 register (in MPWM mode)
  // The PWM resolution formula is:
  // PWM Resolution = log(CC0 + 1) / log(2)
  // PWM frequency = 16MHz / 16 * (9999 + 1)) = 100Hz
  // PWM resolution = log(9999+1)/log(2) = 13bits
  // The CC1 register determines the PWM duty cycle from 0 (0%) upto the CC0 register value (100%)
  // To change dutycycle on the fly use the CCBUF1 register (TC5->COUNT16.CCBUF[1].reg = 4999; Set the duty cycle buffered
  */

  TC4->COUNT16.CC[0].reg = 9999;                    // Use CC0 register as TOP value, set for 100Hz PWM
  while (TC4->COUNT16.SYNCBUSY.bit.CC0);

  TC4->COUNT16.CC[1].reg = 4999;                     // Set the duty cycle to 50% (CC1 half of CC0)
  while (TC4->COUNT16.SYNCBUSY.bit.CC1);

  TC4->COUNT16.CTRLA.bit.ENABLE = 1;                 // Enable timer TC4
  while (TC4->COUNT16.SYNCBUSY.bit.ENABLE);
}

void tc2SetupTimer() { // Note: this is setup with GCLK7, it may not be wise (or even possible) to connect multiple TCx/TCCx to the same clock

   MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC2;           // Activate timer TC2

      // Set up the generic clock (GCLK7) used to clock timers 
      GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(3) |       // Divide the 48MHz clock source by divisor 3: 48MHz/3 = 16MHz
                                    GCLK_GENCTRL_IDC |                  // Set the duty cycle to 50/50 HIGH/LOW
                                    GCLK_GENCTRL_GENEN |           // Enable GCLK7
                                    GCLK_GENCTRL_SRC_DFLL;     // Generate from 48MHz DFLL clock source
      while (GCLK->SYNCBUSY.bit.GENCTRL7);             // Wait for synchronization

      GCLK->PCHCTRL[26].reg = GCLK_PCHCTRL_CHEN |         // Enable perhipheral channel
                                       GCLK_PCHCTRL_GEN_GCLK7;      // Connect generic clock 7 to TC2 at 16MHz

      // Enable the peripheral multiplexer on digital pin 12
      PORT->Group[g_APinDescription[12].ulPort].PINCFG[g_APinDescription[12].ulPin].bit.PMUXEN = 1; 
      // Set the peripheral multiplexer for D12 to peripheral E(4): TC2, Channel 0
      PORT->Group[g_APinDescription[12].ulPort].PMUX[g_APinDescription[12].ulPin >> 1].reg |= PORT_PMUX_PMUXO(4);
  
      TC2->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV16 |         // Set prescaler to 64, 16MHz/16 = 1MHz
                                                TC_CTRLA_PRESCSYNC_PRESC |        // Set the reset/reload to trigger on prescaler clock
                                                TC_CTRLA_MODE_COUNT16;            // Set the counter to 16-bit mode
                                                
      TC2->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MPWM;       // Set-up TC2 timer for Match PWM mode (MPWM)
      
      TC2->COUNT16.CC[0].reg = 19999;                    // Use CC0 register as TOP value, set for 50Hz PWM  
      while (TC2->COUNT16.SYNCBUSY.bit.CC0);
      TC2->COUNT16.CC[1].reg = 9999;                     // Set the duty cycle to 50% (CC1 half of CC0)
      while (TC2->COUNT16.SYNCBUSY.bit.CC1);
      TC2->COUNT16.CTRLA.bit.ENABLE = 1;                 // Enable timer TC2
      while (TC2->COUNT16.SYNCBUSY.bit.ENABLE);
   
}

void TCC0SetupTimer() {

   NVIC_SetPriority(TCC0_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TCC0 to 0 (highest) 
   NVIC_EnableIRQ(TCC0_IRQn);         // Connect TCC0 to Nested Vector Interrupt Controller (NVIC)
   TCC0->INTENSET.reg |= TCC_INTENSET_OVF;  // Enable overflow interrupts
   // may also use TCC_INTENSET_MC1 or TCC_INTENSET_MC0 for match compare interrupts on channels 1 and 0
   
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
  while (TCC0->SYNCBUSY.bit.WAVE);                    // Wait for synchronization

  TCC0->PER.reg = 119999;                            // Set-up the PER (period) register 50Hz PWM
  while (TCC0->SYNCBUSY.bit.PER);            // There is only a single PER and PERBUF register per timer so square brackets cannot be used
  
  TCC0->CC[0].reg = 59999;                           // Set-up the CC (counter compare), channel 0 register for 50% duty-cycle
  while (TCC0->SYNCBUSY.bit.CC0);

  TCC0->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC0
  while (TCC0->SYNCBUSY.bit.ENABLE);
}

/* The SAMD51 TCCx timers each have a number of interrupt handler functions TCCx_y_Handler(), 
       where x is the timer number and y is the handler number.
     TCC1_0_Handler() calls all timer interrupts except match compare interrupts,
       where MC0 calls TCC1_1_Handler(), MC1 calls TCC1_2_Handler() and so on... 
     Timer interrupt flag handlers in section 10.2.2 (NVIC) Interrupt Line Mapping table 
     Peripheral functions defined in CMSIS file ( "samd51j19.h" )*/

void TCC0_Handler() {  // Calling the appropriate interrupt handler means we don't need to test each flag (more optimized than SAMD21)
  if (TCC0->INTENSET.bit.OVF && TCC0->INTFLAG.bit.OVF) {   // Test if the OVF (Overflow) interrupt has occured

    TCC0->INTFLAG.bit.OVF = 1;                            // Clear the OVF interrupt flag (flag bits must always be manualy cleared)
  } 
  
  if (TCC0->INTENSET.bit.MC0 && TCC0->INTFLAG.bit.MC0) {   // Test if the MC0 (Match Compare Channel 0) interrupt has occured

    TCC0->INTFLAG.bit.MC0 = 1;                            // Clear the MC0 interrupt flag (regardless of which handler is called)
  } 

  if (TCC0->INTENSET.bit.MC1 && TCC0->INTFLAG.bit.MC1) {   // Test if the MC1 (Match Compare Channel 1) interrupt has occured

    TCC0->INTFLAG.bit.MC1 = 1;                            // Clear the MC1 interrupt flag 
  }
  // Add other channels as required...
}

void TCC1SetupTimer() {
   /* TCC1 overflow (OVF) interrupt:
   NVIC_SetPriority(TCC1_0_IRQn, 0);      // Set the Nested Vector Interrupt Controller (NVIC) priority for TCC1 to 0 (highest)
   NVIC_EnableIRQ(TCC1_0_IRQn);           // Connect the TCC1 timer to the Nested Vector Interrupt Controller (NVIC)  */

   /* TCC1 match compare channel 2 (MC2) interrupt
   NVIC_SetPriority(TCC1_3_IRQn, 0);      // Set the Nested Vector Interrupt Controller (NVIC) priority for TCC1 to 0 (highest)
   NVIC_EnableIRQ(TCC1_3_IRQn);           // Connect the TCC1 timer to the Nested Vector Interrupt Controller (NVIC) */
   
  // Set up the generic clock (GCLK7) to clock timer TCC1 
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 48MHz clock source by divisor 1: 48MHz/1 = 48MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL7);

  GCLK->PCHCTRL[TCC1_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC1 perhipheral channel
                                                         GCLK_PCHCTRL_GEN_GCLK7;    // Connect generic clock 7 to TCC1

  // Enable the peripheral multiplexer on pin D10
  PORT->Group[g_APinDescription[10].ulPort].PINCFG[g_APinDescription[10].ulPin].bit.PMUXEN = 1;
  
  // Set the D10 (PORT_PA18) peripheral multiplexer to peripheral (even port number) F(5): TCC1, Channel 2
  PORT->Group[g_APinDescription[10].ulPort].PMUX[g_APinDescription[10].ulPin >> 1].reg |= PORT_PMUX_PMUXE(5);
  
  TCC1->CTRLA.reg = TC_CTRLA_PRESCALER_DIV8 |        // Set prescaler to 8, 48MHz/8 = 6MHz
                             TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock                 

  TCC1->CTRLBSET.reg = TCC_CTRLBSET_ONESHOT;         // Enable one shot
  while (TCC1->SYNCBUSY.bit.CTRLB);
  
  TCC1->DRVCTRL.reg |= TCC_DRVCTRL_NRE2;             // Continue to drive the output on TCC1/WO[2] when timer has stopped 
                                                                                     //(rather than becoming tri-state) 
  TCC1->DRVCTRL.reg |= TCC_DRVCTRL_INVEN2;           // Invert the output to generate an active low pulse on TCC1/WO[2]
  
  TCC1->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;             // Set-up TCC1 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC1->SYNCBUSY.bit.WAVE);

  TCC1->PER.reg = 8999;                              // Set-up the PER (period) register for 1500us pulse period
  while (TCC1->SYNCBUSY.bit.PER);
  
  TCC1->CC[2].reg = 8999;                            // Set-up the CC (counter compare), channel 2 register for 1500us pulse width
  while (TCC1->SYNCBUSY.bit.CC2);

  TCC1->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC1
  while (TCC1->SYNCBUSY.bit.ENABLE);
}

void TCC4SetupTimer() {
  // Set up the generic clock (GCLK5) to clock timer TCC4 
  //MCLK->APBDMASK.reg |= MCLK_APBDMASK_TCC4;
  
  GCLK->GENCTRL[5].reg = GCLK_GENCTRL_DIV(3) |       // Divide the 48MHz clock source by divisor 3: 48MHz/3 = 16MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK5
                         GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
                         //GCLK_GENCTRL_SRC_DPLL1;     // Select 100MHz DPLL clock source
                         //GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL5); 

  GCLK->PCHCTRL[38].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC4 peripheral channel      PCHCTRL[38] == TCC4?
                          GCLK_PCHCTRL_GEN_GCLK5;    // Connect generic clock 5 to TCC4

  // Enable the peripheral multiplexer on pin D5
  PORT->Group[g_APinDescription[5].ulPort].PINCFG[g_APinDescription[5].ulPin].bit.PMUXEN = 1;
  
  // Set the D5 (PORT_PB14) peripheral multiplexer to peripheral (even port number) E(5): TCC4, Channel 0
  PORT->Group[g_APinDescription[5].ulPort].PMUX[g_APinDescription[5].ulPin >> 1].reg |= PORT_PMUX_PMUXE(5);

  TCC4->CTRLA.reg = TC_CTRLA_PRESCALER_DIV8 |        // Set prescaler to 8, 48MHz/8 = 6MHz,  N = 8
                    TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock

  TCC4->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;             // Set-up TCC4 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC4->SYNCBUSY.bit.WAVE);

  /* PWM Frequency = GCLK frequency / (N * (TOP + 1)
  // N = timer prescaler (1,2,4,8,16,64,256,1024)
  // TOP = the value of the PER register
  // PWM Resolution = log(CC0 + 1) / log(2)
  // PWM frequency = 16MHz / (8 * (4999 + 1)) = 100Hz
  // PWM resolution = log(4999+1)/log(2) = 12bits
  // The CC0 register determines the PWM duty cycle related to PER?  (CC0/PER == 5000/2500 == 50%)?
  // To change dutycycle on the fly use the CCBUF1 register (TC5->COUNT16.CCBUF[1].reg = ####; Set the duty cycle buffered
  */
/*TCC4 is only a 16-bit timer, 
    therefore the PER/CC registers must be a value between 0 and 65535. 
    Only TCC timers TCC0 and TCC1 are 24-bits. */

  TCC4->PER.reg = 4999;                            // Set-up the PER (period) register 100Hz PWM
  while (TCC4->SYNCBUSY.bit.PER); 
  
  TCC4->CC[0].reg = 2499;                           // Set-up the CC (counter compare), channel 0 register for 50% duty-cycle
  while (TCC4->SYNCBUSY.bit.CC0);            // Check which output channel is used for the pin D5 (PB14) E(5)

  TCC4->CTRLA.bit.ENABLE = 1;                       // Enable timer TCC1
  while (TCC4->SYNCBUSY.bit.ENABLE);
}
