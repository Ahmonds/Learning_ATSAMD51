
void setup()
{
  MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC0;           // Activate timer TC0

  // Set up the generic clock (GCLK7) used to clock timers
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(3) |       // Divide the 48MHz clock source by divisor 3: 48MHz/3 = 16MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         GCLK_GENCTRL_SRC_DFLL;      // Generate from 48MHz DFLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL7);               // Wait for synchronization 

  GCLK->PCHCTRL[9].reg = GCLK_PCHCTRL_CHEN |         // Enable perhipheral channel
                         GCLK_PCHCTRL_GEN_GCLK7;     // Connect generic clock 7 to TC0

  // Enable the peripheral multiplexer on pin A1
  PORT->Group[g_APinDescription[A1].ulPort].PINCFG[g_APinDescription[A1].ulPin].bit.PMUXEN = 1;

  // Set A1 the peripheral multiplexer to peripheral E(4): TC0, Channel 1
  PORT->Group[g_APinDescription[A1].ulPort].PMUX[g_APinDescription[A1].ulPin >> 1].reg |= PORT_PMUX_PMUXO(4);

  TC0->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV16 |        // Set prescaler to 16, 16MHz/16 = 1MHz
                           TC_CTRLA_PRESCSYNC_PRESC |        // Set the reset/reload to trigger on prescaler clock
                           TC_CTRLA_MODE_COUNT16;            // Set the counter to 16-bit mode

  TC0->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MPWM;      // Set-up TC0 timer for Match PWM mode (MPWM)

  TC0->COUNT16.CC[0].reg = 1999;                    // Use CC0 register as TOP value, set for 50Hz PWM
  while (TC0->COUNT16.SYNCBUSY.bit.CC0);             // Wait for synchronization

  TC0->COUNT16.CC[1].reg = 999;                     // Set the duty cycle to 50% (CC1 half of CC0)
  while (TC0->COUNT16.SYNCBUSY.bit.CC1);             // Wait for synchronization

  TC0->COUNT16.CTRLA.bit.ENABLE = 1;                 // Enable timer TC0
  while (TC0->COUNT16.SYNCBUSY.bit.ENABLE);          // Wait for synchronization
}

void loop() {
  
  TCC0->CCBUF[0].reg = 29999;    // Set-up the CCBUF (Counter Compare Buffered), 
                                 // channel 0 register for 25% duty-cycle          
  delay(1000);
  TCC0->CCBUF[0].reg = 89999;    // Set-up the CCBUF (Counter Compare Buffered), 
                                 // channel 0 register for 75% duty-cycle
  delay(1000);
}
