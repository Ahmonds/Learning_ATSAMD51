/*
 * This sketch illustrates how to set a timer on an SAMD21 based board in Arduino (Feather M0, Arduino Zero should work)
 * It should generate a 1Hz square wave as it is (thanks richdrich for the suggestion)
 * Some more info about Timer Counter works can be found in this article: 
 * http://www.lucadavidian.com/2017/08/08/arduino-m0-pro-il-sistema-di-clock/
 * and in the datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/SAM_D21_DA1_Family_DataSheet_DS40001882F.pdf
 * 
 * PWM frequency = GCLK frequency / (Prescaler * (PER + 1))
 * PWM resolution = log (PER + 1) / log(2)
*/

uint32_t sampleRate = 1000; //sample rate in milliseconds, determines how often TC5_Handler is called

#define LED_PIN 13 //just for an example
bool state = 0; //just for an example


void setup() {
  pinMode(LED_PIN,OUTPUT); //this configures the LED pin, you can remove this it's just example code
  tcConfigure(sampleRate); //configure the timer to run at <sampleRate>Hertz
  tcStartCounter(); //starts the timer
}

void loop() {
  //tcDisable(); //This function can be used anywhere if you need to stop/pause the timer
  //tcReset(); //This function should be called everytime you stop the timer
}

//this function gets called by the interrupt at <sampleRate>Hertz
void TC5_Handler (void) {
  //YOUR CODE HERE 
  if(state == true) {
    digitalWrite(LED_PIN,HIGH);
  } else {
    digitalWrite(LED_PIN,LOW);
  }
  state = !state;
  // END OF YOUR CODE
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //Writing a 1 to INTFLAG.bit.MC0 clears the interrupt so that it will run again
}

/* 
 *  TIMER SPECIFIC FUNCTIONS FOLLOW
 *  you shouldn't change these unless you know what you're doing
 */

//Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once
//each time the audio sample frequency period expires.
 void tcConfigure(int sampleRate) {
  
 // select the generic clock generator used as source to the generic clock multiplexer
 GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
 while (GCLK->STATUS.bit.SYNCBUSY);

 tcReset(); //reset TC5

 // Set Timer counter 5 Mode to 16 bits, it will become a 16bit counter ('mode1' in the datasheet)
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
 
 // Set TC5 waveform generation mode to 'match frequency'
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
 
 //set prescaler
 //the clock normally counts at the GCLK_TC frequency, but we can set it to divide that frequency to slow it down
 //you can use different prescaler divisons here like TC_CTRLA_PRESCALER_DIV1 to get a different range
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE; //it will divide GCLK_TC frequency by 1024
 
 //set the compare-capture register. 
 //The counter will count up to this value (it's a 16bit counter so we use uint16_t)
 //this is how we fine-tune the frequency, make it count to a lower or higher value
 //system clock should be 1MHz (8MHz/8) at Reset by default
 TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate);
 while (tcIsSyncing());
 
 // Configure interrupt request
 NVIC_DisableIRQ(TC5_IRQn);
 NVIC_ClearPendingIRQ(TC5_IRQn);
 NVIC_SetPriority(TC5_IRQn, 0);
 NVIC_EnableIRQ(TC5_IRQn);

 // Enable the TC5 interrupt request
 TC5->COUNT16.INTENSET.bit.MC0 = 1;
 while (tcIsSyncing()); //wait until TC5 is done syncing 
} 

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing() {
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter() {
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5 
void tcReset() {
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable() {
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}
