#include <Adafruit_NeoPixel.h>
#include "sam.h"

int value = 0;
uint64_t pastMillis = 0;
Adafruit_NeoPixel strip(1, 8, NEO_GRB + NEO_KHZ800);

void ADC_Init() {
	
	///ADC Clock Config
	MCLK->APBDMASK.bit.ADC0_ = 1;
	GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN; // enable gen clock 1 as source for ADC0
	
	// After configuring ADC Clock, reset ADC registers
	ADC0->CTRLA.bit.SWRST = 1;
	// Wait for ADC to finish reset, bit will stay high until reset clears the bit
	while(ADC0->CTRLA.bit.SWRST);

	//Divide GCLK clock by 4 for adc
	ADC0->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV4;
	
	//Select VDDANA (3.3V chip supply voltage as reference)
	//ADC0->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;
	ADC0->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC0; // 1/2 VDDANA
	
	//Select AIN3(PB09) as positive input and gnd as negative input reference, non-diff mode by default
	ADC0->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_AIN3;
	//ADC0->INPUTCTRL.bit.DIFFMODE = 0; // bit 7
	
	//Choose 12-bit resolution
	ADC0->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT;
	//ADC0->CTRLB.reg = ADC_CTRLB_RESSEL_16BIT;
	//ADC0->CTRLB.bit.FREERUN = 1;	// bit 1 of CTRLB reg
	
	// Set the sample time length to accommodate for input impedance. https://blog.thea.codes/getting-the-most-out-of-the-samd21-adc/ 
	// SampleRate = fCLK_ADC / ( nSAMPLING + nOFFCOMP + nDATA)
	//				^CLKfreq	 ^CLKcycles	 ^CLKcycles ^bitResolution
	// fCLK_ADC = fGCLK_ADC / 2^(1 + CTRLA.PRESCALER)
	ADC0->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(1);	// Increase sampling clock cycles
	// SampleTime = ( SAMPLEN + 1 ) * ( CLK_ADC )
	
	//Accumulate 16 samples and average according to table 45-3 before conversion ready to read
	ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_16 | ADC_AVGCTRL_ADJRES(4);
	
	//Enable ADC
	ADC0->CTRLA.bit.ENABLE = 1;
	
	//wait for ADC to be ready
	while(ADC0->SYNCBUSY.bit.ENABLE);
	
	// In PORT, the Peripheral Multiplexer Enable bit in the PINCFGy register (PINCFGy.PMUXEN)
	//  can be written to '1' to enable the connection between peripheral functions and individual I/O pins
	PORT->Group[1].PINCFG[9].bit.PMUXEN = 1; // Set PB09 as ADC
	PORT->Group[1].PMUX[4].bit.PMUXO = 1;	// PORTB = Group[1], PORTA = Group[0]
							// 	 ^	odd bit in mulitplexer PMUXO/PMUXE
	
	//Set PA08 as ADC
	//PORT->Group[0].PINCFG[8].bit.PMUXEN = 1;
	//PORT->Group[0].PMUX[4].bit.PMUXE = 1;	
}

void setup() {

  Serial.begin(115200);
  while(!Serial);

  strip.begin();
  strip.show();
  strip.setBrightness(32);

  strip.setPixelColor(0, 64, 0, 0);
  strip.show();
  Serial.println(1);
  delay(500);

  strip.setPixelColor(0, 0, 0, 64);
  strip.show();
  Serial.println(10);
  delay(500);

  strip.setPixelColor(0, 0, 64, 0);
  strip.show();
  Serial.println(100);
  delay(500);

	// You can use the xxxSET and xxxCLR registers to modify the correponding xxx register without read/write ops
	//  bit shift over by pin number to assign only that pin's configurations
	PORT->Group[1].DIRCLR.reg = 0<<9; //If a bit in DIR is set to '1', the corresponding pin is configured as an output pin.
	PORT->Group[1].CTRL.reg = 1<<9; //configure PORTB for "continuous sampling"?
	PORT->Group[1].OUTSET.reg = 0; //If bit y in OUT/OUTSET is written to '0', pin y is driven LOW
	PORT->Group[1].PINCFG[9].bit.INEN = 1;		// bit 1 in PINCFG reg
	PORT->Group[1].PINCFG[9].bit.PULLEN = 1;	// bit 2 in PINCFG reg
	
	ADC_Init();
  Serial.println(1000);
}

void loop() {
  //if(millis() - pastMillis > 1) {
    //pastMillis = millis();
    ADC0->SWTRIG.bit.START = 1;
		while(!ADC0->INTFLAG.bit.RESRDY);	// wait for averaged adc data to be ready
		value = ADC0->RESULT.reg;	//copy data from average result register
    Serial.println(value);
  //}
}