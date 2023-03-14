#include <Adafruit_NeoPixel.h>
#include "sam.h"

#define rAudio      A2  //red   PB08
#define lAudio      A3  //white PB09

int valueL = 0, valueR = 0;
uint64_t pastMillis = 0;
Adafruit_NeoPixel strip(1, 8, NEO_GRB + NEO_KHZ800);

#define arrSize 1000
int arr[arrSize];

// working without diff or cont. read
void ADC0_Init() { // PB09, GCLK1 
	
	///ADC Clock Config
	MCLK->APBDMASK.bit.ADC0_ = 1;
	GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN; // enable gen clock 1 as source for ADC0
	
	// After configuring ADC Clock, reset ADC registers
	ADC0->CTRLA.bit.SWRST = 1;
	// Wait for ADC to finish reset, bit will stay high until reset clears the bit
	while(ADC0->CTRLA.bit.SWRST);

  // Set the sample time length to accommodate for input impedance. https://blog.thea.codes/getting-the-most-out-of-the-samd21-adc/ 
	// SampleRate = fCLK_ADC / ( nSAMPLING + nOFFCOMP + nDATA)
	//				        ^CLKfreq	 ^CLKcycles	 ^CLKcycles ^bitResolution

	//Divide GCLK clock by 4 for adc
	ADC0->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV4;
  // fCLK_ADC = fGCLK_ADC / 2^(1 + CTRLA.PRESCALER)
	
	//Select VDDANA (3.3V chip supply voltage as reference)
	//ADC0->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;
	ADC0->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC0; // 1/2 VDDANA
	
	//Select AIN3 (PB09) as positive input and gnd as negative input reference, non-diff mode by default
	ADC0->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_AIN3;// | ADC_INPUTCTRL_DIFFMODE;
	//ADC0->INPUTCTRL.bit.DIFFMODE = 1; // bit 7

	//Choose 12-bit resolution and Result Resolution for averaging
	ADC0->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT;
  // SampleRateAvgd = SampleRate/AVGCTRL.SAMPLENUM

	ADC0->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(1);	// Increase sampling clock cycles
	// nSAMPLING = ( SAMPLEN + 1 )
	
	//Accumulate 8 samples and average according to table 45-3 before conversion ready to read
	ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_4 | ADC_AVGCTRL_ADJRES(2);
  //ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_8 | ADC_AVGCTRL_ADJRES(3);
	
	//Enable ADC
	ADC0->CTRLA.bit.ENABLE = 1;
	
	//wait for ADC to be ready
	while(ADC0->SYNCBUSY.bit.ENABLE);
	
	// In PORT, the Peripheral Multiplexer Enable bit in the PINCFGy register (PINCFGy.PMUXEN)
	//  can be written to '1' to enable the connection between peripheral functions and individual I/O pins
	PORT->Group[1].PINCFG[9].bit.PMUXEN = 1; // Set PB09 as ADC
	PORT->Group[1].PMUX[4].bit.PMUXO = 1;	// PORTB = Group[1], PORTA = Group[0]
							// 	 ^	odd bit in mulitplexer PMUXO/PMUXE
}

void ADC1_Init() { // PB08, GCLK1 
	
	///ADC Clock Config
	MCLK->APBDMASK.bit.ADC1_ = 1;
	GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN; // enable gen clock 1 as source for ADC1
	
	// After configuring ADC Clock, reset ADC registers
	ADC1->CTRLA.bit.SWRST = 1;
	// Wait for ADC to finish reset, bit will stay high until reset clears the bit
	while(ADC1->CTRLA.bit.SWRST);

	//Divide GCLK clock by 4 for adc
	ADC1->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV4;
	
	//Select VDDANA (3.3V chip supply voltage as reference)
	//ADC1->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;
	ADC1->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC0; // 1/2 VDDANA
	
	//Select AIN0 (PB08) as positive input and gnd as negative input reference, non-diff mode by default
	ADC1->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_AIN0;
	//ADC1->INPUTCTRL.bit.DIFFMODE = 0; // bit 7
	
	//Choose 12-bit resolution
	ADC1->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT;
	//ADC1->CTRLB.reg = ADC_CTRLB_RESSEL_16BIT;
	//ADC1->CTRLB.bit.FREERUN = 1;	// bit 1 of CTRLB reg
	
	// Set the sample time length to accommodate for input impedance. https://blog.thea.codes/getting-the-most-out-of-the-samd21-adc/ 
	// SampleRate = fCLK_ADC / ( nSAMPLING + nOFFCOMP + nDATA)
	//				^CLKfreq	 ^CLKcycles	 ^CLKcycles ^bitResolution
	// fCLK_ADC = fGCLK_ADC / 2^(1 + CTRLA.PRESCALER)
	ADC1->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(1);	// Increase sampling clock cycles
	// SampleTime = ( SAMPLEN + 1 ) * ( CLK_ADC )
	
	//Accumulate 16 samples and average according to table 45-3 before conversion ready to read
	ADC1->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_4 | ADC_AVGCTRL_ADJRES(2);
	
	//Enable ADC
	ADC1->CTRLA.bit.ENABLE = 1;
	
	//wait for ADC to be ready
	while(ADC1->SYNCBUSY.bit.ENABLE);
	
	// In PORT, the Peripheral Multiplexer Enable bit in the PINCFGy register (PINCFGy.PMUXEN)
	//  can be written to '1' to enable the connection between peripheral functions and individual I/O pins
	PORT->Group[1].PINCFG[8].bit.PMUXEN = 1; // Set PB09 as ADC
	PORT->Group[1].PMUX[4].bit.PMUXO = 1;	// PORTB = Group[1], PORTA = Group[0]
							// 	 ^	odd bit in mulitplexer PMUXO/PMUXE

}

void setup() {

  Serial.begin(115200);
  while(!Serial);

  strip.begin();
  strip.show();
  strip.setBrightness(16);

	// You can use the xxxSET and xxxCLR registers to modify the correponding xxx register without read/write ops
	//   bit shift over by pin number to assign only that pin's configurations
	PORT->Group[1].DIRCLR.reg = (0<<9 | 0<<8); //If a bit in DIR is set to '1', the corresponding pin is configured as an output pin.
	//PORT->Group[1].CTRL.reg = 1<<9; //configure PORTB for "continuous sampling"?
	PORT->Group[1].OUTSET.reg = 0; //If bit y in OUT/OUTSET is written to '0', pin y is driven LOW

	PORT->Group[1].PINCFG[9].bit.INEN = 1;		// bit 1 in PINCFG reg
	PORT->Group[1].PINCFG[9].bit.PULLEN = 1;	// bit 2 in PINCFG reg

	PORT->Group[1].PINCFG[8].bit.INEN = 1;		// bit 1 in PINCFG reg
	PORT->Group[1].PINCFG[8].bit.PULLEN = 1;	// bit 2 in PINCFG reg

  strip.setPixelColor(0, 64, 0, 0);
  strip.show();
  Serial.println(1);
  delay(500);
	
  //MCLK->APBDMASK.reg = ( 1<<7 | 1<<8 );
	ADC0_Init();

  strip.setPixelColor(0, 0, 0, 64);
  strip.show();
  Serial.println(2);
  delay(500);

  ADC1_Init();

  strip.setPixelColor(0, 0, 64, 0);
  strip.show();
  Serial.println(3);
  delay(500);

// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

  while(valueR < arrSize) {
  ADC0->SWTRIG.bit.START = 1;
  //ADC1->SWTRIG.bit.START = 1;
  
  while(!ADC0->INTFLAG.bit.RESRDY);	// wait for averaged adc data to be ready
  arr[valueR] = ADC0->RESULT.reg;	//copy data from average result register
  valueR++;

  //while(!ADC1->INTFLAG.bit.RESRDY);	// wait for averaged adc data to be ready
  //valueL = ADC1->RESULT.reg;	//copy data from average result register
  }
  
  for (int i = 0; i < arrSize; i++) Serial.println(arr[i]);

// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
}

void loop() {}
