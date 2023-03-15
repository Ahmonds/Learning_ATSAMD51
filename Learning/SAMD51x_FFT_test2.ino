// https://www.utasker.com/docs/uTasker/uTasker_DSP.pdf
// https://www.keil.com/pack/doc/CMSIS/DSP/html/group__RealFFT.html
// https://electronics.stackexchange.com/questions/497595/input-and-output-array-for-cmsis-dsp-real-fft-q15-functions

#include <Adafruit_NeoPixel.h>
#include <algorithm>
#include <iterator>
#include "sam.h"
#include <arm_math.h>
#include "arm_const_structs.h"

#undef printf
#define dataSize 512 // DMAC & FFT data size
#define DMAtype int16_t
arm_rfft_instance_q15 fft;
//arm_rfft_fast_instance_f32 f32_fft;
Adafruit_NeoPixel strip(1, 8, NEO_GRB + NEO_KHZ800);

volatile boolean results0Ready = false;               // Results ready flags
volatile boolean results1Ready = false;
DMAtype adcResultsR[dataSize], adcResultsL[dataSize]; // ADC result arrays
int16_t RFFTin[dataSize],  LFFTin[dataSize];            // FFT input arrays (CMSIS FFT alters input array)
int16_t RFFT[dataSize*2],  LFFT[dataSize*2];            // FFT result arrays

typedef struct {         // DMAC descriptor structure 
  uint16_t btctrl;
  uint16_t btcnt;
  uint32_t srcaddr;
  uint32_t dstaddr;
  uint32_t descaddr;
} dmacdescriptor;

volatile dmacdescriptor wrb[DMAC_CH_NUM] __attribute__ ((aligned (16)));          // Write-back DMAC descriptors
dmacdescriptor descriptor_section[DMAC_CH_NUM] __attribute__ ((aligned (16)));    // DMAC channel descriptors
dmacdescriptor descriptor __attribute__ ((aligned (16)));                         // Place holder descriptor

void setup() {
  strip.begin();
  strip.setBrightness(16);
  strip.show();
  
  Serial.begin(115200);
  while(!Serial);
  strip.setPixelColor(0, 32, 32, 32); strip.show(); delay(500);  // white

  arm_rfft_init_q15(&fft, dataSize, 0, 0);
  //arm_rfft_fast_init_f32(&f32_fft, dataSize);
  // 100kHz/512 = 195.3Hz bin size
  //  50kHz/512 = 97.66Hz bin size  ->   20us*512 = 10.24ms
  
  for (uint16_t i = 0; i < dataSize; i++) { adcResultsR[i] = 100*sin(i*PI/100); }
  strip.setPixelColor(0, 0, 0, 32); strip.show(); delay(500);  // blue

  std::copy(adcResultsR, adcResultsR+dataSize, RFFTin);
  //for (uint16_t i = 0; i < dataSize; i++) { Serial.printf("%d, ", RFFTin[i]); } Serial.println();
  strip.setPixelColor(0, 32, 0, 0); strip.show(); delay(500);  // red

  arm_rfft_q15(&fft, RFFTin, RFFT);                     // output of 512 size rfft_q15 are q9.7 types. Shift result right by 8 (divide 256)
  //arm_cmplx_mag_q15(RFFT, RFFTMag, dataSize<<1);
  //for (uint16_t i = 0; i < dataSize>>1; i++) { Serial.printf("%5d Hz : %d : %d\n", uint16_t(i*97.656), RFFT[i<<1], RFFT[(i<<1)-1]); }
  for (uint16_t i = 0; i < dataSize>>1; i++) { Serial.printf("%5d Hz : %d\n", uint16_t(i*97.656), RFFT[i<<1]*RFFT[i<<1]); } //+ RFFT[i-1]*RFFT[i-1]
  strip.setPixelColor(0, 0, 32, 0); strip.show(); delay(500);  // green

  while(true);
  //setupDMAC();
  //setupADCs();

  ADC0->SWTRIG.bit.START = 1;                 // Software trigger to start first ADC conversion
  while(ADC0->SYNCBUSY.bit.SWTRIG);
  DMAC->Channel[0].CHCTRLA.bit.ENABLE = 1;    // Enable DMAC channel 0
  DMAC->Channel[1].CHCTRLA.bit.ENABLE = 1;
  
}// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

void loop()  {
  while (!(results0Ready && results1Ready));
  
  /*for (uint16_t i = 0; i < dataSize; i++) {
    Serial.print(adcResultsR[i]);                     // Output ADC results to Serial plotter
    Serial.print(F(","));
    Serial.println(adcResultsL[i]);
  }*/

  std::copy(adcResultsR, adcResultsR+dataSize, RFFTin);
  arm_rfft_q15(&fft, RFFTin, RFFT);                     // output of 512 size rfft_q15 are q9.7 types. Shift result right by 8 (divide 256)
  //arm_cmplx_mag_q15(RFFT, RFFTMag, dataSize); 
  for (uint16_t i = 0; i < dataSize<<1; i+=2) { Serial.printf("%3.0d: %d\n", (i*50000/dataSize), ( RFFT[i]*RFFT[i] + RFFT[--i]*RFFT[--i] )); }


  while(true);
  results0Ready = false;                                // Clear the result ready flags
  results1Ready = false;
  DMAC->Channel[0].CHCTRLA.bit.ENABLE = 1;              // Enable DMAC channels
  DMAC->Channel[1].CHCTRLA.bit.ENABLE = 1;
}// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

void setupDMAC() { 

  DMAC->BASEADDR.reg = (uint32_t) descriptor_section;                         // Specify the location of the descriptors
  DMAC->WRBADDR.reg = (uint32_t) wrb;                                         // Specify the location of the write back descriptors
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);                // Enable the DMAC peripheral
   
  DMAC->Channel[0].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(ADC0_DMAC_ID_RESRDY) |  // Set DMAC to trigger when ADC0 result is ready
                                 DMAC_CHCTRLA_TRIGACT_BURST;                  // DMAC burst transfer
  descriptor.descaddr = (uint32_t) 0;                                         // Run descriptor only once
  descriptor.srcaddr = (uint32_t) &ADC0->RESULT.reg;                          // Take the result from the ADC0 RESULT register
  descriptor.dstaddr = (uint32_t) adcResultsR + sizeof(DMAtype) * dataSize;   // Place it in the adcResults0 array
  descriptor.btcnt = dataSize;                                                // Beat count
  descriptor.btctrl = DMAC_BTCTRL_BEATSIZE_HWORD |                            // Beat size is HWORD (16-bits)
                      DMAC_BTCTRL_DSTINC |                                    // Increment the destination address
                      DMAC_BTCTRL_VALID;                                      // Descriptor is valid                    
  memcpy(&descriptor_section[0], &descriptor, sizeof(descriptor));            // Copy the descriptor to the descriptor section

  NVIC_SetPriority(DMAC_0_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for DMAC Channel 0
  NVIC_EnableIRQ(DMAC_0_IRQn);         // Connect DMAC Channel 0 to Nested Vector Interrupt Controller (NVIC)
  
  DMAC->Channel[0].CHINTENSET.reg = DMAC_CHINTENSET_TCMPL;                    // Activate the transfer complete (TCMPL) interrupt on DMAC channel 0
  DMAC->Channel[1].CHPRILVL.reg = DMAC_CHPRILVL_PRILVL_LVL0;
  
  DMAC->Channel[1].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(ADC1_DMAC_ID_RESRDY) |  // Set DMAC to trigger when ADC1 result is ready
                                 DMAC_CHCTRLA_TRIGACT_BURST;                  // DMAC burst transfer
  descriptor.descaddr = (uint32_t) 0;                                         // Set up a circular descriptor
  descriptor.srcaddr = (uint32_t) &ADC1->RESULT.reg;                          // Take the result from the ADC1 RESULT register
  descriptor.dstaddr = (uint32_t) adcResultsL + sizeof(DMAtype) * dataSize;   // Place it in the adcResults1 array
  descriptor.btcnt = dataSize;                                                // Beat count
  descriptor.btctrl = DMAC_BTCTRL_BEATSIZE_HWORD |                            // Beat size is HWORD (16-bits)
                      DMAC_BTCTRL_DSTINC |                                    // Increment the destination address
                      DMAC_BTCTRL_VALID;                                      // Descriptor is valid                  
  memcpy(&descriptor_section[1], &descriptor, sizeof(descriptor));            // Copy the descriptor to the descriptor section
  
  NVIC_SetPriority(DMAC_1_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for DMAC Channel 1
  NVIC_EnableIRQ(DMAC_1_IRQn);         // Connect DMAC Channel 1 to Nested Vector Interrupt Controller (NVIC)
  
  DMAC->Channel[1].CHINTENSET.reg = DMAC_CHINTENSET_TCMPL;                    // Activate the transfer complete (TCMPL) interrupt on DMAC channel 1
  DMAC->Channel[1].CHPRILVL.reg = DMAC_CHPRILVL_PRILVL_LVL0;
}// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

void setupADCs() {// A3 left (ADC 1, slave) --- A2 right (ADC0, master) 
  ADC1->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_AIN1 | ADC_INPUTCTRL_DIFFMODE; // Set the analog input to A3 (left audio)
  while(ADC1->SYNCBUSY.bit.INPUTCTRL);                // Wait for synchronization
  
  ADC1->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC0;     // 1/2 VDDANA ref
  ADC1->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_4 | ADC_AVGCTRL_ADJRES(2);    // Average 4 readings
  //ADC1->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_8 | ADC_AVGCTRL_ADJRES(3);

  ADC1->SAMPCTRL.bit.SAMPLEN = 0x02;                  // Extend sampling time by SAMPCTRL cycles
  while(ADC1->SYNCBUSY.bit.SAMPCTRL);                 // Wait for synchronization

  ADC1->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT |          // Set ADC resolution to 12 bits 
                    ADC_CTRLB_FREERUN;                // Set ADC to free run mode  
  while(ADC1->SYNCBUSY.bit.CTRLB);
  
  ADC1->CTRLA.bit.SLAVEEN = 1;                        // Set ADC1 to slave, ADC0 to master, both share CTRLA register
  
  // (12(bits) 1(sample) + 2(sampleCtrl)) * 4(avg) = 60 ticks
  // ((12+1+2)*4)/((48*10^6)/8) = 10.0us

  ADC0->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_AIN2 | ADC_INPUTCTRL_DIFFMODE; // Set the analog input to A2 (right audio)
  while(ADC0->SYNCBUSY.bit.INPUTCTRL);                // Wait for synchronization

  ADC0->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC0;     // 1/2 VDDANA ref
  ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_4 | ADC_AVGCTRL_ADJRES(2);
  //ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_8 | ADC_AVGCTRL_ADJRES(3);

  ADC0->SAMPCTRL.bit.SAMPLEN = 0x02;
  while(ADC0->SYNCBUSY.bit.SAMPCTRL);

  ADC0->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT |          // Set ADC resolution to 12 bits 
                    ADC_CTRLB_FREERUN;                // Set ADC to free run mode        
  while(ADC0->SYNCBUSY.bit.CTRLB);

  ADC0->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV16;        // Generated clock divider
  ADC0->CTRLA.bit.ENABLE = 1;                         // Enable the ADC
  while(ADC0->SYNCBUSY.bit.ENABLE);

  //ADC0->SWTRIG.bit.START = 1;                       // Initiate a software trigger to start an ADC conversion
  //while(ADC0->SYNCBUSY.bit.SWTRIG);
}// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

void DMAC_0_Handler() {// Interrupt handler for DMAC channel 0 (right audio) 

  if (DMAC->Channel[0].CHINTFLAG.bit.TCMPL) {         // Check if DMAC channel 0 transfer is complete
    DMAC->Channel[0].CHINTFLAG.bit.TCMPL = 1;         // Clear the transfer complete (TCMPL) interrupt flag
    results0Ready = true;                             // Set the results 0 ready flag
  }
}// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

void DMAC_1_Handler() {// Interrupt handler for DMAC channel 1 (left audio) 

  if (DMAC->Channel[1].CHINTFLAG.bit.TCMPL) {         // Check if DMAC channel 1 has been suspended (SUSP) 
    DMAC->Channel[1].CHINTFLAG.bit.TCMPL = 1;         // Clear the transfer complete (TCMPL) interrupt flag
    results1Ready = true;                             // Set the results 1 ready flag  
  }
}// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =