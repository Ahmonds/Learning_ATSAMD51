// https://www.utasker.com/docs/uTasker/uTasker_DSP.pdf  DSP walkthrough
// https://arm-software.github.io/CMSIS-DSP/v1.12.0/group__RealFFT.html  CMSIS Library Website

// https://community.arm.com/support-forums/f/architectures-and-processors-forum/52016/how-do-we-correctly-use-the-cmsis-dsp-functions-that-have-fixed-point-qx-input-outputs 
// https://community.arm.com/support-forums/f/architectures-and-processors-forum/8520/12-bit-from-adc-to-q15-for-a-fft

#include <Adafruit_NeoPixel.h>
#include <algorithm>
#include <iterator>
#include "sam.h"
#include <arm_math.h>
#include "arm_const_structs.h"

#define breakPoint while(flip) Debounce(); flip = true;

#define dataSize 512 // DMAC & FFT data size    1/51=0.001953125
#define FFTtype float // float = 32bits (single precision)
#define DMAtype int16_t 
arm_rfft_fast_instance_f32 F32FFT;
Adafruit_NeoPixel strip(1, 8, NEO_GRB + NEO_KHZ800);

#define Button 5
#define dbDelay 50
volatile bool ButtonState = false, flip = true;
volatile uint32_t PrevBounceMillis = 0;

volatile boolean results0Ready = false;               // Results ready flags
volatile boolean results1Ready = false;
DMAtype adcResultsR[dataSize], adcResultsL[dataSize]; // ADC result arrays
FFTtype RFFTin[dataSize], LFFTin[dataSize];           // FFT input arrays (CMSIS FFT alters input array)
FFTtype RFFT[dataSize], LFFT[dataSize];               // FFT result arrays

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
  enableFPU();

  pinMode(Button, INPUT_PULLUP);
  strip.begin();
  strip.setBrightness(16);
  strip.clear();
  strip.show();

  Serial.begin(115200); while(!Serial);

  for (uint16_t i = 0; i < dataSize; i++) { adcResultsR[i] = (DMAtype) 10*sin(i*2*PI/50); }  // * * * Generate 1kHz data for testing * * *
  //while(true);//  * * * * * * * * * * * * * * * * * BLOCK CODE * * * * * * * * * * * * * * * * *

  strip.setPixelColor(0, 16, 16, 16);
  strip.show();
  /*setupDMAC();
  setupADCs();

  ADC0->SWTRIG.bit.START = 1;                 // Software trigger to start first ADC conversion
  while(ADC0->SYNCBUSY.bit.SWTRIG);
  DMAC->Channel[0].CHCTRLA.bit.ENABLE = 1;    // Enable DMAC channel 0
  DMAC->Channel[1].CHCTRLA.bit.ENABLE = 1;*/
  PrevBounceMillis = millis();
}// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

void loop()  {
  strip.setPixelColor(0, 16, 0, 0);
  strip.show();
  //while (!(results0Ready && results1Ready));
  
  /*for (uint16_t i = 0; i < dataSize; i++) { // Output ADC results to Serial plotter 
    Serial.print(RFFTin[i]);
    Serial.print(F(","));
    Serial.println(LFFTin[i]);
  } while(flip) Debounce(); flip = true;*/
  
  arm_rfft_fast_init_f32(&F32FFT, dataSize); // https://stackoverflow.com/questions/5456801/c-int-float-casting 
  for(uint16_t i = 0; i < dataSize; i++) RFFTin[i] = adcResultsR[i]*0.0001f;
  //std::copy(adcResultsR, adcResultsR+dataSize, RFFTin); // Copy DMAC results into FFT input array (CMSIS alters input array)

  for (uint16_t i; i < dataSize; i++) Serial.println(adcResultsR[i]);
  strip.setPixelColor(0, 0, 0, 16);
  strip.show();
  breakPoint

  arm_rfft_fast_f32(&F32FFT, RFFTin, RFFT, 0);          // CMSIS f32 (float) Real FFT
  for (uint16_t i = 0; i < dataSize>>1; i++) { 
    Serial.printf("%3d : %5d Hz : ", i, uint16_t(i*50000/dataSize));
    Serial.println( RFFT[i<<1]*RFFT[i<<1] );
    //Serial.println( RFFT[i<<1] );
  }

  strip.setPixelColor(0, 0, 16, 0);
  strip.show();
  breakPoint

  results0Ready = false;                                // Clear the result ready flags
  results1Ready = false;
  DMAC->Channel[0].CHCTRLA.bit.ENABLE = 1;              // Enable DMAC channels
  DMAC->Channel[1].CHCTRLA.bit.ENABLE = 1;
}// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

void enableFPU() {
    __asm volatile (
    "LDR.W R0, =0xE000ED88 \n"
    "LDR R1, [R0] \n"
    "ORR R1, R1, #(0xF << 20) \n"
    "STR R1, [R0] \n"
    "DSB \n"
    "ISB \n"
   );
}// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

void setupDMAC() { 

  DMAC->BASEADDR.reg = (uint32_t) descriptor_section;                         // Specify the location of the descriptors
  DMAC->WRBADDR.reg = (uint32_t) wrb;                                         // Specify the location of the write back descriptors
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);                // Enable the DMAC peripheral
   
  DMAC->Channel[0].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(ADC0_DMAC_ID_RESRDY) |  // Set DMAC to trigger when ADC0 result is ready
                                 DMAC_CHCTRLA_TRIGACT_BURST;                  // DMAC burst transfer
  descriptor.descaddr = (uint32_t) 0;                                         // Run descriptor only once
  descriptor.srcaddr = (uint32_t) &ADC0->RESULT.reg;                             // Take the result from the ADC0 RESULT register
  descriptor.dstaddr = (uint32_t) adcResultsR + sizeof(DMAtype) * dataSize;   // Place it in the adcResults0 array
  descriptor.btcnt = dataSize;                                                // Beat count
  descriptor.btctrl = DMAC_BTCTRL_BEATSIZE_HWORD |                             // Beat size is WORD (32-bits)
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
  descriptor.srcaddr = (uint32_t) &ADC1->RESULT.reg;                             // Take the result from the ADC1 RESULT register
  descriptor.dstaddr = (uint32_t) adcResultsL + sizeof(DMAtype) * dataSize;   // Place it in the adcResults1 array
  descriptor.btcnt = dataSize;                                                // Beat count
  descriptor.btctrl = DMAC_BTCTRL_BEATSIZE_HWORD |                             // Beat size is HWORD (16-bits)
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
  //ADC1->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 | ADC_AVGCTRL_ADJRES(0);
  ADC1->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_4 | ADC_AVGCTRL_ADJRES(2);    // Average 4 readings
  //ADC1->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_8 | ADC_AVGCTRL_ADJRES(3);

  ADC1->SAMPCTRL.bit.SAMPLEN = 0x02;                  // Extend sampling time by SAMPCTRL cycles
  while(ADC1->SYNCBUSY.bit.SAMPCTRL);                 // Wait for synchronization

  ADC1->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT |          // Set ADC resolution to 12 bits 
                    ADC_CTRLB_FREERUN;                // Set ADC to free run mode  
  while(ADC1->SYNCBUSY.bit.CTRLB);
  
  ADC1->CTRLA.bit.SLAVEEN = 1;                        // Set ADC1 to slave, ADC0 to master, both share CTRLA register
  
  // (12(bits) 1(sample) + 2(sampleCtrl)) * 4(avg) = 60 ticks
  // 1/ ((12+1+2)*4)/((48*10^6)/16) = 50kHz (20us/sample)

  ADC0->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_AIN2 | ADC_INPUTCTRL_DIFFMODE; // Set the analog input to A2 (right audio)
  while(ADC0->SYNCBUSY.bit.INPUTCTRL);                // Wait for synchronization

  ADC0->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC0;     // 1/2 VDDANA ref
  //ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 | ADC_AVGCTRL_ADJRES(0);
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

void Debounce() { // Activate on button press
  if (!ButtonState) {
    if (digitalRead(Button) == ButtonState) PrevBounceMillis = millis();
    if (millis() - PrevBounceMillis > dbDelay) ButtonState = true;
  }

  if (ButtonState) {
    if (digitalRead(Button) == ButtonState) PrevBounceMillis = millis();
    if (millis() - PrevBounceMillis > dbDelay) {
      ButtonState = false;
      flip = false;
    }
  }
}