// Use SAMD51's DMAC to read the ADC0 and ADC1 on A4 and A5 (Metro M4 or A2 and A3 on Feather/Itsy Bitsy M4) respectively

volatile boolean results0Ready = false;                      // Results ready flags
volatile boolean results1Ready = false;
uint16_t adcResults0[1024];                                  // ADC results array 0
uint16_t adcResults1[1024];                                  // ADC results array 1

typedef struct           // DMAC descriptor structure
{
  uint16_t btctrl;
  uint16_t btcnt;
  uint32_t srcaddr;
  uint32_t dstaddr;
  uint32_t descaddr;
} dmacdescriptor ;

volatile dmacdescriptor wrb[DMAC_CH_NUM] __attribute__ ((aligned (16)));          // Write-back DMAC descriptors
dmacdescriptor descriptor_section[DMAC_CH_NUM] __attribute__ ((aligned (16)));    // DMAC channel descriptors
dmacdescriptor descriptor __attribute__ ((aligned (16)));                         // Place holder descriptor

void setup() {
  Serial.begin(115200);                                                       // Start the native USB port
  while(!Serial);                                                             // Wait for the console to open
  
  DMAC->BASEADDR.reg = (uint32_t)descriptor_section;                          // Specify the location of the descriptors
  DMAC->WRBADDR.reg = (uint32_t)wrb;                                          // Specify the location of the write back descriptors
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);                // Enable the DMAC peripheral
   
  DMAC->Channel[0].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(ADC0_DMAC_ID_RESRDY) |  // Set DMAC to trigger when ADC0 result is ready
                                 DMAC_CHCTRLA_TRIGACT_BURST;                  // DMAC burst transfer
  descriptor.descaddr = (uint32_t)0;                                           // Run descriptor only once
  descriptor.srcaddr = (uint32_t)&ADC0->RESULT.reg;                           // Take the result from the ADC0 RESULT register
  descriptor.dstaddr = (uint32_t)adcResults0 + sizeof(uint16_t) * 1024;       // Place it in the adcResults0 array
  descriptor.btcnt = 1024;                                                     // Beat count
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
  descriptor.descaddr = (uint32_t)0;                                          // Set up a circular descriptor
  descriptor.srcaddr = (uint32_t)&ADC1->RESULT.reg;                           // Take the result from the ADC1 RESULT register
  descriptor.dstaddr = (uint32_t)adcResults1 + sizeof(uint16_t) * 1024;       // Place it in the adcResults1 array
  descriptor.btcnt = 1024;                                                    // Beat count
  descriptor.btctrl = DMAC_BTCTRL_BEATSIZE_HWORD |                            // Beat size is HWORD (16-bits)
                      DMAC_BTCTRL_DSTINC |                                    // Increment the destination address
                      DMAC_BTCTRL_VALID;                                      // Descriptor is valid                  
  memcpy(&descriptor_section[1], &descriptor, sizeof(descriptor));            // Copy the descriptor to the descriptor section
  
  NVIC_SetPriority(DMAC_1_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for DMAC Channel 1
  NVIC_EnableIRQ(DMAC_1_IRQn);         // Connect DMAC Channel 1 to Nested Vector Interrupt Controller (NVIC)
  
  DMAC->Channel[1].CHINTENSET.reg = DMAC_CHINTENSET_TCMPL;                    // Activate the transfer complete (TCMPL) interrupt on DMAC channel 1
  DMAC->Channel[1].CHPRILVL.reg = DMAC_CHPRILVL_PRILVL_LVL0;
  
  ADC1->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_AIN1_Val;   // Set the analog input to A3
  while(ADC1->SYNCBUSY.bit.INPUTCTRL);                // Wait for synchronization
  ADC1->SAMPCTRL.bit.SAMPLEN = 0x02;                  // Extend sampling time by SAMPCTRL ADC cycles (12 + 1 + 2)/3MHz = 5.0us sample time = 200kHz 
  while(ADC1->SYNCBUSY.bit.SAMPCTRL);                 // Wait for synchronization
  ADC1->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT |          // Set ADC resolution to 12 bits 
                    ADC_CTRLB_FREERUN;                // Set ADC to free run mode  
  while(ADC1->SYNCBUSY.bit.CTRLB);                    // Wait for synchronization
  ADC1->CTRLA.bit.SLAVEEN = 1;                        // Set ADC1 to slave, ADC0 to master, both share CTRLA register
  
  ADC0->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_AIN2_Val;     // Set the analog input to A2
  while(ADC0->SYNCBUSY.bit.INPUTCTRL);                // Wait for synchronization
  ADC0->SAMPCTRL.bit.SAMPLEN = 0x02;                  // Extend sampling time by SAMPCTRL ADC cycles (12 + 1 + 2)/3MHz = 5.0us sample time = 200kHz
  while(ADC0->SYNCBUSY.bit.SAMPCTRL);                 // Wait for synchronization  
  ADC0->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT |          // Set ADC resolution to 12 bits 
                    ADC_CTRLB_FREERUN;                // Set ADC to free run mode        
  while(ADC0->SYNCBUSY.bit.CTRLB);                    // Wait for synchronization
  ADC0->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV16;        // Divide Clock ADC GCLK by 16 (48MHz/16 = 3MHz) (12 + 1 + 2)/3MHz = 5.0us sample time 
  ADC0->CTRLA.bit.ENABLE = 1;                         // Enable the ADC
  while(ADC0->SYNCBUSY.bit.ENABLE);                   // Wait for synchronization
  ADC0->SWTRIG.bit.START = 1;                         // Initiate a software trigger to start an ADC conversion
  while(ADC0->SYNCBUSY.bit.SWTRIG);                   // Wait for synchronization
  DMAC->Channel[0].CHCTRLA.bit.ENABLE = 1;            // Enable DMAC channel 0
  DMAC->Channel[1].CHCTRLA.bit.ENABLE = 1;            // Enable DMAC channel 1
}

void loop() 
{  
  while (!(results0Ready && results1Ready));          // Wait for both sets of results to be ready
  
  for (uint32_t i = 0; i < 1024; i++)
  {
    Serial.print(adcResults0[i]);                     // Output the results to the Serial plotter
    Serial.print(F(","));
    Serial.println(adcResults1[i]);
  }
  results0Ready = false;                              // Clear the results0 ready flag
  results1Ready = false;                              // Clear the results1 ready flag
    
  delay(1000);                                        // Wait for 1 second
  DMAC->Channel[0].CHCTRLA.bit.ENABLE = 1;            // Enable DMAC channel 0
  DMAC->Channel[1].CHCTRLA.bit.ENABLE = 1;            // Enable DMAC channel 1
}

void DMAC_0_Handler()                                 // Interrupt handler for DMAC channel 0
{
  if (DMAC->Channel[0].CHINTFLAG.bit.TCMPL)           // Check if DMAC channel 0 transfer is complete
  {  
    DMAC->Channel[0].CHINTFLAG.bit.TCMPL = 1;         // Clear the transfer complete (TCMPL) interrupt flag
    results0Ready = true;                             // Set the results 0 ready flag
  }
}

void DMAC_1_Handler()                                 // Interrupt handler for DMAC channel 1
{
  if (DMAC->Channel[1].CHINTFLAG.bit.TCMPL)           // Check if DMAC channel 1 has been suspended (SUSP) 
  {    
    DMAC->Channel[1].CHINTFLAG.bit.TCMPL = 1;         // Clear the transfer complete (TCMPL) interrupt flag
    results1Ready = true;                             // Set the results 1 ready flag  
  }
}
