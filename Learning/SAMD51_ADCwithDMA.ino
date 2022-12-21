// Produces a sawtooth on A0 and samples A2 and A3 into buffers via DMA

#include <Adafruit_ZeroDMA.h>
#include <wiring_private.h>  // for access to pinPeripheral

// Create buffers for DMAing data around -- the .dmabuffers section is supposed to
// optimize the memory location for the DMA controller
__attribute__ ((section(".dmabuffers"), used)) static uint16_t dac_buffer[4096], adc_buffer[2][4096];

void setup() {
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  
  Serial.begin(115200);
  while(!Serial)

  // Populate output buffer
  for (auto i = 0; i < 4096; i++) dac_buffer[i] = i % 4096;

  clock_init();

  // Initialize peripherals
  adc_init(A2, ADC0);
  adc_init(A3, ADC1);
  dac_init();
  dma_init();
  timer_init();

  // Trigger both ADCs to enter free-run mode
  ADC0->SWTRIG.bit.START = 1;
  ADC1->SWTRIG.bit.START = 1;
}

int i = 0;
void loop() {
  // The M4's digitalWrite implementation is surprisingly fast -- this loop runs at 2MHz
  digitalWrite(10, HIGH);
  digitalWrite(10, LOW);

  //Serial.println(adc_buffer[0][i]);
  Serial.println(adc_buffer[1][i]);
  i++;
}

void dma_left_complete(Adafruit_ZeroDMA *dma) {
  // This is called twice every full block (every 2048 samples, ~2.1ms)
  digitalWrite(11, HIGH);
  digitalWrite(11, LOW);
}

void dma_right_complete(Adafruit_ZeroDMA *dma) {
  // This is called once every full block (every 4096 samples, ~4.2ms)
  digitalWrite(12, HIGH);
  digitalWrite(12, LOW);
}

void clock_init() {
  // Create a new generic 48MHz clock for our timer
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 48MHz clock source by divisor 1: 48MHz/1 = 48MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL7);

  // Setup TCC0's clock
  GCLK->PCHCTRL[TCC0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK7;
  
  // Setup ADC1's clock (ADC0 is pre-configured by Arduino)
  GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
}

void timer_init() {
  TCC0->CTRLA.reg = TC_CTRLA_PRESCALER_DIV4 |        // Set prescaler to 8, 48MHz/8 = 6MHz
                    TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock                 

  TCC0->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;             // Set-up TCC0 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC0->SYNCBUSY.bit.WAVE);

  TCC0->PER.reg = 12;                                // Set-up the PER (period) register 50Hz PWM
  while (TCC0->SYNCBUSY.bit.PER);
  
  TCC0->CC[0].reg = 6;                               // Set-up the CC (counter compare), channel 0 register for 50% duty-cycle
  while (TCC0->SYNCBUSY.bit.CC0);

  TCC0->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC0
  while (TCC0->SYNCBUSY.bit.ENABLE);
}

void dma_init() {
  static Adafruit_ZeroDMA left_in_dma, right_in_dma, out_dma;
  
  {
    left_in_dma.allocate();
  
    auto desc = left_in_dma.addDescriptor(
      (void *)(&ADC0->RESULT.reg),
      adc_buffer[0],
      2048,
      DMA_BEAT_SIZE_HWORD,
      false,
      true);
    desc->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;

    desc = left_in_dma.addDescriptor(
      (void *)(&ADC0->RESULT.reg),
      &adc_buffer[0][2048],
      2048,
      DMA_BEAT_SIZE_HWORD,
      false,
      true);
    desc->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;

    left_in_dma.loop(true);
    left_in_dma.setTrigger(0x44);
    left_in_dma.setAction(DMA_TRIGGER_ACTON_BEAT);
    left_in_dma.setCallback(dma_left_complete);
  
    left_in_dma.startJob();
  }

  {
    right_in_dma.allocate();
  
    auto desc = right_in_dma.addDescriptor(
      (void *)(&ADC1->RESULT.reg),
      adc_buffer[1],
      4096,
      DMA_BEAT_SIZE_HWORD,
      false,
      true);
    desc->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;

    right_in_dma.loop(true);
    right_in_dma.setTrigger(0x46);
    right_in_dma.setAction(DMA_TRIGGER_ACTON_BEAT);
    right_in_dma.setCallback(dma_right_complete);
  
    right_in_dma.startJob();
  }

  {
    out_dma.allocate();
  
    auto desc = out_dma.addDescriptor(
      dac_buffer,                                                                                                                                     
      (void *)(&DAC->DATA[0]),
      4096,
      DMA_BEAT_SIZE_HWORD,
      true,
      false);
    desc->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;

    out_dma.loop(true);
    out_dma.setTrigger(0x16);
    out_dma.setAction(DMA_TRIGGER_ACTON_BEAT);
    
    out_dma.startJob();
  }
}

void dac_init() {
  // Disable DAC
  DAC->CTRLA.bit.ENABLE = 0;
  while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);

  // Use an external reference voltage (see errata; the internal reference is busted)
  DAC->CTRLB.reg = DAC_CTRLB_REFSEL_VREFPB;
  while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);

  // Enable channel 0
  DAC->DACCTRL[0].bit.ENABLE = 1;
  while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);

  // Enable DAC
  DAC->CTRLA.bit.ENABLE = 1;
  while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
}

void adc_init(int inpselCFG, Adc *ADCx) {
  // Configure the ADC pin (cheating method)
  pinPeripheral(inpselCFG, PIO_ANALOG);
  
  ADCx->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)
  while( ADCx->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL );
  
  ADCx->INPUTCTRL.bit.MUXPOS = g_APinDescription[inpselCFG].ulADCChannelNumber; // Selection for the positive ADC input
  while( ADCx->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL );
  
  ADCx->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV4_Val;
  while( ADCx->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB );
  
  ADCx->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
  while( ADCx->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB );
  
  ADCx->SAMPCTRL.reg = 0x0;                        
  while( ADCx->SYNCBUSY.reg & ADC_SYNCBUSY_SAMPCTRL );
  
  ADCx->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // 1 sample only (no oversampling nor averaging)
            ADC_AVGCTRL_ADJRES(0x0ul);   // Adjusting result by 0
  while( ADCx->SYNCBUSY.reg & ADC_SYNCBUSY_AVGCTRL );
  
  ADCx->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val; // 1/2 VDDANA = 0.5* 3V3 = 1.65V
  while(ADCx->SYNCBUSY.reg & ADC_SYNCBUSY_REFCTRL);
  
  ADCx->CTRLB.reg |= 0x02;  ; //FREERUN
  while( ADCx->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB);
  
  ADCx->CTRLA.bit.ENABLE = 0x01;
  while( ADCx->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE );
}