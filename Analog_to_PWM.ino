#include "wiring_private.h"  //some automation of which TC to use for which pins

const int ADCpin = 2;  //Pin for ADC, 0 cant be used in IDACRed mode as it has the DAC tied to it. AO can be used for ref GND for sound card with sound pin to ADCpin. On zero, 2 stands for A1
int pin = 13;          //PWM pin, 13 is usually connected to LED so you can see it working.

const float Frequency = 20.0f;  //PWM frequency, higher the frequency lower resolution. Wont work below 0.00281 hz wont work
const int ADCDiv = 5;               //Set up dividor of time for ADC, with high PWM frequencies high values may let the Duty period only to change every few cycles, while too low values may lead to less stable output. Up to 255.
int GAIN = 1;                       //1, 2, 4, 8, 16, 32, multiplier of input, only goes up to 16 in ICADRef mode
const int res = 12;                 //Set up Resolution of the ADC, 8 or 10 or 12 bits

/*  Calibration  */
int16_t minv = 0;     //Minimum meassured value of the input signal, make sure the value doesnt go below
int16_t maxv = 4095;  //Maximum meassured value og the input signal

/*  DAC AREF */
const bool IDACRef = 1;     //Use DAC as a reference instead of GND
const int BaseV = 512;      //0-1024 base voltage, 512 works the best as VCC/2
const bool Compensate = 1;  //Tries to compensate for idacref on by taking away 2048. Use BaseV to center the readings in this mode.

const int ADCClk = 3;  //Selects ADC clock generator, both to be between 3-8
const int PWMClk = 4;  //Selects PWM clock generator, they cant be the same.


//Not to be changed
int16_t a;       //Value dividor here Select between 3-8
int16_t Analog;  //Analog read values go here
int Period;      //Time period calculated here

int GCLKDIV;   //Dividor of PWM GCLK
int PRESC;     //Prescaler of PWM
int PRESVAL;   //Value by which this divides
int PRESCALC;  //Calculating which PRESC to use
int PWMCLKID;  //Find out which clock to attach.


uint32_t _tcNum;     //TC selector
uint8_t _tcChannel;  //TC channel selecttor

PinDescription _pinDesc;  //pin
uint32_t _pinAttr;        //pin attribute

void setup() {

  calc();                             //Calculates constants based on user set values
  if (IDACRef) { DACSetup(); };       //Setup DAC if needed
  PWMClock();                         //Sets up PWM clock speeds
  PWMSetup();                         //Sets up PWM
  genericClockSetup(ADCClk, ADCDiv);  //Sets up ADC clock speeds
  ADCSetup();                         //Sets up ADC
  ADCPort();                          //Selects ADCPort, next version will allow it to be changed
  ADC->SWTRIG.bit.START = true;       //Does first ADC read
}

void Tcxh() {

  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;  //Wait for new analog value to be ready
  Analog = ADC->RESULT.reg;               //Write it down
  ADC->SWTRIG.bit.START = true;           //Start reading again
}


/*  Wire up ADC ports  */
void ADCPort() {

  if (IDACRef) {
    ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN(GAIN) | ADC_INPUTCTRL_MUXNEG(0) | ADC_INPUTCTRL_MUXPOS(ADCpin);
  } else {
    ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN(GAIN) | ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS(ADCpin);
  };
}

/*  Set up ADC GCLK  */
void genericClockSetup(int clk, int dFactor) {
  // Enable the APBC clock for the ADC
  REG_PM_APBCMASK |= PM_APBCMASK_ADC;


  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(dFactor) |  // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(clk);        // Select Generic Clock (GCLK) 3
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization
  ;

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |        // Enable GCLK3
                     GCLK_GENCTRL_SRC_DFLL48M |  // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(clk);       // Select GCLK3
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(clk) | GCLK_CLKCTRL_ID_ADC;

  /* Wait for bus synchronization. */
  while (GCLK->STATUS.bit.SYNCBUSY) {};
}




/*  Sets up ADC  */

void ADCSetup() {



  /* Calibrate values. */
  uint32_t bias = (*((uint32_t *)ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
  uint32_t linearity = (*((uint32_t *)ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
  linearity |= ((*((uint32_t *)ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;

  /* Wait for bus synchronization. */
  while (ADC->STATUS.bit.SYNCBUSY) {};

  /* Write the calibration data. */
  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);

  while (ADC->STATUS.bit.SYNCBUSY) {};

  /* Use the internal VCC reference. This is 1/2 of what's on VCCA.
   since VCCA is typically 3.3v, this is 1.65v.
*/
  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;

  /* Number of ADC samples to capture */
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1;

  /* Sets resolution and uses smallest possible divider so cDIV has the most control */

  if (res == 8) {
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_8BIT;
  } else if (res == 10) {
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_10BIT;
  } else if (res == 12) {
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_12BIT;
  } else {
    Serial.println("Unsupported resolution, change the value res to 8 10 or 12");
  };
  /*  Allows for Diff mode so values can be negative  */
  if (IDACRef) {
    ADC->CTRLB.bit.DIFFMODE = 1;
  };

  ADC->SAMPCTRL.reg = 0x00;  //Ensures speed isnt limitedS

  while (ADC->STATUS.bit.SYNCBUSY) {};

  /* Enable the ADC. */
  ADC->CTRLA.bit.ENABLE = true;
}


void calc() {
  /*  Pin attributes  */
  _pinDesc = g_APinDescription[pin];
  _pinAttr = _pinDesc.ulPinAttribute;
  _tcNum = GetTCNumber(_pinDesc.ulPWMChannel);
  PWMCLKID = int(26 + (_tcNum / 2));                             //PWM CLK channel
  GCLKDIVCalc();                                                 //Calculate which GCLK divider to use
  Prescaler();                                                   //Calculate prescaler value
  Period = int((48000000 / Frequency / PRESVAL / GCLKDIV) - 1);  //Calculates number of cycles period takes up.
  /*  IDAC ref calculations  */
  if (IDACRef) {
    if (Compensate) {
      minv -= 2048;
      maxv -= 2048;
    };
    if (GAIN < 32) { GAIN = 2 * GAIN; };
  };
  a = (maxv - minv);  //Calculate by how much to divide
  //Calculate GAIN setup values
  if (GAIN == 1) {
    GAIN = 15;
  } else {
    GAIN = log2(GAIN / 2);
  };
}

/*  Empty  */
void loop() {
}

/*  Set up PWM clock  */
void PWMClock() {
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(GCLKDIV) | GCLK_GENDIV_ID(PWMClk);  // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |        // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |  // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(PWMClk);    // Select GCLK4


  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |        // Enable GCLK4 to TCC2 (and TC3)
                     GCLK_CLKCTRL_GEN(PWMClk) |  // Select GCLK4
                     GCLK_CLKCTRL_ID(PWMCLKID);  // Feed GCLK4 to TCC2 (and TC3)
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization


  while (GCLK->STATUS.bit.SYNCBUSY)
    ;
}

void PWMSetup() {

  //Which channel to use
  _tcChannel = GetTCChannelNumber(_pinDesc.ulPWMChannel);

  //Which timer to use
  if (_pinAttr & PIN_ATTR_TIMER) {
    pinPeripheral(pin, PIO_TIMER);
  } else {
    pinPeripheral(pin, PIO_TIMER_ALT);
  }



  // Check which timer to use
  if (_tcNum >= TCC_INST_NUM) {
    // Convert to 8-bit

    // -- Configure TC
    Tc *TCx = (Tc *)GetTC(_pinDesc.ulPWMChannel);

    //reset
    TCx->COUNT8.CTRLA.bit.SWRST = 1;

    while (TCx->COUNT16.STATUS.bit.SYNCBUSY)
      ;

    // Disable TCx
    TCx->COUNT8.CTRLA.bit.ENABLE = 0;

    while (TCx->COUNT16.STATUS.bit.SYNCBUSY)
      ;

    // Set Timer counter Mode to 8 bits, normal PWM, PRESCALER_DIV1
    TCx->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT8 | TC_CTRLA_WAVEGEN_NPWM | TC_CTRLA_PRESCALER_DIV1;

    while (TCx->COUNT16.STATUS.bit.SYNCBUSY)
      ;

    // Set the Dutycycle
    TCx->COUNT8.CC[_tcChannel].reg = (uint8_t)Period;

    while (TCx->COUNT16.STATUS.bit.SYNCBUSY)
      ;

    // Set PER to _compareValue to match frequency
    // convert to 8-bit
    TCx->COUNT8.PER.reg = Period >> 8;

    while (TCx->COUNT16.STATUS.bit.SYNCBUSY)
      ;

    /*  Set up interrupts  */
    TCx->COUNT8.EVCTRL.bit.OVFEO = 1;
    TCx->COUNT8.INTENSET.bit.OVF = 1;
    TCx->COUNT8.INTFLAG.bit.OVF = 1;

    interset();

    TCx->COUNT8.CTRLA.bit.PRESCALER = PRESC;  //Set calculated prescaler value
    TCx->COUNT8.CTRLA.bit.ENABLE = 1;         //Turn timer on

    while (TCx->COUNT16.STATUS.bit.SYNCBUSY)
      ;
  } else {


    // -- Configure TCC
    Tcc *TCCx = (Tcc *)GetTC(_pinDesc.ulPWMChannel);

    // Disable TCCx
    TCCx->CTRLA.bit.ENABLE = 0;

    while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
      ;

    // Set prescaler
    //TCCx->CTRLA.reg |= _prescalerConfigBits;

    // while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);

    // Set TCCx as normal PWM
    TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;

    while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
      ;

    // Set the Dutycycle
    TCCx->CC[_tcChannel].reg = Period;

    while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
      ;

    // Set PER to _compareValue to match frequency
    TCCx->PER.reg = Period;

    while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
      ;

    /*  Set up interrupts  */
    TCCx->EVCTRL.bit.OVFEO = 1;
    TCCx->INTENSET.bit.OVF = 1;
    TCCx->INTFLAG.bit.OVF = 1;

    interset();

    // Set prescaler value and enable
    TCCx->CTRLA.bit.PRESCALER = PRESC;
    TCCx->CTRLA.bit.ENABLE = 1;

    while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
      ;
  };
}



/*  Sets up interrupts based on the timer channel  */
void interset() {
  switch (_tcNum) {
    case (0):
      {
        NVIC_SetPriority(TCC0_IRQn, 3);
        NVIC_EnableIRQ(TCC0_IRQn);
      }
    case (1):
      {
        NVIC_SetPriority(TCC1_IRQn, 3);
        NVIC_EnableIRQ(TCC1_IRQn);
      }
    case (2):
      {
        NVIC_SetPriority(TCC2_IRQn, 3);
        NVIC_EnableIRQ(TCC2_IRQn);
      }
    case (3):
      {
        NVIC_SetPriority(TC3_IRQn, 3);
        NVIC_EnableIRQ(TC3_IRQn);
      }
    case (4):
      {
        NVIC_SetPriority(TC4_IRQn, 3);
        NVIC_EnableIRQ(TC4_IRQn);
      }
    case (5):
      {
        NVIC_SetPriority(TC5_IRQn, 3);
        NVIC_EnableIRQ(TC5_IRQn);
      }
  };
}



/*  Deals with and redirects all possible timer handles, this code may get simplified in the future if a way is found  */

void TCC0_Handler() {  //gets activated when PWM cycle ends

  Tcxh();

  TCC0->CC[_tcChannel].reg = Period * (Analog - minv) / a;

  TCC0->INTFLAG.bit.OVF = 1;  //reset interrupt flag
}


void TCC1_Handler() {  //gets activated when PWM cycle ends

  Tcxh();

  TCC1->CC[_tcChannel].reg = Period * (Analog - minv) / a;

  TCC1->INTFLAG.bit.OVF = 1;  //reset interrupt flag
}


void TCC2_Handler() {  //gets activated when PWM cycle ends

  Tcxh();

  TCC2->CC[_tcChannel].reg = Period * (Analog - minv) / a;

  TCC2->INTFLAG.bit.OVF = 1;  //reset interrupt flag
}

void TC3_Handler() {  //gets activated when PWM cycle ends

  Tcxh();

  TC3->COUNT8.CC[_tcChannel].reg = Period * (Analog - minv) / a;

  TC3->COUNT8.INTFLAG.bit.OVF = 1;  //reset interrupt flag
}

void TC4_Handler() {  //gets activated when PWM cycle ends

  Tcxh();

  TC4->COUNT8.CC[_tcChannel].reg = Period * (Analog - minv) / a;

  TC4->COUNT8.INTFLAG.bit.OVF = 1;  //reset interrupt flag
}

void TC5_Handler() {  //gets activated when PWM cycle ends

  Tcxh();

  TC5->COUNT8.CC[_tcChannel].reg = Period * (Analog - minv) / a;

  TC5->COUNT8.INTFLAG.bit.OVF = 1;  //reset interrupt flag
}

/*  Calculate which Prescaler to use  */

void Prescaler() {
  PRESCALC = (1500 / Frequency);

  if (PRESCALC > 256) {
    PRESC = 7;
    PRESVAL = 1024;
  } else if (PRESCALC > 64) {
    PRESC = 6;
    PRESVAL = 256;
  } else if (PRESCALC > 16) {
    PRESC = 5;
    PRESVAL = 64;
  } else if (PRESCALC > 8) {
    PRESC = 4;
    PRESVAL = 16;
  } else if (PRESCALC > 4) {
    PRESC = 3;
    PRESVAL = 8;
  } else if (PRESCALC > 2) {
    PRESC = 2;
    PRESVAL = 4;
  } else if (PRESCALC > 1) {
    PRESC = 1;
    PRESVAL = 2;
  } else {
    PRESC = 0;
    PRESVAL = 1;
  };
}

/*  Calculate which GCLK divider to use  */

void GCLKDIVCalc() {
  GCLKDIV = int(1.44 / Frequency);
  if (GCLKDIV < 1) { GCLKDIV = 1; };
  if (GCLKDIV > 255) { GCLKDIV = 255; };
}

/*  Set up the DAC  */

void DACSetup() {


  PM->APBCMASK.reg |= PM_APBCMASK_DAC;  //Deliver power

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(DAC_GCLK_ID) | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);  //Attacch clock

  DAC->CTRLA.reg = DAC_CTRLA_SWRST;  //Reset
  while (DAC->CTRLA.reg & DAC_CTRLA_SWRST)
    ;

  DAC->CTRLB.reg = DAC_CTRLB_EOEN | DAC_CTRLB_IOEN | DAC_CTRLB_REFSEL_AVCC;  //Enable external and internal refferences
  DAC->CTRLA.reg = DAC_CTRLA_ENABLE;                                         //Enable the DAC

  DAC->DATA.reg = BaseV;  //Set up the DAC output value
}
