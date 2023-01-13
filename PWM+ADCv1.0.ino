#include "wiring_private.h"

const int ADCpin = 0;           //Pin for ADC
int pin = 13;                   //PWM pin, 13 is usually connected to LED so you can see it working.
const float Frequency = 1.0f;  //PWM frequency, higher the frequency lower resolution. Wont work below 0.035 hz wont work.
const int ADCDiv = 5;             //Set up dividor of time for ADC, with high PWM frequencies high values may let the Duty period only to change every few cycles, while too low values may lead to less stable output. Up to 255.
int GAIN = 1;                   //1, 2, 4, 8, 16, 32, multiplier of input voltage
const int res = 12;             //Set up Resolution of the ADC, 8 or 10 or 12 bits
const int ADCClk = 3;  //Selects ADC clock generator, both to be between 3-8
const int PWMClk = 4;  //Selects PWM clock generator, they cant be the same. 





//Calibration
const int minv = 0;     //Minimum meassured value of the input signal
const int maxv = 4000;  //Maximum meassured value og the input signal


//Not to be changed
int a;               //Value dividor here Select between 3-8
int Analog;          //Analog read values go here
int Period;          //Time period calculated here

int GCLKDIV;
int PRESC;
int PRESVAL;
int PRESCALC;
int PWMCLKID;


uint32_t _tcNum;
uint8_t _tcChannel;

PinDescription _pinDesc;
uint32_t _pinAttr;

void setup() {

  calc();                         //Calculates constants based on user set values
  PWMClock();
  PWMSetup();                     //Sets up PWM
  genericClockSetup(ADCClk, ADCDiv);  //Sets up clock speeds
  ADCSetup();                     //Sets up ADC
  ADCPort();                      //Selects ADCPort, next version will allow it to be changed
  ADC->SWTRIG.bit.START = true;   //Does first ADC read
  Serial.begin(2000000);
}

void Tcxh() {


  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;  //Wait for new analog value to be ready
  Analog = ADC->RESULT.reg;               //Write it down
  ADC->SWTRIG.bit.START = true;           //Start reading again
}




void ADCPort() {
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN(GAIN) | ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_PIN0;

  ADC->INPUTCTRL.bit.MUXPOS = ADCpin;

  //Selects port and sets Gaiin
}


void genericClockSetup(int clk, int dFactor) {
  // Enable the APBC clock for the ADC
  REG_PM_APBCMASK |= PM_APBCMASK_ADC;


  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(dFactor) |  // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(clk);       // Select Generic Clock (GCLK) 3
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization
  ;

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |        // Enable GCLK3
                     GCLK_GENCTRL_SRC_DFLL48M |  // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(clk);         // Select GCLK3
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(clk) | GCLK_CLKCTRL_ID_ADC;

  /* Wait for bus synchronization. */
  while (GCLK->STATUS.bit.SYNCBUSY) {};
}






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




  ADC->SAMPCTRL.reg = 0x00;  //Ensures speed isnt limitedS

  while (ADC->STATUS.bit.SYNCBUSY) {};

  /* Enable the ADC. */
  ADC->CTRLA.bit.ENABLE = true;
}


void calc() {

  _pinDesc = g_APinDescription[pin];
  _pinAttr = _pinDesc.ulPinAttribute;
  _tcNum = GetTCNumber(_pinDesc.ulPWMChannel);
  PWMCLKID = int(26+(_tcNum/2));
  GCLKDIVCalc();
  Prescaler();
  Period = int((48000000 / Frequency / PRESVAL / GCLKDIV) - 1);  //Calculates number of cycles period takes up.
  a = (maxv - minv);                                   //Calculate by how much to divide
  //Calculate GAIN setup values
  if (GAIN == 1) {
    GAIN = 15;
  } else {
    GAIN = log2(GAIN / 2);
  };
}

void loop() {
}

void PWMClock() {
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(GCLKDIV) |  
                    GCLK_GENDIV_ID(PWMClk);    // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |        // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |  // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(PWMClk);         // Select GCLK4


  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |       // Enable GCLK4 to TCC2 (and TC3)
                     GCLK_CLKCTRL_GEN(PWMClk) |   // Select GCLK4
                     GCLK_CLKCTRL_ID(PWMCLKID);  // Feed GCLK4 to TCC2 (and TC3)
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization


  while (GCLK->STATUS.bit.SYNCBUSY);
}

void PWMSetup() {
  // New pin or freqChange






  _tcChannel = GetTCChannelNumber(_pinDesc.ulPWMChannel);

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

    // Set Timer counter Mode to 8 bits, normal PWM, PRESCALER_DIV256
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

    TCx->COUNT8.EVCTRL.bit.OVFEO = 1;
    TCx->COUNT8.INTENSET.bit.OVF = 1;
    TCx->COUNT8.INTFLAG.bit.OVF = 1;

    interset();

    TCx->COUNT8.CTRLA.bit.PRESCALER = PRESC;
    TCx->COUNT8.CTRLA.bit.ENABLE = 1;

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


    TCCx->EVCTRL.bit.OVFEO = 1;
    TCCx->INTENSET.bit.OVF = 1;
    TCCx->INTFLAG.bit.OVF = 1;


    interset();



    // Enable TCCx
    TCCx->CTRLA.bit.PRESCALER = PRESC;
    TCCx->CTRLA.bit.ENABLE = 1;

    while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
      ;
  };
}




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

void Prescaler() {
  PRESCALC = (2000 / Frequency);

  if (PRESCALC > 64) {
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

void GCLKDIVCalc() {
GCLKDIV = int(8/Frequency);
if (GCLKDIV < 1) {GCLKDIV = 1;};
if (GCLKDIV > 255) {GCLKDIV = 255;};
}
