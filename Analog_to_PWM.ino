const int ADCpin = 2;  //Pin for ADC, 0 cant be used in IDACRed mode as it has the DAC tied to it. AO can be used for ref GND for sound card with sound pin to ADCpin. On zero, 2 stands for A1
int PWMpin = 13;          //PWM pin, 13 is usually connected to LED so you can see it working.

const float Frequency = 1.0f;  //PWM frequency, higher the frequency lower resolution. Wont work below 0.00281 hz wont work
const int ADCDiv = 5;          //Set up dividor of time for ADC, with high PWM frequencies high values may let the Duty period only to change every few cycles, while too low values may lead to less stable output. Up to 255.
const int Samples = 1; //Ammount of ADC samples, can be 1, 2, 4, 8, 16, 32, 64, 128, 256, 512 or 1028
int GAIN = 1;                  //1, 2, 4, 8, 16, 32, multiplier of input, only goes up to 16 in ICADRef mode
const int res = 12;            //Set up Resolution of the ADC, 8 or 10 or 12 bits

/*  Calibration  */
int16_t minv = 0;     //Minimum meassured value of the input signal, make sure the value doesnt go below
int16_t maxv = 4095;  //Maximum meassured value og the input signal

/*  DAC AREF */
const bool IDACRef = 1;     //Use DAC as a reference instead of GND
const int BaseV = 512;      //0-1024 base voltage, 512 works the best as VCC/2
const bool Compensate = 1;  //Tries to compensate for idacref on by taking away 2048. Use BaseV to center the readings in this mode.

const bool Interrupt = 1;  //Enable PWM Interrupts

const int ADCClk = 3;  //Selects ADC clock generator, both to be between 3-8
const int PWMClk = 4;  //Selects PWM clock generator, they cant be the same.
const int DACClk = 0;  //DAC clock generator, doesnt really matter


//Not to be changed
int16_t a;       //Value dividor here Select between 3-8
int16_t Analog;  //Analog read values go here
int Period;      //Time period calculated here

int GCLKDIV;   //Dividor of PWM GCLK
int PRESC;     //Prescaler of PWM
int PRESVAL;   //Value by which this divides
int PRESCALC;  //Calculating which PRESC to use

uint8_t TCChannel;



void setup() {


  if (IDACRef) {
  AttachClock(DACClk, 0x21); 
  DACSetup(BaseV, DACClk);  //Setup DAC if needed
};
  genericClockSetup(ADCClk, ADCDiv);        //Sets up ADC clock and divides it
  AttachClock(ADCClk, 0x1E);                //Attaches ADC clock            //Attaches PWM clock
  PWMSetup(PWMpin, Frequency, PWMClk, Interrupt);  //Sets up PWM
  PWMSetup(9, 0.1, 5, 0);  //Sets up PWM again, if the pin shares the same clock or timer, it can cause issues.
                //Sets up ADC clock speeds
  ADCSetup(IDACRef, res, Samples);                                       //Sets up ADC
  AttachADC(ADCpin, GAIN, IDACRef, Compensate);                                        //Selects ADCPort, next version will allow it to be changed
  ADC->SWTRIG.bit.START = true;                     //Does first ADC read
}


//Shared TCC handler
void Tcxh() {

  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;  //Wait for new analog value to be ready
  Analog = ADC->RESULT.reg;               //Write it down
  ADC->SWTRIG.bit.START = true;           //Start reading again
}


/*  Repeating code  */
void loop() {
}


/*  Wire up ADC ports  */
void AttachADC(int ADCpin, int gain, bool IDACRefon, bool Compensate) {

  if (IDACRefon) {
  if (Compensate) {
      minv -= 2048;
      maxv -= 2048;
    };
  if (gain < 32) { gain = 2 * gain; };
  };
  a = (maxv - minv);  //Calculate by how much to divide
  //Calculate GAIN setup values
  if (gain == 1) {
    gain = 15;
  } else {
    gain = log2(gain / 2);
  }


  if (IDACRefon) {
    ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN(gain) | ADC_INPUTCTRL_MUXNEG(0) | ADC_INPUTCTRL_MUXPOS(ADCpin);
  } else {
    ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN(gain) | ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS(ADCpin);
  };
}

/*  Set up ADC GCLK  */
void genericClockSetup(int clk, int dFactor) {

  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(dFactor) | GCLK_GENDIV_ID(clk);  // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |        // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |  // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(clk);    // Select GCLK4


  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

}



/*  Sets up ADC  */

void ADCSetup(bool DacRef, int Res, int Samp) {



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




  /* Sets resolution and uses smallest possible divider so cDIV has the most control */

  if (Res == 8) {
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_8BIT;
  } else if (res == 10) {
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_10BIT;
  } else if (res == 12) {
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_12BIT;
  } else {
    Serial.println("Unsupported resolution, change the value res to 8 10 or 12");
  };
  /*  Allows for Diff mode so values can be negative  */
  if (DacRef) {
    ADC->CTRLB.bit.DIFFMODE = 1;
  };

  /* Number of ADC samples to capture */
  ADC->SAMPCTRL.reg = log2(Samp);

  while (ADC->STATUS.bit.SYNCBUSY) {};

  /* Enable the ADC. */
  ADC->CTRLA.bit.ENABLE = true;
}



/*  Set up PWM clock  */


void PWMSetup(int PMpin, float Frequency, int CLKID, bool EnablePWMInt) {

  PinDescription _pinDesc = g_APinDescription[PMpin];
  uint32_t _pinAttr = _pinDesc.ulPinAttribute;
  uint32_t _tcNum = GetTCNumber(_pinDesc.ulPWMChannel);
  uint8_t _tcChannel = GetTCChannelNumber(_pinDesc.ulPWMChannel);

  TCChannel = _tcChannel;

  GCLKDIVCalc(Frequency); 
  genericClockSetup(CLKID, GCLKDIV);       //Sets up PWM clock and divides it
  AttachClock(PWMClk, int(26 + (_tcNum / 2)));
  Prescaler(Frequency);   


  int period = int((48000000 / Frequency / PRESVAL / GCLKDIV) - 1);

  if (EnablePWMInt) { 
  Period = period;
  interset(_tcNum);
   };


  //Which timer to use
  if (_pinAttr & PIN_ATTR_TIMER) {
    SetupPWMPins(PMpin, PIO_TIMER);
  } else {
    SetupPWMPins(PMpin, PIO_TIMER_ALT);
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
    TCx->COUNT8.CC[_tcChannel].reg = (uint8_t)period;

    while (TCx->COUNT16.STATUS.bit.SYNCBUSY)
      ;

    // Set PER to _compareValue to match frequency
    // convert to 8-bit
    TCx->COUNT8.PER.reg = period >> 8;

    while (TCx->COUNT16.STATUS.bit.SYNCBUSY)
      ;

    /*  Set up interrupts  */
    TCx->COUNT8.EVCTRL.bit.OVFEO = 1;
    TCx->COUNT8.INTENSET.bit.OVF = 1;
    TCx->COUNT8.INTFLAG.bit.OVF = 1;



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
    TCCx->CC[_tcChannel].reg = period;

    while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
      ;

    // Set PER to _compareValue to match frequency
    TCCx->PER.reg = period;

    while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
      ;

    /*  Set up interrupts  */
    TCCx->EVCTRL.bit.OVFEO = 1;
    TCCx->INTENSET.bit.OVF = 1;
    TCCx->INTFLAG.bit.OVF = 1;



    // Set prescaler value and enable
    TCCx->CTRLA.bit.PRESCALER = PRESC;
    TCCx->CTRLA.bit.ENABLE = 1;

    while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
      ;
  };
}



/*  Sets up interrupts based on the timer channel  */
void interset(uint32_t _tcNum) {
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



/*  Calculate which Prescaler to use  */

void Prescaler(float Frequency) {
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

void GCLKDIVCalc(float Frequency) {
  GCLKDIV = int(1.44 / Frequency);
  if (GCLKDIV < 1) { GCLKDIV = 1; };
  if (GCLKDIV > 255) { GCLKDIV = 255; };
}

/*  Set up the DAC  */

void DACSetup(int BaseVoltage, int GCLKID) {




  DAC->CTRLA.reg = DAC_CTRLA_SWRST;  //Reset
  while (DAC->CTRLA.reg & DAC_CTRLA_SWRST)
    ;

  DAC->CTRLB.reg = DAC_CTRLB_EOEN | DAC_CTRLB_IOEN | DAC_CTRLB_REFSEL_AVCC;  //Enable external and internal refferences
  DAC->CTRLA.reg = DAC_CTRLA_ENABLE;                                         //Enable the DAC

  DAC->DATA.reg = BaseVoltage;  //Set up the DAC output value
}


void AttachClock(int clk, int clkid) {
   GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(clkid) | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(clk);  //Attacch clock
}




void SetupPWMPins(uint32_t ulPin, EPioType ulPeripheral) {  //Set up PWM pins


  if (g_APinDescription[ulPin].ulPin & 1)  // is pin odd?
  {
    uint32_t temp;

    // Get whole current setup for both odd and even pins and remove odd one
    temp = (PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg) & PORT_PMUX_PMUXE(0xF);
    // Set new muxing
    PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg = temp | PORT_PMUX_PMUXO(ulPeripheral);
    // Enable port mux
    PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR;
  } else  // even pin
  {
    uint32_t temp;

    temp = (PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg) & PORT_PMUX_PMUXO(0xF);

    PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg = temp | PORT_PMUX_PMUXE(ulPeripheral);

    PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR;  // Enable port mux
  }
}


void TCC0_Handler() {  //gets activated when PWM cycle ends

  Tcxh();

  TCC0->CC[TCChannel].reg = Period * (Analog - minv) / a;

  TCC0->INTFLAG.bit.OVF = 1;  //reset interrupt flag
}


void TCC1_Handler() {  //gets activated when PWM cycle ends

  Tcxh();

  TCC1->CC[TCChannel].reg = Period * (Analog - minv) / a;

  TCC1->INTFLAG.bit.OVF = 1;  //reset interrupt flag
}


void TCC2_Handler() {  //gets activated when PWM cycle ends

  Tcxh();

  TCC2->CC[TCChannel].reg = Period * (Analog - minv) / a;

  TCC2->INTFLAG.bit.OVF = 1;  //reset interrupt flag
}

void TC3_Handler() {  //gets activated when PWM cycle ends

  Tcxh();

  TC3->COUNT8.CC[TCChannel].reg = Period * (Analog - minv) / a;

  TC3->COUNT8.INTFLAG.bit.OVF = 1;  //reset interrupt flag
}

void TC4_Handler() {  //gets activated when PWM cycle ends

  Tcxh();

  TC4->COUNT8.CC[TCChannel].reg = Period * (Analog - minv) / a;

  TC4->COUNT8.INTFLAG.bit.OVF = 1;  //reset interrupt flag
}

void TC5_Handler() {  //gets activated when PWM cycle ends

  Tcxh();

  TC5->COUNT8.CC[TCChannel].reg = Period * (Analog - minv) / a;

  TC5->COUNT8.INTFLAG.bit.OVF = 1;  //reset interrupt flag
}
