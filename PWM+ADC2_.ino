

const float Frequency = 20000.0f;  //PWM frequency, higher the frequency lower resolution. Wont work below 1000 hz without manually dividing the clock
const int cDiv = 5;    //Set up dividor of time for ADC, with high PWM frequencies high values may let the Duty period only to change every few cycles, while too low values may lead to less stable output
int GAIN = 1;          //1, 2, 4, 8, 16, 32, multiplier of input voltage
const int res = 12; //Set up Resolution of the ADC, 8 or 10 or 12 bits

//Calibration
const int minv = 0;    //Minimum meassured value of the input signal
const int maxv = 4000;  //Maximum meassured value og the input signal


//Not to be changed
int a; //Value dividor here
const int gClk = 3; //Selects ADC clock, not functioning in this version
int Analog; //Analog read values go here
int Period; //Time period calculated here

void setup() {

  calc(); //Calculates constants based on user set values
  PWMsetup(); //Sets up PWM
  genericClockSetup(gClk, cDiv);  //Sets up clock speeds
  ADCSetup(); //Sets up ADC
  ADCPort(); //Selects ADCPort, next version will allow it to be changed
  ADC->SWTRIG.bit.START = true; //Does first ADC read

}


void TCC2_Handler() { //gets activated when PWM cycle ends

  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY; //Wait for new analog value to be ready
  Analog = ADC->RESULT.reg; //Write it down

  ADC->SWTRIG.bit.START = true; //Start reading again

  REG_TCC2_CCB1 = Period/2; //Set PWM duty cycle based on the last Analog value and calibration value, center using minv

  TCC2->INTFLAG.bit.OVF = 1; //reset interrupt flag



}



void ADCPort() {
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN(GAIN) | ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_PIN0;

  ADC->INPUTCTRL.bit.MUXPOS = 0;

  //Selects port and sets Gaiin
}


void genericClockSetup(int clk, int dFactor) {
  // Enable the APBC clock for the ADC
  REG_PM_APBCMASK |= PM_APBCMASK_ADC;


  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(cDiv) |  // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(3);       // Select Generic Clock (GCLK) 3
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization
  ;

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |        // Enable GCLK3
                     GCLK_GENCTRL_SRC_DFLL48M |  // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(3);         // Select GCLK3
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK3 | GCLK_CLKCTRL_ID_ADC;

  /* Wait for bus synchronization. */
  while (GCLK->STATUS.bit.SYNCBUSY) {};
}



void PWMsetup() {
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |  
                    GCLK_GENDIV_ID(4);    // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |        // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |  // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);         // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  // Enable the port multiplexer for digital pin 13 (D13): timer TCC2 output
  PORT->Group[g_APinDescription[13].ulPort].PINCFG[g_APinDescription[13].ulPin].bit.PMUXEN = 1;

  // Connect the TCC0 timer to the port output - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[11].ulPort].PMUX[g_APinDescription[11].ulPin >> 1].reg = PORT_PMUX_PMUXO_E;

  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |       // Enable GCLK4 to TCC2 (and TC3)
                     GCLK_CLKCTRL_GEN_GCLK4 |   // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC2_TC3;  // Feed GCLK4 to TCC2 (and TC3)
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC2_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
                   TCC_WAVE_WAVEGEN_DSBOTTOM;  // Setup dual slope PWM on TCC2
  while (TCC2->SYNCBUSY.bit.WAVE)
    ;  // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  // 20000 = 50Hz, 10000 = 100Hz, 2500  = 400Hz
  REG_TCC2_PER = Period;  // Set the frequency of the PWM on TCC2
  while (TCC2->SYNCBUSY.bit.PER)
    ;

  REG_TCC2_CCB1 = 100;  // TCC2 CCB1 - duty cycle
  while (TCC2->SYNCBUSY.bit.CCB3)
    ;


  REG_TCC2_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |  // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;           // Enable the TCC2 output
  while (TCC2->SYNCBUSY.bit.ENABLE)
    ;  // Wait for synchronization

  NVIC_SetPriority(TCC2_IRQn, 0); //Sets up interrupt flag
  NVIC_EnableIRQ(TCC2_IRQn);

  REG_TCC2_INTFLAG |= TC_INTFLAG_OVF;  // Clear the interrupt flags
  REG_TCC2_INTENSET = TC_INTENSET_OVF;
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

  Period = int((48000000 / 2 / Frequency) - 1); //Calculates number of cycles period takes up.
  a = (maxv - minv); //Calculate by how much to divide
  //Calculate GAIN setup values
  if (GAIN == 1) {
    GAIN = 15;
  } else {
    GAIN = log2(GAIN / 2);
  };
}

void loop() {
}


