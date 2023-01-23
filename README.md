# Fast-Analog-to-PWM-for-SAMD21-Arduino

This project is now part of Speeduino, you can find it at https://github.com/Bexin3/Speeduino
SAMC21 version (untested) - https://github.com/Bexin3/Snowduino-Null


A .ino project for SAMD21 Arduinos tested on Arduino Zero that allows conversion of Analog values into PWM duty cycle, tested frequencies are 0.00281-200000hz
Newer versions support m0 seeduino xiao and custom choosing of analog input and PWM output pins, as well as higher frequency range.
Newest version has added a support for use of the DAC as an internal voltage reference. Defualtly it is set to produce half the voltage. This voltage has its output at A0 so dont use it for the ADC in this mode. Instead you can for example connect GND of a sound card, or well analog signal that goes below 0v to A0, and the positive to your selected ADC pin, so you can also meassure negative values. In cases like this having "GND" value on ADC pin would result in 50% duty cycle. For this use its recommended to set up gain, and minimum as well as maximum value the card may output, though I recommend turning of compensation and doing this manually. Make sure the value doesnt go under minimum set as that would cause an overflow, and with something like a soundcard minimum and maximum should be roughly similar magnitude.
