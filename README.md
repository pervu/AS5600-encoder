
# AS5600

Arduino library for AS5600 and AS5600L 12-bits Magnetic Encoder


## Features

- Work with signals from I2C bus
- Work with signals from analog(PWM) pin
- Configure all AS5600 registers
- Get the Raw Angle
- Get the Deg Angle
- Get the Rotation Speed
- Check the magnet
- Get magnetic intensity


## AS5600 QuickStart
```cpp
#include <EncAS5600.h>

EncAS5600 *as5600;

void setup() {
    // Create an object to communicate with the encoder,
    // use the encoder with default parameters
    as5600 = new EncAS5600(modetype_t::I2C);

    // Initialize the encoder
    as5600->begin();

    // Specify the handler function for the encoder
    as5600->setEncHandler(encTick);

    // Start polling the ecoder by timer (Default 10000 µs)
    as5600->start();
}

void loop() {
// There is no need to use any updates in the loop
}
```

## Descriprions

**AS5600 chip**
<p align="center">
	<img src="https://github.com/pervu/AS5600-encoder/blob/main/as5600.png" alt="AS5600"/>
</p>
OUT is PWM pin (uses in PWM mode), to get analog values from AS5600

******* Configure AS5600 ******************
.sda - I2C sda pin (default 21)
.scl - I2C scl pin (default 22)
.pwmPin - pin to read data in PWM and I2CPWM modes (default 15)
.i2cAddress - adress AS5600 (default 0x36)
.filter - filter of input data (1 - ON, 0 - OFF) (default 0)
.delta - the higher the value, the lower the sensitivity of the encoder (default 110)
.period - encoder polling frequency in µs (default 10000)
.reg - register configuration for AS5600 (See details below)
**********************************************

******* AS5600 register configuration ********
Power Mode (PM)
00 = NOM, 01 = LPM1, 10 = LPM2, 11 = LPM3
Hysteresis (HYST()
00 = OFF, 01 = 1 LSB, 10 = 2 LSBs, 11 = 3 LSBs
Output Stage (OUTS)
00 = analog (full range from 0% to 100% between GND and VDD), 01 = analog
(reduced range from 10% to 90% between GND and VDD), 10 = digital PWM
PWM Frequency (PWMF)
00 = 115 Hz; 01 = 230 Hz; 10 = 460 Hz; 11 = 920 Hz
Slow Filter (SF)
00 = 16x (Forced in Low Power Mode (LPM)); 01 = 8x; 10 = 4x; 11 = 2x
Fast Filter Threshold (FTH)
000 = slow filter only, 001 = 6 LSBs, 010 = 7 LSBs, 011 = 9 LSBs,100 = 18 LSBs, 101
= 21 LSBs, 110 = 24 LSBs, 111 = 10 LSBs
Watchdog (WD)
0 = OFF, 1 = ON
**********************************************

******* Encoder modes ************************
I2C - encoder operation via i2c bus
I2CPWM - configuration via i2c bus, receiving data as an analog (PWM) signal
PWM - receiving data in the form of an analog signal,
in this mode it is impossible to write to AS5600 registers
**********************************************

**.getAutomaticGainControl method**
The AS5600 uses Automatic Gain Control in a closed loop to
compensate for variations of the magnetic field strength due
to changes of temperature, airgap between IC and magnet, and
magnet degradation. The AGC register indicates the gain. For
the most robust performance, the gain value should be in the
center of its range. The airgap of the physical system can be
adjusted to achieve this value.
In 5V operation, the AGC range is 0-255 counts. The AGC range
is reduced to 0-128 counts in 3.3V mode.


