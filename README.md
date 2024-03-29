
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
- Filter input signals


## AS5600 QuickStart
```cpp
#include <EncAS5600.h>

EncAS5600 *as5600;

// Encoder handler
void encTick(EncAS5600 &e);

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

void encTick(EncAS5600 &e)
{
    String dir;
    if (e.getRightDir()) dir = "R";
    else dir = "L";

    printf("Speed: %d\t Ticks: %d\t AngleDeg: %f\t RawAngle: %d\t Dir: %s\n", 
            e.getSpeed(), e.getTicks(), e.getAngDeg(), e.getRawAngle(), dir);
}
```

## Descriptions

**AS5600 chip**
<p align="center">
	<img src="https://github.com/pervu/AS5600-encoder/blob/main/as5600.png" alt="AS5600"/>
</p>

OUT is PWM pin (uses in PWM mode), to get analog values from AS5600


**Configure AS5600 as5600config_t**

.sda - I2C sda pin (default 21)<br>
.scl - I2C scl pin (default 22)<br>
.pwmPin - pin to read data in PWM and I2CPWM modes (default 15)<br>
.i2cAddress - adress AS5600 (default 0x36)<br>
.filter - filter of input data (1 - ON, 0 - OFF) (default 0)<br>
.delta - the higher the value, the lower the sensitivity of the encoder (default 110)<br>
.period - encoder polling frequency in µs (default 10000)<br>
.reg - register configuration for AS5600 (See details below)<br>


**AS5600 register configuration regConf_t reg**

Power Mode (PM)<br>
00 = NOM, 01 = LPM1, 10 = LPM2, 11 = LPM3<br>
Hysteresis (HYST()<br>
00 = OFF, 01 = 1 LSB, 10 = 2 LSBs, 11 = 3 LSBs<br>
Output Stage (OUTS)<br>
00 = analog (full range from 0% to 100% between GND and VDD), 01 = analog<br>
(reduced range from 10% to 90% between GND and VDD), 10 = digital PWM<br>
PWM Frequency (PWMF)<br>
00 = 115 Hz; 01 = 230 Hz; 10 = 460 Hz; 11 = 920 Hz<br>
Slow Filter (SF)<br>
00 = 16x (Forced in Low Power Mode (LPM)); 01 = 8x; 10 = 4x; 11 = 2x<br>
Fast Filter Threshold (FTH)<br>
000 = slow filter only, 001 = 6 LSBs, 010 = 7 LSBs, 011 = 9 LSBs,100 = 18 LSBs, 101<br>
= 21 LSBs, 110 = 24 LSBs, 111 = 10 LSBs<br>
Watchdog (WD)<br>
0 = OFF, 1 = ON<br>


**Encoder modes modetype_t**

- I2C - encoder operation via i2c bus
- I2CPWM - configuration via i2c bus, receiving data as an analog (PWM) signal
- PWM - receiving data in the form of an analog signal, in this mode it is impossible to write to AS5600 registers


**.getAutomaticGainControl() method**

The AS5600 uses Automatic Gain Control in a closed loop to
compensate for variations of the magnetic field strength due
to changes of temperature, airgap between IC and magnet, and
magnet degradation. The AGC register indicates the gain. For
the most robust performance, the gain value should be in the
center of its range. The airgap of the physical system can be
adjusted to achieve this value.
In 5V operation, the AGC range is 0-255 counts. The AGC range
is reduced to 0-128 counts in 3.3V mode.


