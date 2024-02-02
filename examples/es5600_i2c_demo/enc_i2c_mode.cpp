
#include <Arduino.h>
#include <EncAS5600.h>

EncAS5600 *as5600;

// Encoder handler
void encTick(EncAS5600 &e);

void setup() {
    
    Serial.begin(115200);
    while (Serial.available()) {};

    // ******* Конфигурация AS5600 ******************
    // .sda - I2C sda pin (default 21)
    // .scl - I2C scl pin (default 22)
    // .pwmPin - pin to read data in PWM and I2CPWM modes (default 15)
    // .i2cAddress - adress AS5600 (default 0x36)
    // .filter - filter of input data (1 - ON, 0 - OFF) (default 0)
    // .delta - the higher the value, the lower the sensitivity of the encoder (default 110)
    // .period - encoder polling frequency in µs (default 10000)
    // .reg - register configuration for AS5600 (See details below)
    // **********************************************
    // ******* AS5600 register configuration ********
    // Power Mode (PM)
    // 00 = NOM, 01 = LPM1, 10 = LPM2, 11 = LPM3
    // Hysteresis (HYST()
    // 00 = OFF, 01 = 1 LSB, 10 = 2 LSBs, 11 = 3 LSBs
    // Output Stage (OUTS)
    // 00 = analog (full range from 0% to 100% between GND and VDD), 01 = analog
    // (reduced range from 10% to 90% between GND and VDD), 10 = digital PWM
    // PWM Frequency (PWMF)
    // 00 = 115 Hz; 01 = 230 Hz; 10 = 460 Hz; 11 = 920 Hz
    // Slow Filter (SF)
    // 00 = 16x (Forced in Low Power Mode (LPM)); 01 = 8x; 10 = 4x; 11 = 2x
    // Fast Filter Threshold (FTH)
    // 000 = slow filter only, 001 = 6 LSBs, 010 = 7 LSBs, 011 = 9 LSBs,100 = 18 LSBs, 101
    // = 21 LSBs, 110 = 24 LSBs, 111 = 10 LSBs
    // Watchdog (WD)
    // 0 = OFF, 1 = ON
    // **********************************************
    // ******* Encoder modes ************************
    // I2C - encoder operation via i2c bus
    // I2CPWM - configuration via i2c bus, receiving data as an analog (PWM) signal
    // PWM - receiving data in the form of an analog signal,
    // in this mode it is impossible to write to AS5600 registers

    // Example of some parameters for configuring the AS5600
    as5600config_t conf;
    conf.sda = 21;
    conf.scl = 22;
    conf.pwmPin = 15;
    conf.filter = 0;
    conf.delta = 110;
    conf.period = 10000;
    conf.reg.bits.FTH = 0b111;
    conf.reg.bits.SF = 0b10;
    conf.reg.bits.HYST = 0b11;

    // Create an object to communicate with the encoder
    as5600 = new EncAS5600(modetype_t::I2C, conf);

    // To use the encoder with default parameters,
    // you can give only connection mode:
    // as5600 = new EncAS5600(modetype_t::I2C);

    // Initialize the encoder
    as5600->begin();

    // Check the connection, works only with I2C and I2CPWM modes
    if (!as5600->isConnected())
    {
        printf("Connection error!");
    }
    else
    {
        // AGC Register 0x1A
        // The AS5600 uses Automatic Gain Control in a closed loop to
        // compensate for variations of the magnetic field strength due
        // to changes of temperature, airgap between IC and magnet, and
        // magnet degradation. The AGC register indicates the gain. For
        // the most robust performance, the gain value should be in the
        // center of its range. The airgap of the physical system can be
        // adjusted to achieve this value.
        // In 5V operation, the AGC range is 0-255 counts. The AGC range
        // is reduced to 0-128 counts in 3.3V mode.
        printf("AutomaticGainControl: %d\n", as5600->getAutomaticGainControl());
        
        // Specify the handler function for the encoder
        as5600->setEncHandler(encTick);
        
        // Start polling the ecoder by timer (Default 10000 µs)
        as5600->start();

        printf("EncAS5600 starts\n\n");
    }
}

void loop() {
    // Instead of starting the timer, using the .start method, the 'encoder can be polled in a loop
    // Can use it with esp8266 or arduino
    //as5600->loop();
}

void encTick(EncAS5600 &e)
{
    String dir;
    if (e.getRightDir())
    {
        dir = "R";
    }
    else{
        dir = "L";
    }

    printf("Speed: %d\t Ticks: %d\t AngleDeg: %f\t RawAngle: %d\t Dir: %s\n", 
            e.getSpeed(), e.getTicks(), e.getAngDeg(), e.getRawAngle(), dir);
}

