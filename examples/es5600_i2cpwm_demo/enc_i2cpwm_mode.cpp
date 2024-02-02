#include <Arduino.h>
#include <EncAS5600.h>

EncAS5600 *as5600;

// Encoder handler
void encTick(EncAS5600 &e);

void setup() {
    
    Serial.begin(115200);
    while (Serial.available()) {};

    // Example of some parameters for configuring the AS5600
    as5600config_t conf;
    conf.sda = 21;
    conf.scl = 22;
    conf.pwmPin = 15;
    conf.filter = 0;
    conf.delta = 110;
    conf.period = 10000;

    // Create an object to communicate with the encoder
    as5600 = new EncAS5600(modetype_t::I2CPWM, conf);

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