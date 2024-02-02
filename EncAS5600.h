//***************************************************
//* Library to simplify working with AS5600 module  *
//* For ESP32, ESP8266, Arduino                     *
//* Copyright (C) 2024 Pavel Pervushkin.            *
//* Released under the MIT license.                 *
//***************************************************

#ifndef EncAS5600_h
#define EncAS5600_h

#include <Arduino.h>
#include "Wire.h"
#include "esp_timer.h"

//===========================
#define EVERY_MS(x) \
  static uint32_t tmr;\
  bool flag = millis() - tmr >= (x);\
  if (flag) tmr += (x);\
  if (flag)
//===========================


typedef enum {
  I2C,
  PWM,
  I2CPWM
} modetype_t;

union as5600regword {
  struct {
    uint8_t PM : 2;
    uint8_t HYST : 2;
    uint8_t OUTS : 2;
    uint8_t PWMF : 2;
    uint8_t SF : 2;
    uint8_t FTH : 3;
    uint8_t WD : 1;
    uint8_t EMPTY : 2;
  } bits;
  struct {
    uint8_t byte2;
    uint8_t byte1;
  } conf;
};

typedef union as5600regword regConf_t;

typedef struct {
  bool filter = 1;
  uint8_t delta = 110;
  uint8_t i2cAddress = 0x36;
  uint8_t sda = 21;
  uint8_t scl = 22;
  uint8_t pwmPin = 15;
  uint32_t period = 10000;
  regConf_t reg;
} as5600config_t;



class EncAS5600
{
private:
  uint8_t _i2cAddress;

  esp_timer_handle_t encTimer;
  uint32_t _timer = 0;
  // Working mode
  modetype_t _mode;
  // ON/OFF data filter
  uint8_t _filter;
  // delta size - higher the value, lower the sensitivity
  uint8_t _delta;
  // i2c pins
  uint8_t _sda;
  uint8_t _scl;
  // pwm pin
  uint8_t _pwm;
  // polling period
  uint16_t _period;
  // register struct
  regConf_t _reg;

  // Vars for the calced values
  uint32_t _speed = 0;
  uint32_t _ticks = 0;
  uint16_t _rawAngle = 0;
  double _degAngle = 0;
  uint32_t _ticksL = 0;
  uint32_t _ticksR = 0;

  // direction of encoder knob rotation
  bool _rightDirection = true;
  // secondary variables for encCalc()
  int _angPrev = 0;
  uint32_t _rotTimeStart = 0;
  uint32_t _rotTimeEnd = 1;
  bool _sendStatus = false;
  // read one byte from i2c
  uint8_t readByte(uint8_t addr);
  // write one byte to i2c
  void writeByte(uint8_t addr, uint8_t value);
  // Angle reading in absolute values for I2C, I2CPWM modes
  uint16_t readRawAngleI2C();
  // Angle reading in absolute values for PWM mode
  uint16_t readRawAnglePWM();
  // Getting speed and ticks
  void encCalcLoop();
  // Filtering data from noise
  int kalmanFilter(const float &data, const float &_err_measure, const float &_q);
  float findMedianN_optim(float newVal);

protected:
  typedef std::function<void(EncAS5600 &enc)> CallbackFunction;

  CallbackFunction tick_cb = NULL;
  void _handleTick();

  static void loopTmr(void *arg);

public:
  EncAS5600(const modetype_t &mode);
  EncAS5600(const modetype_t &mode, const as5600config_t &config);
  ~EncAS5600(){}

  void begin();
  // Timer start
  void start();
  // Encoder polling in a cycle
  void loop();
  // Checking that the encoder is working
  bool isConnected();
  // Callback for calling a function outside of a class
  void setEncHandler(CallbackFunction f);
  // Getting the speed value
  uint32_t getSpeed();
  // Getting the direction of encoder knob rotation
  uint32_t getRightDir();
  // Getting ticks per encoder rotation
  uint32_t getTicks();
  // Gain control
  uint8_t getAutomaticGainControl();
  // Get angle in degrese values (0 - 360)
  double getAngDeg();
  // Get angle in absolute values (0 - 4096)
  uint16_t getRawAngle();
};

#endif