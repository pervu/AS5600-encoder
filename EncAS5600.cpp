#include "EncAS5600.h"

#define DEFAULT_SPEED 8

// Create EncAS5600 object with default config
EncAS5600::EncAS5600(const modetype_t &mode)
{
    // Init default values
    _i2cAddress = 0x36;
    _mode = mode;
    _sda = 21;
    _scl = 22;
    _pwm = 15;
    _filter = 0;
    _delta = 110;
    _period = 10000;

    // Init default AS5600 registers
    _reg.bits.PM = 0;
    _reg.bits.HYST = 0;     //0b11
    _reg.bits.OUTS = 0;
    _reg.bits.PWMF = 0;
    _reg.bits.SF = 0;       //0b10
    _reg.bits.FTH = 0;      //0b111
    _reg.bits.WD = 0;
    _reg.bits.EMPTY = 0;

    printf("Encoder address 0x%x \n", _i2cAddress);
}

// Create EncAS5600 object and read settings from config
EncAS5600::EncAS5600(const modetype_t &mode, const as5600config_t &config)
{
    _i2cAddress = config.i2cAddress;
    _mode = mode;
    _sda = config.sda;
    _scl = config.scl;
    _pwm = config.pwmPin;
    _filter = config.filter;
    _delta = config.delta;
    _period = config.period;
    _reg = config.reg;

    printf("Encoder address 0x%x \n", _i2cAddress);
}

void EncAS5600::start()
{
    #if defined(ESP32)
        // Configure timer
        esp_timer_create_args_t timerConfig;
        timerConfig.arg = this;
        timerConfig.callback = reinterpret_cast<esp_timer_cb_t>(loopTmr);
        timerConfig.dispatch_method = ESP_TIMER_TASK;
        timerConfig.name = "encTimer";
        esp_timer_create(&timerConfig, &encTimer);
        // Start timer
        esp_timer_start_periodic(encTimer, _period);
    #endif
}

void EncAS5600::begin()
{ 
    if ((_mode == modetype_t::I2C) || (_mode == modetype_t::I2CPWM))
    {
        // I2C init
        Wire.begin(_sda, _scl);
        Wire.setClock(100000);

        //**** AS5600 Registers config ****
        writeByte(0x07, _reg.conf.byte1);
        writeByte(0x08, _reg.conf.byte2);
        
        // Read vals for testing
        uint8_t conf1 = readByte(0x07);
        uint8_t conf2 = readByte(0x08);
        printf("%x", conf1);
        printf("%x", conf2);
    }

    if ((_mode == modetype_t::PWM) || (_mode == modetype_t::I2CPWM))
    {
        pinMode(_pwm, INPUT);
    }
}

void EncAS5600::loop()
{
    EVERY_MS(_period/1000)
    {
        encCalcLoop();
    }
}

uint8_t EncAS5600::readByte(uint8_t addr)
{

    Wire.beginTransmission(_i2cAddress);
    Wire.write(addr);
    Wire.endTransmission();
    Wire.requestFrom(_i2cAddress, (uint8_t)1);
    uint32_t waitSteps = 0;
    while(!Wire.available())
    {
        waitSteps++;
        if (waitSteps > 4000)
        {
            Serial.println("I2C err");
            return 0;
        }
    }
    return Wire.read();
}

void EncAS5600::writeByte(uint8_t addr, uint8_t value)
{
    Wire.beginTransmission(_i2cAddress);
    Wire.write(addr);
    Wire.write(value);
    Wire.endTransmission(); 
}

bool EncAS5600::isConnected()
{
    if ((_mode == modetype_t::I2C) || (_mode == modetype_t::I2CPWM))
    {
        // Magnet connection status
        static uint8_t magnetStatus = 0;

        // Reading data from the module
        magnetStatus = readByte(0x0B);

        //////////////////////////////////////
        //     7 6  5  4  3 2 1 0           //
        //    | | |MD|ML|MH| | | |          //
        //  MH: Too strong magnet           //
        //  ML: Too weak magnet             //
        //  MD: Ok                          //
        //////////////////////////////////////
        if ((magnetStatus >> 5) & 0x01)
        {
            printf("Encoder OK\n");

            uint8_t agc = getAutomaticGainControl();
            printf("AGC %d \n", agc);

            return true;
        }
        else if ((magnetStatus >> 4) & 0x01)
        {
            printf("Too weak magnet");
        }
        else if ((magnetStatus >> 3) & 0x1)
        {
            Serial.println("Too strong magnet");
        }
        else{
            Serial.println("Unknown error");
        }
    }
    else
    {
        printf("I2C is not connected or run\n");
    }

    return false;
}

// AGC Register 0x1A
// The AGC register indicates the gain. For
// the most robust performance, the gain value should be in the
// center of its range. The airgap of the physical system can be
// adjusted to achieve this value.
uint8_t EncAS5600::getAutomaticGainControl()
{
    return readByte(0x1A);
}

double EncAS5600::getAngDeg()
{
    return _degAngle;
}

uint16_t EncAS5600::getRawAngle()
{
    return _rawAngle;
}

void EncAS5600::loopTmr(void *arg)
{
    EncAS5600 *obj = (EncAS5600 *)arg;
    obj->encCalcLoop();
}


void EncAS5600::setEncHandler(CallbackFunction f)
{
    tick_cb = f;
}

uint32_t EncAS5600::getSpeed()
{
    return _speed;
}

uint32_t EncAS5600::getRightDir()
{
    return _rightDirection;
}

uint32_t EncAS5600::getTicks()
{
    return _ticks;
}

void EncAS5600::_handleTick()
{
    if (tick_cb != NULL) tick_cb(*this);
}

uint16_t EncAS5600::readRawAngleI2C()
{
    uint16_t lowbyte;
    uint16_t highbyte;
    uint16_t rawAngle;

    lowbyte = readByte(0x0F);
    highbyte = readByte(0x0E);

    highbyte = highbyte << 8; //shifting to left
    rawAngle = highbyte | lowbyte; //uint16_t is 16 bits (as well as the word)

    if (_filter)
    {
        return static_cast<uint16_t>(kalmanFilter(rawAngle, 20.0f, 0.1f));
    }

    return rawAngle;
}

uint16_t EncAS5600::readRawAnglePWM()
{
    static uint16_t prev = 0;

    uint16_t rawAngle = analogRead(_pwm);
    rawAngle = constrain(rawAngle, 0, 4096);
    if (_filter)
    {
        return static_cast<uint16_t>(kalmanFilter(rawAngle, 20.0f, 0.1f));
    }
    return rawAngle;
}

void EncAS5600::encCalcLoop()
{
    static int angPrev = 0;
    static int changeStep = 0;

    static int prev = 0;

    //uint16_t rawAngle = 0;

    if ((_mode == modetype_t::I2C) || (_mode == modetype_t::I2CPWM))
    {
        _rawAngle = readRawAngleI2C();
    }
    else if (_mode == modetype_t::PWM)
    {
        _rawAngle = readRawAnglePWM();
    }

    if (_rawAngle == prev)
    {   
        // same value, nothing to calc, return
        prev = _rawAngle;
        return;
    }
    prev = _rawAngle;

    // Convert the relative value into degrees
    // 12 bits -> 4096 different levels per 360 degrees:
    // 360/4096 = 0.087890625
    // degAngle = rawAngle * 0.087890625;
    _degAngle = static_cast<double>(_rawAngle) * 0.087890625;
    uint16_t angCurr = _rawAngle;

    // Only take values that differ from the previous value by 10 degrees.
    // Changing the value by 10 degrees is approximately one click of the encoder.
    // One click is one tick
    if (angCurr >= _angPrev + _delta)
    {
        // Direction of rotation
        _rightDirection = true;

        // Skip the first tick, then calculate the rotation speed
        if (_ticksR == 0)
        {
            _rotTimeStart = millis();
            _speed = DEFAULT_SPEED;
        }
        else 
        {
            uint32_t speed = _ticksR*1000/(millis() - _rotTimeStart);
            if (_filter)
            {
                _speed = findMedianN_optim(constrain(speed, 1, 100));
            }
            else
            {
                _speed = constrain(speed, 1, 100);
            }
        }  

        _ticksR++;
        _angPrev = angCurr; 
        _rotTimeEnd = millis();
        _sendStatus = true;
    }

    if (angCurr <= _angPrev - _delta)
    {
        // Direction of rotation
        _rightDirection = false;

        // Skip the first tick, then calculate the rotation speed
        if (_ticksL == 0)
        {
            _rotTimeStart = millis();
            _speed = DEFAULT_SPEED;
        }
        else 
        {
            uint32_t speed = _ticksL*1000/(millis() - _rotTimeStart);
            if (_filter)
            {
                _speed = findMedianN_optim(constrain(speed, 1, 100));
            }
            else
            {
                _speed = constrain(speed, 1, 100);
            }
        }  

        _ticksL++;
        _angPrev = angCurr; 
        _rotTimeEnd = millis();
        _sendStatus = true;
    }

    // If there were random tick registrations in the other direction
    // don't count them
    // Select only _ticksR
    if ((_ticksR > _ticksL) && _sendStatus)
    {
        _rightDirection = true;
        _ticks = _ticksR;
        _sendStatus = false;
        // Send to callback func
        _handleTick();
    }
    // Or only _ticksL
    else if ((_ticksR < _ticksL) && _sendStatus)
    {
        _rightDirection = false;
        _ticks = _ticksL;
        _sendStatus = false;
        // Send to callback func
        _handleTick();
    }

    // Reset values if the knob does not rotate longer than 300 ms
    if (millis() - _rotTimeEnd > 300) {
        _sendStatus = false;
        _ticks = 0;
        _ticksL = 0;
        _ticksR = 0;
        _speed = DEFAULT_SPEED;
    }

}

//float _err_measure approximate measurement noise
//float _q rate of change of values 0.001-1, vary by yourself
int EncAS5600::kalmanFilter(const float &data, const float &_err_measure, const float &_q)
{
    float newVal = static_cast<float>(data);
    float _kalman_gain, _current_estimate;
    static float _err_estimate = _err_measure;
    static float _last_estimate;
    _kalman_gain = (float)_err_estimate / (_err_estimate + _err_measure);
    _current_estimate = _last_estimate + (float)_kalman_gain * (newVal - _last_estimate);
    _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
    _last_estimate = _current_estimate;
    return static_cast<int>(_current_estimate);
}

#define NUM_READ 3  // median order
// Median filter, optimal for filtering emissions
float EncAS5600::findMedianN_optim(float newVal)
{
  static float buffer[NUM_READ];  // static buffer
  static byte count = 0;
  buffer[count] = newVal;
  if ((count < NUM_READ - 1) and (buffer[count] > buffer[count + 1])) {
    for (int i = count; i < NUM_READ - 1; i++) {
      if (buffer[i] > buffer[i + 1]) {
        float buff = buffer[i];
        buffer[i] = buffer[i + 1];
        buffer[i + 1] = buff;
      }
    }
  } else {
    if ((count > 0) and (buffer[count - 1] > buffer[count])) {
      for (int i = count; i > 0; i--) {
        if (buffer[i] < buffer[i - 1]) {
          float buff = buffer[i];
          buffer[i] = buffer[i - 1];
          buffer[i - 1] = buff;
        }
      }
    }
  }
  if (++count >= NUM_READ) count = 0;
  return buffer[(int)NUM_READ / 2];
}