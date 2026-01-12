#ifndef MotorController_h
#define MotorController_h

#include <Arduino.h>
#include <Encoder.h>
#include <Wire.h>
#include "INA219.h"

class MotorController
{
    public: 
        MotorController(int enca, int encb, int pwm, int in1, int in2, uint8_t INA_i2c_addr);
        void begin();
        void update();
        void setTargetVelocity(float target);
        void setGains(float kp_set, float ki_set);
        void setMotor(int dir, int pwmVal);

    private:
        Encoder encoder;
        INA219 ina;

        int pwmPin, in1Pin, in2Pin;

        long prevT, posPrev;
        
        float vFilt, vFiltPrev, vFiltPrevPrev;
        float vPrev, vPrevPrev;

        float eintegral;
        float samples_no_current;

        float kp, ki, vt;
};

#endif