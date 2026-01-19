#ifndef MotorController_h
#define MotorController_h

#include <Arduino.h>
// #include <Encoder.h>
#include <Wire.h>
// #include "INA219.h"

struct MotorData {
    float velocity;
    int pwm;
    long encoderPos;
    float current_mA;
};

class MotorController
{
    public: 
        MotorController(int enca, int encb, int pwm, int in1, int in2, uint8_t INA_i2c_addr);
        void begin();
        MotorData update();
        void setTargetVelocity(float target);
        void setGains(float kp_set, float ki_set);
        void setMotor(int dir, int pwmVal);

        void setExternalLoad(float load);

    private:
        // Encoder encoder;
        // INA219 ina;

        int pwmPin, in1Pin, in2Pin;

        long prevT, posPrev;
        
        float vFilt, vFiltPrev, vFiltPrevPrev;
        float vPrev, vPrevPrev;

        float eintegral;
        float samples_no_current;

        float kp, ki, vt;

        float external_load;     // Nm (to simulate stall)
        float sim_i = 0;
        float sim_omega = 0;
        float sim_theta = 0;
        float pwm_prev = 0;

        const float R = 2.5;     // Ohms
        const float L = 0.002;   // Henrys
        const float Ke = 0.015;  // Back-EMF constant
        const float Kt = 0.015;  // Torque constant
        const float J = 0.0001;  // Inertia
        const float b = 0.0001;  // Friction
        const float V_supply = 12.0;
};

#endif