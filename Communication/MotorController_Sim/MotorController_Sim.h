#ifndef MotorController_Sim_h
#define MotorController_Sim_h

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

struct MotorData {
    long ts;
    float velocity;
    int pwm;
    long encoderPos;
    float current_mA;
};

class MotorController_Sim
{
    public: 
        MotorController_Sim();
        MotorData update();
        void setTargetVelocity(float target);
        void setGains(float kp_set, float ki_set);
        void setMotor(int dir, int pwmVal);

        void setExternalLoad(float load);

    private:

        long prevT, posPrev;
        
        float vFilt, vFiltPrev, vFiltPrevPrev;
        float vPrev, vPrevPrev;

        float eintegral;
        float samples_no_current;

        float kp, ki, vt;

        float sim_i, sim_omega, sim_theta;
        float pwm_prev;

        float external_load; 
        const float V_supply = 12;
        const float T_stall = 8;
        // ((33 / 60) * 2 * M_PI)
        const float omega_NL = (float) 33 * 2 * M_PI / 60;
        const float i_NL = 0.450; 
        const float J = 0.1;
        const float L = 0.2;

        const float K = V_supply / ((V_supply / T_stall) * i_NL + omega_NL);
        const float R = K * V_supply / T_stall;
        const float b = K * i_NL / omega_NL;
        bool stalled = false; 
};

#endif