#include <util/atomic.h>
#include <math.h>
#include "MotorController_Sim.h"


MotorController_Sim::MotorController_Sim()
 :  prevT(0), posPrev(0),
    vFilt(0), vFiltPrev(0), vFiltPrevPrev(0),
    vPrev(0), vPrevPrev(0),
    eintegral(0), samples_no_current(0),
    kp(5), ki(5), vt(10), external_load(0), 
    sim_theta(0), sim_i(0), sim_omega(0), pwm_prev(0)
{
}


MotorData MotorController_Sim::update()
{
    // Simulates Motor
    long currT = millis();
    float deltaT = ((float)(currT-prevT)) / 1.0e3;

    if(deltaT < 1e-3){
        prevT = currT;
        return MotorData{};
    }

    float Vin = (pwm_prev / 255.0) * V_supply;
    sim_i += deltaT * (Vin - R * sim_i - K * sim_omega) / L;

    if (!stalled){
        sim_omega += deltaT * (K * sim_i - b * sim_omega - external_load) / J;
    }

    if (fabs(sim_omega) < 0.01 && fabs(K * sim_i - b * sim_omega) < external_load) {
        sim_omega = 0;
        stalled = true;
    }

    sim_theta += sim_omega * deltaT;

    long pos = (long) (sim_theta * 28080 / (2 * M_PI));
    float current = sim_i * 1000;

    // Compute velocity
    float velocity = (pos - posPrev) / deltaT;
    posPrev = pos;
    prevT = currT;

    // Convert count/s to cm/s
    // 6 is radius of wheel
    float v = velocity * (2 * M_PI / 28080) * 6;

    // 2nd order Low-pass filter (10 Hz cutoff)
    vFilt = 0.07384 * v + 0.14768 * vPrev + 0.07384 * vPrevPrev + 1.09804 * vFiltPrev - 0.39341 * vFiltPrevPrev;
    vPrevPrev = vPrev;
    vPrev = v;
    vFiltPrevPrev = vFiltPrev;
    vFiltPrev = vFilt;

    // Compute the control signal u
    float e = vt-vFilt;        
    float u = 12*vt + kp*e + ki*eintegral;

    // Anti-windup
    if (fabs(u) < 255 || u*e < 0) {
        eintegral += e * deltaT;
    }

    if (fabs(current) < 1)
    {
        samples_no_current += 1;
        if (samples_no_current > 10){
            eintegral = 0;
            samples_no_current = 0;
        }
    }
    else{
        samples_no_current = 0;
    }

    // Set the motor speed and direction
    int dir = 1;
    if (u<0){
        dir = -1;
    }
    int pwr = (int) fabs(u);
    if(pwr > 255){
        pwr = 255;
    }
    pwm_prev = pwr * dir;

    // Serial.print(millis());
    // Serial.print(" ");
    // Serial.print(vt);
    // Serial.print(" ");
    // Serial.print(vFilt);
    // Serial.print(" ");
    // Serial.print(pos);
    // Serial.print(" ");
    // Serial.println(current, 2);

    MotorData output;
    output.ts = currT;
    output.velocity = vFilt;
    output.encoderPos = pos;
    output.pwm = pwr * dir;
    output.current_mA = current;

    return output;
}

void MotorController_Sim::setTargetVelocity(float target)
{
    vt = target;
}

void MotorController_Sim::setGains(float kp_set, float ki_set)
{
    kp = kp_set;
    ki = ki_set;
}

void MotorController_Sim::setExternalLoad(float load) {
    external_load = load;
}