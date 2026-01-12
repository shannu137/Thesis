#include <util/atomic.h>
#include <Encoder.h>
#include <math.h>
#include "INA219.h"
#include "MotorController.h"


MotorController::MotorController(int enca, int encb, int pwm, int in1, int in2, uint8_t INA_i2c_addr)
 : encoder(enca, encb), ina(INA_i2c_addr),
    pwmPin(pwm), in1Pin(in1), in2Pin(in2),
    prevT(0), posPrev(0),
    vFilt(0), vFiltPrev(0), vFiltPrevPrev(0),
    vPrev(0), vPrevPrev(0),
    eintegral(0), samples_no_current(0),
    kp(5), ki(5), vt(10)
{
}

void MotorController::begin()
{
    pinMode(pwmPin,OUTPUT);
    pinMode(in1Pin,OUTPUT);
    pinMode(in2Pin,OUTPUT);

    Wire.begin();
    if (!ina.begin() )
    {
    Serial.println("Could not connect. Fix and Reboot");
    }

    ina.setMaxCurrentShunt(3.2, 0.1);
    ina.setShuntSamples(7);
}

void MotorController::update()
{
    long pos = encoder.read();

    // Compute velocity
    long currT = micros();
    float deltaT = ((float)(currT-prevT)) / 1.0e6;
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
    float u = 7*vt + kp*e + ki*eintegral;

    // Anti-windup
    if (fabs(u) < 255 || u*e < 0) {
        eintegral += e * deltaT;
    }

    if (fabs(ina.getCurrent_mA()) < 1)
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
    setMotor(dir,pwr);

    Serial.print(millis());
    Serial.print(" ");
    Serial.print(vt);
    Serial.print(" ");
    Serial.print(vFilt);
    Serial.print(" ");
    Serial.print(pos);
    Serial.print(" ");
    Serial.println(ina.getCurrent_mA(), 2);

    delay(10);
}

void MotorController::setTargetVelocity(float target)
{
    vt = target;
}

void MotorController::setGains(float kp_set, float ki_set)
{
    kp = kp_set;
    ki = ki_set;
}

void MotorController::setMotor(int dir, int pwmVal)
{
    analogWrite(pwmPin,pwmVal); // Motor speed
    if(dir == -1){ 
        // Turn one way
        digitalWrite(in1Pin,HIGH);
        digitalWrite(in2Pin,LOW);
    }
    else if(dir == 1){
        // Turn the other way
        digitalWrite(in1Pin,LOW);
        digitalWrite(in2Pin,HIGH);
    }
    else{
        // Or dont turn
        digitalWrite(in1Pin,LOW);
        digitalWrite(in2Pin,LOW);    
    }
}

