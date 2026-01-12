#include <MotorController.h>

#define ENCA 18
#define ENCB 19
#define PWM 7
#define IN1 5
#define IN2 6


MotorController motor(ENCA, ENCB, PWM, IN1, IN2, 0x40);

void setup(){
    Serial.begin(115200);
    motor.begin();
    motor.setTargetVelocity(-10);
}

void loop(){
    motor.update();
}