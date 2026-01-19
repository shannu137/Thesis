#include <MotorController_Sim.h>

MotorController_Sim motor;

void setup(){
    Serial.begin(115200);
    // motor.setTargetVelocity(-10);
}

void loop(){
    motor.update();
}
