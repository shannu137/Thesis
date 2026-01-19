#include <MotorController_Sim.h>

MotorController_Sim motor;

void setup(){
    Serial.begin(115200);
    // motor.setTargetVelocity(-10);
}

void loop(){
    MotorData data = motor.update();
    Serial.print(data.ts);
    Serial.print(" ");
    Serial.print(data.current_mA);
    Serial.print(" ");
    Serial.print(data.encoderPos);
    Serial.print(" ");
    Serial.print(data.pwm);
    Serial.print(" ");
    Serial.println(data.velocity);

    delay(10);
}
