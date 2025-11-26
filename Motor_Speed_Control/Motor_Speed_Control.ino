#include <util/atomic.h>
#include <Encoder.h>
#include <math.h>
#include "INA219.h"

// Pins
#define ENCA 18
#define ENCB 19
#define PWM 7
#define IN1 5
#define IN2 6

// globals
long prevT = 0;
long posPrev = 0;

Encoder myEnc(ENCA, ENCB);
INA219 INA(0x40);

float vFilt = 0;
float vFiltPrev = 0;
float vFiltPrevPrev = 0;
float vPrev = 0;
float vPrevPrev = 0;

float eintegral = 0;
float samples_no_current = 0;

void setup() {
  Serial.begin(115200);

  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

    Wire.begin();
    if (!INA.begin() )
    {
      Serial.println("Could not connect. Fix and Reboot");
    }
    INA.setMaxCurrentShunt(3.2, 0.1);
    INA.setShuntSamples(7);
}

void loop() {

  long pos = 0;
  pos = myEnc.read();

  // Compute velocity
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to cm/s
  float v = velocity * (2 * M_PI / 28080) * 6;

  // 2nd order Low-pass filter (10 Hz cutoff)
  vFilt = 0.07384 * v + 0.14768 * vPrev + 0.07384 * vPrevPrev + 1.09804 * vFiltPrev - 0.39341 * vFiltPrevPrev;
  vPrevPrev = vPrev;
  vPrev = v;
  vFiltPrevPrev = vFiltPrev;
  vFiltPrev = vFilt;

  // Set a target
  float vt = 10.0;  // rpm

  // Compute the control signal u
  float kp = 4;
  float ki = 2;
  float e = vt-vFilt;
  // eintegral = eintegral + e*deltaT;
  
  float u = 7*vt + kp*e + ki*eintegral;

// For integral windup
  if (fabs(u) < 255 || u*e < 0) {
    eintegral += e * deltaT;
  }

  if (fabs(INA.getCurrent_mA()) < 1)
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
  setMotor(dir,pwr,PWM,IN1,IN2);

  Serial.print(millis());
  Serial.print(" ");
  Serial.print(vt);
  Serial.print(" ");
  Serial.print(vFilt);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print(" ");
  Serial.println(INA.getCurrent_mA(), 2);
  delay(10);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == -1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == 1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}