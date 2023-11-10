#include "motor.h"
#include <Arduino.h>


Motor::Motor() {
 
  PIDlimit = 1000;    //resolution of policy
  PIDrate = 5;        //control rate
  policy = 0;
  Ihist = 0.0;
  error = 0.0;
  prevErr = 0.0;
}

void Motor::computeError() {
  //Compute error
  error = desPos - curPos;
}

double Motor::proportion() {
  //compute propportional term
  double P = kp * error;
  return P;
}

double Motor::integral(double dt) {
  //compute integral term
  if (-PIDlimit < policy < PIDlimit) {
    Ihist += error * dt;
  }
  double I = constrain(ki * Ihist, -900, 900);
  return I;
}

double Motor::derivative(double dt) {
  //compute derivative term
  double dx = (error - prevErr)/dt;
  double D = kd * dx;
  return D;
}

void Motor::remapPID(double PIDout, double PIDmin, double PIDmax) {
  //Policy is on [-1000:1000] range, must remap to PWM values for motor commands
  int pwmCommand = (int)(255.0 * ((policy) / (PIDmax))); //Normalize
  mCommand = constrain(abs(pwmCommand), 0, 255);         //Ensure clipping
  if (curPos < desPos) {
    if (pwmInvert) {
      mCommand = 255 - mCommand;                         //motor specific configuration
    }
  } else {
    if (!pwmInvert) {
      mCommand = 255 - mCommand;
    }
  }

  /*Serial.print("pos: ");
  Serial.print(curPos);
  Serial.print("  policy: ");
  Serial.print(policy);
  Serial.print("  pwmCmd: ");
  Serial.print(pwmCommand);
  Serial.print("  mCommand: ");
  Serial.print(mCommand);*/
}

void Motor::PID_controller() {
  //update feedback policy term

  curTime = millis();                     //get current time
  double dt = (curTime - prevTime)/1000;  //compute change in time since last call

  computeError(); 

  //compute policy terms
  double P = proportion();                //compute proportinal term
  double I = integral(dt);                //compute Integral term
  //Integral wrapup
  if ((abs(error)*2 + abs(prevErr)*2) < 1) {
    Ihist = 0;
  }
  double D = derivative(dt);              //compute derivative

  //update params
  prevErr = error;
  prevTime = curTime;

  //update policy
  policy = constrain(P + I + D, -PIDlimit, PIDlimit);
  remapPID(policy, -PIDlimit, PIDlimit);

  /*Serial.print(" P: ");
  Serial.print(P);
  Serial.print(" I: ");
  Serial.print(I);
  Serial.print(" D: ");
  Serial.println(D);*/

}

void Motor::command(int pos) {
  desPos = pos;
  PID_controller();
  if (policy < 0) {
    if (posHigh) {
      digitalWrite(dirPin, LOW);
    } else {
      digitalWrite(dirPin, HIGH);
    }
  } else {
    if (posHigh) {
      digitalWrite(dirPin, HIGH);
    } else {
      digitalWrite(dirPin, LOW);
    }
  }
  analogWrite(pwmPin, mCommand);
}

