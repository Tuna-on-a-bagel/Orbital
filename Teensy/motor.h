// motor.h
#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
#include <cstdint>

// Define the Encoder struct
struct Encoder {
  volatile int pos; // ISR stable counter
  int EA;           // hall effect pin A
  int EB;           // hall effect pin B
};

// Declare the Motor class
class Motor {
  public:
    Motor();

    void computeError();
    double proportion();
    double integral(double dt);
    double derivative(double dt);
    void remapPID(double PIDout, double PIDmin, double PIDmax);
    void PID_controller();
    void command(int pos);

    Encoder encoder; // init encoder struct

    int desPos;      // desired position (command from pi)
    int curPos;      // stable position variable -> updated from volatile encoder pos
    int dirPin;      // pin connection to controller
    int pwmPin;      // pin connection to controller
    int posHigh;     // logic for motor wiring configuration
    bool pwmInvert;  // logic for wiring configuration

    double kp;
    double ki;
    double kd;
    double PIDlimit; // resolution of policy
    unsigned long PIDrate;     // control rate

    double Ihist;
    double error;
    double prevErr;

    double policy;   // PID output
    int mCommand;    // pwm mapped PID output

  private:
    double prevTime; // Last time update was called
    double curTime;
};

#endif // MOTOR_H
