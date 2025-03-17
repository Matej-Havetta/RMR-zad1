#ifndef PID_H
#define PID_H


// PID controller class
class PIDController {
private:
    double Kp;
    double Ki;
    double Kd;
    double dt;
    double integral;
    double prev_error;

public:
    PIDController(double p, double i, double d, double dt, double integral, double prev_error);
    double update(double setpoint, double measured_value);

};
#endif // PID_H
