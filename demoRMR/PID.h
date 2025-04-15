#ifndef PID_H
#define PID_H


// PID controller class
class PIDController {
private:
    double Kp;
    double Ki;
    double Kd;
    double step;
    double prev_output;

public:
    PIDController(double p, double i, double d, double step, double prev_output);
    double update(double setpoint, double measured_value);
    void setStep(double step);
    void setPrevOut(double new_prev);

};
#endif // PID_H
