#include <PID.h>
#include <math.h>
#include <iostream>


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
    PIDController(double p, double i, double d, double dt, double integral, double prev_error) :
        Kp(p), Ki(i), Kd(d), dt(dt), integral(integral), prev_error(0.0) {}

    // double Update(double setpoint, double measured_value) {
    //     // Calculate the error
    //     double error = setpoint - measured_value;

    //     // Calculate the proportional term
    //     double proportional = kp_ * error;

    //     // Calculate the integral term
    //     integral_ += ki_ * error * dt_;

    //     // Calculate the derivative term
    //     double derivative = kd_ * (error - prev_error_) / dt_;
    //     prev_error_ = error;

    //     // Calculate the control output
    //     double output = proportional + integral_ + derivative;

    //     return output;
    // }
};
