#include <PID.h>
#include <math.h>

// PID controller class
PIDController::PIDController (double p, double i, double d, double dt, double integral, double prev_error) :
        Kp(p), Ki(i), Kd(d), dt(dt), integral(integral), prev_error(0.0) {}

double PIDController::update(double setpoint, double measured_value) {
        // // Calculate the error
        // double error = setpoint - measured_value;
        // // Calculate the proportional term
        // double proportional = Kp * error;
        // // Calculate the integral term
        // integral += Ki * error * dt;
        // // Calculate the derivative term
        // double derivative = Kd * (error - prev_error) / dt;
        // prev_error = error;
        // // Calculate the control output
        // double output = proportional + integral + derivative;

        double error = measured_value;
        //integral += error;
        //double derivative = error - prev_error;
        double output = Kp * error; //+ Ki * integral; // + Kd * derivative;
        prev_error = error;

        return output;
    }

