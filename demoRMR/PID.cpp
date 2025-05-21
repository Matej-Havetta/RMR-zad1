#include <PID.h>
#include <math.h>

// PID controller class
PIDController::PIDController (double p, double i, double d, double step, double prev_output) :
        Kp(p), Ki(i), Kd(d), step(step), prev_output(prev_output) {}


double PIDController::update(double setpoint, double measured_value) {
        double error = measured_value;
        double output = Kp * error;
        if(output>prev_output+step){
            output = prev_output+step;
        }
        prev_output=output;
        return output;
    }


void PIDController::setPrevOut(double new_prev) {
    prev_output = new_prev;
}

