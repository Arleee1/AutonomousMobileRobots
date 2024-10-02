#ifndef DPID_H
#define DPID_H
class DPID {
public:
    // PID gains
    double Kp;
    double Ki;
    double Kd;
    // Previous error and accumulated integral
    double previousError;
    double integral;

    // Constructor to initialize PID gains
    DPID(double Kp_, double Ki_, double Kd_)
        : Kp(Kp_), Ki(Ki_), Kd(Kd_), previousError(0), integral(0) {}

    // Method to calculate PID output
    double compute(double error, double dt);
};
#endif