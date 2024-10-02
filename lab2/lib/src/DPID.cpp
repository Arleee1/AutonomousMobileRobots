double DPID::compute(double error, double dt) {
    // Calculate integral
    integral += error * dt;
    // Calculate derivative
    double derivative = (error - previousError) / dt;
    // Update the previous error
    previousError = error;
    // Calculate the control signal
    return Kp * error + Ki * integral + Kd * derivative;
}