#ifndef PID_HPP
#define PID_HPP
#include "iostream"
class PID {
public:
  PID(double kp, double ki, double kd, double min_output, double max_output,
      double integrator_min, double integrator_max, double dt, std::string axis,
      bool debug = false)
      : kp_(kp), ki_(ki), kd_(kd), min_output_(min_output),
        max_output_(max_output), integrator_min_(integrator_min),
        integrator_max_(integrator_max), dt_(dt), axis_(axis), debug_(debug) {

    if (debug) {
      printf("Initialized PID for: %s\n", axis_.c_str());
      printf("Kp: %.2f | Ki: %.2f | Kd: %.2f\n", kp_, ki_, kd_);
      printf("Output limits: %.2f to %.2f\n", min_output_, max_output_);
      printf("Integrator limits: %.2f to %.2f\n", integrator_min_, integrator_max_);
      printf("Dt: %.4f\n", dt_);
    }
  }

  void reset() {
    prev_error_ = 0.0;
    integrator_ = 0.0;
  }

  double update(double setpoint, double measurement) {
    double error = setpoint - measurement;

    double derivative = dt_ > 1e-6 ? (error - prev_error_) / dt_ : 0.0;

    integrator_ += error * dt_;
    if (integrator_ > integrator_max_)
      integrator_ = integrator_max_;
    else if (integrator_ < integrator_min_)
      integrator_ = integrator_min_;

    double output = kp_ * error + ki_ * integrator_ + kd_ * derivative;

    if (output > max_output_)
      output = max_output_;
    else if (output < min_output_)
      output = min_output_;

    prev_error_ = error;

    if (debug_) {
      printf("Axis: %s\n", axis_.c_str());
      printf("setpoint: %.2f | Measurement: %.2f | Error : %.2f\n", setpoint,
             measurement, error);
      printf("P: %.2f | I: %.2f | D: %.2f\n", kp_ * error, ki_ * integrator_,
             kd_ * derivative);
      printf("Output (clamped): %.2f\n", output);
    }

    return output;
  }

private:
  double kp_, ki_, kd_;
  double min_output_, max_output_;
  double integrator_min_, integrator_max_;
  double prev_error_ = 0.0, integrator_ = 0.0;
  double dt_;

  std::string axis_;
  bool debug_ = false;
};

#endif