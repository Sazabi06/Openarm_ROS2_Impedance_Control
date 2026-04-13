#pragma once
#include <cmath>

namespace openarm_hw_control
{
/**
 * @brief Zero-allocation 1st-order Low Pass Filter for real-time control.
 * Used to smoothly ramp payload mass to prevent torque spikes.
 */
class LowPassFilter
{
public:
    LowPassFilter() : y_prev_(0.0), alpha_(1.0) {}

    // dt = 0.002s (500Hz), tau = time constant (e.g. 0.5s)
    void configure(double dt, double tau) {
        if (tau <= 0.0) {
            alpha_ = 1.0;
        } else {
            alpha_ = dt / (tau + dt);
        }
    }

    // Step the filter in the real-time update loop
    double limit(double x) {
        y_prev_ = alpha_ * x + (1.0 - alpha_) * y_prev_;
        return y_prev_;
    }

    void reset(double val) { y_prev_ = val; }
    double get() const { return y_prev_; }

private:
    double y_prev_;
    double alpha_;
};
} // namespace openarm_hw_control
