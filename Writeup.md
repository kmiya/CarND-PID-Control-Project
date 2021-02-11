# PID Controller

### The PID procedure follows what was taught in the lessons.
See the following code. The functions follow what was taught in the lessons.
```c++
void PID::UpdateError(const double cte) {
  /**
   * Update PID errors based on cte.
   */
  p_error_ = cte * Kp_;
  // it may overflow after a long-distance driving
  i_error_ += cte;
  d_error_ = cte - pre_cte_;
  pre_cte_ = cte;
}

double PID::TotalError() const {
  /**
   * Calculate and return the total error
   */
  return -Kp_ * p_error_ -Ki_ * i_error_ - Kd_ * d_error_;
}
```

### Describe the effect each of the P, I, D components had in your implementation.

If the factor `Kp` of the P component is increased,
the car tries to return to the center of the lane more quickly.

If the factor `Kd` of the D component is increased,
the oscillation of the steer angle is reduced more quickly.

The I component is used to reduce the bias between the car position and the center of the lane.

### Describe how the final hyperparameters were chosen.
Done through manual tuning. Tried to make the driving more comfortable and safe.