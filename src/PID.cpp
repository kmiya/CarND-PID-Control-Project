#include "PID.h"

void PID::Init(const double Kp, const double Ki, const double Kd) noexcept {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  pre_cte_ = 0;
  p_error_ = 0;
  i_error_ = 0;
  d_error_ = 0;
}

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