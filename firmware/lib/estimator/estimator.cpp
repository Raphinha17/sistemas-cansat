#include "estimator.h"

#include <math.h>

float baroAltitude(float p_pa, float p0_pa) {
  if (p_pa <= 0 || p0_pa <= 0) return 0.0f;
  return 44330.0f * (1.0f - powf(p_pa / p0_pa, 1.0f / 5.255f));
}

void VerticalKF::reset(float h0, float vz0) {
  x_[0] = h0;
  x_[1] = vz0;
  P[0][0] = baro_var_;
  P[0][1] = P[1][0] = 0;
  P[1][1] = 1.0f;
}

void VerticalKF::predict(float a_up, float dt) {
  // x = F x + B a,  F = [1 dt; 0 1],  B = [dt^2/2; dt]
  x_[0] += x_[1] * dt + 0.5f * a_up * dt * dt;
  x_[1] += a_up * dt;
  // P = F P F' + Q,  Q = sigma_a^2 * B B'
  const float p00 = P[0][0], p01 = P[0][1], p10 = P[1][0], p11 = P[1][1];
  const float q = acc_sigma_ * acc_sigma_;
  const float b0 = 0.5f * dt * dt, b1 = dt;
  P[0][0] = p00 + dt * (p10 + p01) + dt * dt * p11 + q * b0 * b0;
  P[0][1] = p01 + dt * p11 + q * b0 * b1;
  P[1][0] = p10 + dt * p11 + q * b0 * b1;
  P[1][1] = p11 + q * b1 * b1;
}

void VerticalKF::updateBaro(float h_baro) {
  // H = [1 0]
  const float y = h_baro - x_[0];
  const float s = P[0][0] + baro_var_;
  const float k0 = P[0][0] / s, k1 = P[1][0] / s;
  x_[0] += k0 * y;
  x_[1] += k1 * y;
  const float p00 = P[0][0], p01 = P[0][1];
  P[0][0] -= k0 * p00;
  P[0][1] -= k0 * p01;
  P[1][0] -= k1 * p00;
  P[1][1] -= k1 * p01;
  // Simetría numérica
  const float m = 0.5f * (P[0][1] + P[1][0]);
  P[0][1] = P[1][0] = m;
}

BaseCal::BaseCal(int window_s, float sample_hz) {
  window_ = window_s < 1 ? 1 : (window_s > kMaxWindow ? kMaxWindow : window_s);
  per_sec_ = (int)(sample_hz + 0.5f);
  if (per_sec_ < 1) per_sec_ = 1;
}

void BaseCal::reset() {
  acc_ = 0;
  acc_n_ = 0;
  head_ = filled_ = 0;
  has_ = frozen_ = false;
}

void BaseCal::add(float p_pa) {
  if (frozen_) return;
  acc_ += p_pa;
  acc_n_++;
  if (!has_) {
    // Valor provisional desde la primera muestra para poder calcular altura.
    p0_ = p_pa;
    has_ = true;
  }
  if (acc_n_ < per_sec_) return;
  ring_[head_] = (float)(acc_ / acc_n_);
  head_ = (head_ + 1) % window_;
  if (filled_ < window_) filled_++;
  acc_ = 0;
  acc_n_ = 0;
  double s = 0;
  for (int i = 0; i < filled_; i++) s += ring_[i];
  p0_ = (float)(s / filled_);
}
