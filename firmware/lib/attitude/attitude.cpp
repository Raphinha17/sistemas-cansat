#include "attitude.h"

#include <math.h>

static constexpr float kDeg2Rad = 0.017453292519943295f;
static constexpr float kRad2Deg = 57.29577951308232f;

void Mahony::reset() {
  q[0] = 1;
  q[1] = q[2] = q[3] = 0;
  ix_ = iy_ = iz_ = 0;
  acc_used_ = false;
}

void Mahony::initFromAccel(float ax, float ay, float az) {
  // Inversa de g_cuerpo = [-sin(p), sin(r)cos(p), cos(r)cos(p)], yaw = 0.
  const float r = atan2f(ay, az);
  const float p = atan2f(-ax, sqrtf(ay * ay + az * az));
  const float cr = cosf(r / 2), sr = sinf(r / 2), cp = cosf(p / 2), sp = sinf(p / 2);
  q[0] = cr * cp;
  q[1] = sr * cp;
  q[2] = cr * sp;
  q[3] = -sr * sp;
  ix_ = iy_ = iz_ = 0;
}

void Mahony::update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  gx *= kDeg2Rad;
  gy *= kDeg2Rad;
  gz *= kDeg2Rad;
  float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

  // Corrección con el acelerómetro solo cerca de 1 g (en caída libre o con arrastre
  // fuerte el acelerómetro no indica la vertical).
  const float an = sqrtf(ax * ax + ay * ay + az * az);
  acc_used_ = fabsf(an - 1.0f) < acc_gate_g_;
  if (acc_used_) {
    ax /= an;
    ay /= an;
    az /= an;
    // Dirección estimada de "arriba" en el cuerpo (tercera fila de R).
    const float vx = 2 * (q1 * q3 - q0 * q2);
    const float vy = 2 * (q0 * q1 + q2 * q3);
    const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    // Error = medido x estimado
    const float ex = ay * vz - az * vy;
    const float ey = az * vx - ax * vz;
    const float ez = ax * vy - ay * vx;
    if (ki_ > 0) {
      ix_ += ki_ * ex * dt;
      iy_ += ki_ * ey * dt;
      iz_ += ki_ * ez * dt;
    }
    gx += kp_ * ex + ix_;
    gy += kp_ * ey + iy_;
    gz += kp_ * ez + iz_;
  } else {
    // El integrador sigue corrigiendo el sesgo estimado aunque no haya corrección.
    gx += ix_;
    gy += iy_;
    gz += iz_;
  }

  // Integración: q_dot = 0.5 * q ⊗ (0, w)
  const float hx = 0.5f * gx * dt, hy = 0.5f * gy * dt, hz = 0.5f * gz * dt;
  q[0] = q0 + (-q1 * hx - q2 * hy - q3 * hz);
  q[1] = q1 + (q0 * hx + q2 * hz - q3 * hy);
  q[2] = q2 + (q0 * hy - q1 * hz + q3 * hx);
  q[3] = q3 + (q0 * hz + q1 * hy - q2 * hx);
  const float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n > 0) {
    for (float &c : q) c /= n;
  } else {
    reset();
  }
}

float Mahony::roll() const {
  return atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])) * kRad2Deg;
}

float Mahony::pitch() const {
  float s = 2 * (q[0] * q[2] - q[3] * q[1]);
  if (s > 1) s = 1;
  if (s < -1) s = -1;
  return asinf(s) * kRad2Deg;
}

float Mahony::yaw() const {
  return atan2f(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])) * kRad2Deg;
}

float Mahony::verticalSpecificForce(float ax, float ay, float az) const {
  const float vx = 2 * (q[1] * q[3] - q[0] * q[2]);
  const float vy = 2 * (q[0] * q[1] + q[2] * q[3]);
  const float vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
  return vx * ax + vy * ay + vz * az;
}

float Mahony::axisTiltDeg(int axis, int sign) const {
  // Componente vertical (mundo) del eje del cuerpo = elemento de la tercera fila de R.
  float v;
  switch (axis) {
    case 0: v = 2 * (q[1] * q[3] - q[0] * q[2]); break;
    case 1: v = 2 * (q[0] * q[1] + q[2] * q[3]); break;
    default: v = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]; break;
  }
  v *= (sign < 0) ? -1.0f : 1.0f;
  if (v > 1) v = 1;
  if (v < -1) v = -1;
  return acosf(v) * kRad2Deg;
}

void GyroBiasCal::reset() {
  count_ = 0;
  ok_ = false;
  for (int i = 0; i < 3; i++) sum_[i] = sum2_[i] = 0;
}

bool GyroBiasCal::add(float gx, float gy, float gz) {
  if (done()) return true;
  const float g[3] = {gx, gy, gz};
  for (int i = 0; i < 3; i++) {
    sum_[i] += g[i];
    sum2_[i] += (double)g[i] * g[i];
  }
  if (++count_ < n_) return false;
  ok_ = true;
  for (int i = 0; i < 3; i++) {
    const double m = sum_[i] / n_;
    const double var = sum2_[i] / n_ - m * m;
    bias[i] = (float)m;
    if (var > (double)max_std_ * max_std_) ok_ = false;
  }
  return true;
}
