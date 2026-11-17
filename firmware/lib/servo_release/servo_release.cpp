#include "servo_release.h"

float servoPulseUs(float angle_deg) {
  if (angle_deg > 90.0f) angle_deg = 90.0f;
  if (angle_deg < -90.0f) angle_deg = -90.0f;
  return SERVO_US_CENTRO + angle_deg * SERVO_US_POR_GRADO;
}

void ServoSequencer::start(uint32_t t_ms) {
  // El primer intento va directo a -80°: en CAIDA el servo ya está alimentado en -10°,
  // y si no lo estaba (p. ej. tras un reinicio) basta con alimentarlo ya en -80°.
  phase_ = kLibera;
  t_phase_ = t_ms;
  attempt_ = 1;
  confirmed_ = false;
}

void ServoSequencer::step(uint32_t t_ms, bool confirm) {
  const float el = (uint32_t)(t_ms - t_phase_) * 0.001f;
  switch (phase_) {
    case kRetorno:
      if (el >= SERVO_T_RETORNO_S) {
        phase_ = kLibera;
        t_phase_ = t_ms;
      }
      break;
    case kLibera:
      if (confirm) {
        confirmed_ = true;
        phase_ = kFin;
      } else if (el >= SERVO_CONFIRMA_T_S) {
        if (attempt_ <= SERVO_REINTENTOS) {
          attempt_++;
          phase_ = kRetorno;
          t_phase_ = t_ms;
        } else {
          phase_ = kFin;  // sin confirmación tras todos los intentos
        }
      }
      break;
    default:
      break;
  }
}

ServoCmd ServoSequencer::cmd() const {
  ServoCmd c;
  switch (phase_) {
    case kRetorno:
      c.power = true;
      c.angle_deg = SERVO_ANG_BLOQUEO_DEG;
      break;
    case kLibera:
      c.power = true;
      c.angle_deg = SERVO_ANG_LIBERA_DEG;
      break;
    case kFin:
      c.power = false;  // terminado: sin energía (queda donde quedó)
      c.angle_deg = SERVO_ANG_LIBERA_DEG;
      break;
    default:
      break;
  }
  return c;
}
