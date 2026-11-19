#include "sim_input.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint16_t simCrc16(const char *s, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= (uint16_t)((uint8_t)s[i]) << 8;
    for (int b = 0; b < 8; b++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

bool simAppendCrc(char *line, size_t cap) {
  const size_t n = strlen(line);
  if (n + 6 > cap) return false;  // "*HHHH" + terminador
  snprintf(line + n, cap - n, "*%04X", (unsigned)simCrc16(line, n));
  return true;
}

static int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Lee un campo numérico hasta la siguiente coma. Vacío o no numérico -> NaN.
static const char *field(const char *p, float &v) {
  const char *end = strchr(p, ',');
  if (!end) end = p + strlen(p);
  char buf[24];
  size_t n = (size_t)(end - p);
  if (n >= sizeof buf) n = sizeof buf - 1;
  memcpy(buf, p, n);
  buf[n] = 0;
  // recorta espacios y fin de línea
  while (n && (buf[n - 1] == ' ' || buf[n - 1] == '\r' || buf[n - 1] == '\n')) buf[--n] = 0;
  char *q = buf;
  while (*q == ' ') q++;
  char *stop = nullptr;
  v = *q ? strtof(q, &stop) : NAN;
  if (*q && (stop == q || *stop)) v = NAN;  // texto no numérico
  return *end == ',' ? end + 1 : nullptr;
}

bool parseSimLine(const char *line, CoreInput &out, bool *has_crc) {
  if (has_crc) *has_crc = false;
  if (strncmp(line, "SIM,", 4) != 0) return false;

  // CRC opcional: "*HHHH" al final (después solo puede venir el fin de línea).
  size_t body_len = strlen(line);
  bool crc = false;
  if (const char *star = strchr(line, '*')) {
    unsigned v = 0;
    for (int i = 1; i <= 4; i++) {
      const int d = hexVal(star[i]);  // se detiene en el terminador (hexVal(0) = -1)
      if (d < 0) return false;
      v = v * 16 + (unsigned)d;
    }
    for (const char *q = star + 5; *q; q++)
      if (*q != '\r' && *q != '\n' && *q != ' ') return false;
    body_len = (size_t)(star - line);
    if (simCrc16(line, body_len) != v) return false;
    crc = true;
  }
  if (body_len > SIM_LINEA_MAX) return false;
  char body[SIM_LINEA_MAX + 1];
  memcpy(body, line, body_len);
  body[body_len] = 0;

  const char *p = body + 4;
  float v[9];
  int n = 0;
  while (p && n < 9) p = field(p, v[n++]);
  if (n != 9 || p != nullptr) return false;  // exactamente 9 campos
  if (isnan(v[0]) || v[0] < 0) return false;

  CoreInput c;
  c.t_ms = (uint32_t)lroundf(v[0]);
  c.imu_valid = true;
  for (int i = 0; i < 3; i++) {
    c.acc_g[i] = v[1 + i];
    c.gyro_dps[i] = v[4 + i];
    if (isnan(c.acc_g[i]) || isnan(c.gyro_dps[i])) c.imu_valid = false;
  }
  if (!c.imu_valid)
    for (int i = 0; i < 3; i++) c.acc_g[i] = c.gyro_dps[i] = 0;
  c.baro_valid = !isnan(v[7]);
  c.p_pa = c.baro_valid ? v[7] : 0;
  c.temp_c = isnan(v[8]) ? 0 : v[8];
  out = c;
  if (has_crc) *has_crc = crc;
  return true;
}

// ---------------------------------------------------------------------------
// SimLink
// ---------------------------------------------------------------------------
void SimLink::reset() { *this = SimLink(); }

SimLink::Result SimLink::feed(const char *line, uint32_t now_ms, CoreInput &out) {
  CoreInput in;
  bool crc = false;
  if (!parseSimLine(line, in, &crc) || (strict_ && !crc)) {
    reject(now_ms);
    return INVALIDA;
  }
  if (crc) strict_ = true;
  if (has_next_ && in.t_ms != next_) {
    n_gap_ += (int32_t)(in.t_ms - next_) > 0 ? 1 : 0;
    if (strict_) {
      if ((int32_t)(in.t_ms - next_) < 0) {
        n_dup_++;  // ya procesada (llega otra vez tras un reenvío)
        return REPETIDA;
      }
      requestResend(now_ms);  // faltan líneas antes de esta
      return ADELANTADA;
    }
    // Modo compatible: se acepta como antes (solo queda contado).
  }
  has_next_ = true;
  next_ = in.t_ms + SIM_PERIODO_MS;
  n_ok_++;
  out = in;
  return ACEPTADA;
}

void SimLink::reject(uint32_t now_ms) {
  n_bad_++;
  requestResend(now_ms);
}

void SimLink::requestResend(uint32_t now_ms) {
  if (!strict_ || !has_next_) return;
  // Un pedido por línea esperada; se repite si el reenvío no llega en SIM_REPITE_MS.
  if (asked_ && asked_t_ == next_ && (uint32_t)(now_ms - asked_ms_) < SIM_REPITE_MS) return;
  asked_ = true;
  asked_t_ = next_;
  asked_ms_ = now_ms;
  pending_ = true;
  n_resend_++;
}

bool SimLink::takeResend(uint32_t &t_ms) {
  if (!pending_) return false;
  pending_ = false;
  t_ms = asked_t_;
  return true;
}
