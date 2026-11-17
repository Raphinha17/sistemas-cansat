#include "sim_input.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

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

bool parseSimLine(const char *line, CoreInput &out) {
  if (strncmp(line, "SIM,", 4) != 0) return false;
  const char *p = line + 4;
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
  return true;
}
