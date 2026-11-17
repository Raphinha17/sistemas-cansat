#include "telemetry_pack.h"

#include <math.h>

// Redondeo al entero más cercano con saturación al rango de destino.
static int32_t satRound(float v, int32_t lo, int32_t hi) {
  if (!(v == v)) return 0;  // NaN
  const float r = roundf(v);
  if (r <= (float)lo) return lo;
  if (r >= (float)hi) return hi;
  return (int32_t)r;
}

static int16_t s16(float v) { return (int16_t)satRound(v, -32768, 32767); }

TelemetryPacket packTelemetry(const TelemetryValues &v) {
  TelemetryPacket p;
  p.version = PACKET_VERSION;
  p.estado = v.estado;
  p.secuencia = v.secuencia;
  p.t_ms = v.t_ms;
  const float pr = roundf(v.presion_pa);
  p.presion_Pa = pr <= 0 ? 0u : (pr >= 4294967040.0f ? 0xFFFFFFFFu : (uint32_t)pr);
  p.temp_centi = s16(v.temp_c * 100.0f);
  p.altura_dm = s16(v.altura_m * 10.0f);
  p.vz_cms = s16(v.vz_ms * 100.0f);
  p.ax_mg = s16(v.acc_g[0] * 1000.0f);
  p.ay_mg = s16(v.acc_g[1] * 1000.0f);
  p.az_mg = s16(v.acc_g[2] * 1000.0f);
  p.roll_centi = s16(v.roll_deg * 100.0f);
  p.pitch_centi = s16(v.pitch_deg * 100.0f);
  p.yaw_centi = s16(v.yaw_deg * 100.0f);
  p.banderas = v.banderas;
  p.bateria_dv = (uint8_t)satRound(v.bateria_v * 10.0f, 0, 255);
  return p;
}
