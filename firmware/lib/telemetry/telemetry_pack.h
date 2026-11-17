// Armado del paquete de telemetría (packet.h) a partir de unidades físicas.
// Portable (sin Arduino). Mismas escalas y saturación que protocol/telemetry.py.
#pragma once

#include <stdint.h>

#include "packet.h"

struct TelemetryValues {
  uint8_t estado = ST_AUTOPRUEBA;
  uint16_t secuencia = 0;
  uint32_t t_ms = 0;
  float presion_pa = 0;
  float temp_c = 0;
  float altura_m = 0;
  float vz_ms = 0;
  float acc_g[3] = {0, 0, 0};
  float roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
  uint8_t banderas = 0;
  float bateria_v = 0;
};

TelemetryPacket packTelemetry(const TelemetryValues &v);
