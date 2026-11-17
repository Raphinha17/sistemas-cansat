// Lector del CSV crudo de tools/flight_sim.py (--raw-csv) para las pruebas native.
// Columnas: ver RAW_COLUMNS en tools/flight_sim.py.
#pragma once

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <vector>

#ifndef SIM_DATA_DIR
#error "SIM_DATA_DIR no definido (lo define scripts/gen_sim_data.py)"
#endif

struct SimRow {
  float t;
  float acc[3];   // g
  float gyro[3];  // °/s (con sesgo)
  float p_pa, temp_c;
  int baro_new;
  float h, vz, roll, pitch, yaw;  // verdad física
  std::string fase;               // ground, ascent, hover, fall, impact
};

inline std::vector<SimRow> loadSim(const char *name) {
  std::vector<SimRow> rows;
  const std::string path = std::string(SIM_DATA_DIR) + name + ".csv";
  FILE *f = fopen(path.c_str(), "r");
  if (!f) return rows;
  char line[512];
  if (!fgets(line, sizeof line, f)) {  // encabezado
    fclose(f);
    return rows;
  }
  while (fgets(line, sizeof line, f)) {
    SimRow r;
    char fase[32] = {0};
    const int n = sscanf(line, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%31s", &r.t,
                         &r.acc[0], &r.acc[1], &r.acc[2], &r.gyro[0], &r.gyro[1], &r.gyro[2],
                         &r.p_pa, &r.temp_c, &r.baro_new, &r.h, &r.vz, &r.roll, &r.pitch,
                         &r.yaw, fase);
    if (n != 16) continue;
    r.fase = fase;
    rows.push_back(r);
  }
  fclose(f);
  return rows;
}

inline float angDiff(float a, float b) {
  float d = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
  return d;
}
