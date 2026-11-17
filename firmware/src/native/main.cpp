// Programa de PC (entorno native): reproduce un CSV crudo de tools/flight_sim.py
// (--raw-csv) a través de Mahony + Kalman y escribe un CSV con estimación y verdad,
// útil para ajustar parámetros.
//
//   pio run -e native
//   .pio/build/native/program <entrada.csv> > salida.csv
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "attitude.h"
#include "config.h"
#include "estimator.h"

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "uso: %s <raw.csv de flight_sim.py>\n", argv[0]);
    return 2;
  }
  FILE *f = fopen(argv[1], "r");
  if (!f) {
    fprintf(stderr, "no se pudo abrir %s\n", argv[1]);
    return 1;
  }
  const float dt = 1.0f / CONTROL_HZ;
  GyroBiasCal cal(GYRO_CAL_SAMPLES, GYRO_CAL_MAX_STD_DPS);
  Mahony m(MAHONY_KP, MAHONY_KI, MAHONY_ACC_GATE_G);
  VerticalKF kf(KF_ACC_SIGMA, KF_BARO_SIGMA);
  BaseCal base(P0_WINDOW_S, (float)CONTROL_HZ / BARO_DECIM);
  bool gyro_ready = false, kf_init = false;

  char line[512];
  fgets(line, sizeof line, f);  // encabezado
  printf("t,h_est,vz_est,roll_est,pitch_est,yaw_est,h_true,vz_true,roll_true,pitch_true,fase\n");
  while (fgets(line, sizeof line, f)) {
    float t, a[3], g[3], p, temp, h, vz, r, pi, y;
    int bn;
    char fase[32];
    if (sscanf(line, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%31s", &t, &a[0], &a[1],
               &a[2], &g[0], &g[1], &g[2], &p, &temp, &bn, &h, &vz, &r, &pi, &y, fase) != 16)
      continue;
    if (!gyro_ready) {
      if (cal.add(g[0], g[1], g[2])) {
        m.initFromAccel(a[0], a[1], a[2]);
        gyro_ready = true;
      }
    } else {
      m.update(g[0] - cal.bias[0], g[1] - cal.bias[1], g[2] - cal.bias[2], a[0], a[1], a[2],
               dt);
    }
    const float a_up = gyro_ready ? (m.verticalSpecificForce(a[0], a[1], a[2]) - 1.0f) * kG : 0;
    if (kf_init) kf.predict(a_up, dt);
    if (strcmp(fase, "ground") != 0) base.freeze();
    if (bn) {
      base.add(p);
      const float hb = baroAltitude(p, base.p0());
      if (!kf_init) {
        kf.reset(hb, 0);
        kf_init = true;
      } else {
        kf.updateBaro(hb);
      }
    }
    printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s\n", t, kf.h(), kf.vz(),
           m.roll(), m.pitch(), m.yaw(), h, vz, r, pi, fase);
  }
  fclose(f);
  return 0;
}
