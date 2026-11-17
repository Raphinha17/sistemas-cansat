// Pruebas del Kalman vertical (h, vz), la altura barométrica y la calibración de p0,
// con datos sintéticos y con el vuelo de tools/flight_sim.py.
#include <math.h>
#include <stdio.h>
#include <unity.h>

#include "../sim_csv.h"
#include "attitude.h"
#include "estimator.h"

void setUp() {}
void tearDown() {}

static const float DT = 0.01f;

static void test_baro_altitude() {
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, baroAltitude(101325, 101325));
  // ~12 Pa por metro cerca del nivel del mar
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 100.0f, baroAltitude(101325 - 1200.5f, 101325));
  // Misma fórmula que el simulador: p = p0 (1 - h/44330)^5.255
  const float p = 101000.0f * powf(1 - 450.0f / 44330.0f, 5.255f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 450.0f, baroAltitude(p, 101000.0f));
}

static void test_predict_constant_accel() {
  VerticalKF kf;
  kf.reset(0, 0);
  for (int i = 0; i < 100; i++) kf.predict(2.0f, DT);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, kf.h());   // 0.5 a t^2
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f, kf.vz());  // a t
}

static void test_baro_only_tracks_velocity() {
  // Sin acelerómetro (IMU muerto): solo barómetro a 25 Hz, subida a 6 m/s.
  VerticalKF kf(0.5f, 0.4f);
  kf.setAccSigma(3.0f);  // como hace el firmware con IMU muerto (KF_ACC_SIGMA_SIN_IMU)
  kf.reset(0, 0);
  for (int i = 1; i <= 2000; i++) {
    kf.predict(0, DT);
    if (i % 4 == 0) kf.updateBaro(6.0f * i * DT);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 6.0f, kf.vz());
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 120.0f, kf.h());
}

static void test_base_cal() {
  BaseCal b(30, 25.0f);
  TEST_ASSERT_FALSE(b.hasValue());
  // 29 s: aún no lista
  for (int i = 0; i < 29 * 25; i++) b.add(100000.0f + ((i % 2) ? 2.0f : -2.0f));
  TEST_ASSERT_TRUE(b.hasValue());
  TEST_ASSERT_FALSE(b.ready());
  for (int i = 0; i < 25; i++) b.add(100000.0f);
  TEST_ASSERT_TRUE(b.ready());
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 100000.0f, b.p0());
  // Ventana móvil: tras 30 s a otra presión, p0 la sigue por completo
  for (int i = 0; i < 30 * 25; i++) b.add(100100.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 100100.0f, b.p0());
  // Congelada: ya no cambia
  b.freeze();
  for (int i = 0; i < 100; i++) b.add(90000.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 100100.0f, b.p0());
  // Recarga desde NVS
  BaseCal c(30, 25.0f);
  c.setFrozen(98765.0f);
  TEST_ASSERT_TRUE(c.ready());
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 98765.0f, c.p0());
}

// Mismo flujo que el lazo de control del firmware (src/flight/main.cpp).
static void run_flight(const char *name) {
  const std::vector<SimRow> rows = loadSim(name);
  TEST_ASSERT_TRUE_MESSAGE(rows.size() > 10000, "no se pudo leer el CSV del simulador");

  GyroBiasCal cal(200, 0.5f);
  Mahony m(2.0f, 0.02f, 0.15f);
  VerticalKF kf(0.5f, 0.4f);
  BaseCal base(30, 25.0f);
  bool gyro_ready = false, kf_init = false;

  // Errores por fase: 0 tierra/ascenso, 1 caída libre, 2 descenso con autogiro
  double se_h[3] = {0}, se_v[3] = {0};
  float max_ev[3] = {0}, max_eh[3] = {0};
  int n[3] = {0};
  bool terminal_seen = false, descent = false;
  float h_max_est = -1e9f, h_max_true = -1e9f;
  float h_est_at_200 = NAN;

  for (const SimRow &r : rows) {
    if (!gyro_ready) {
      if (cal.add(r.gyro[0], r.gyro[1], r.gyro[2])) {
        m.initFromAccel(r.acc[0], r.acc[1], r.acc[2]);
        gyro_ready = true;
      }
    } else {
      m.update(r.gyro[0] - cal.bias[0], r.gyro[1] - cal.bias[1], r.gyro[2] - cal.bias[2],
               r.acc[0], r.acc[1], r.acc[2], DT);
    }
    const float a_up =
        gyro_ready ? (m.verticalSpecificForce(r.acc[0], r.acc[1], r.acc[2]) - 1.0f) * kG : 0;
    if (kf_init) kf.predict(a_up, DT);
    // El firmware congela p0 al despegar (máquina de estados futura); aquí, al iniciar
    // el ascenso según la verdad del simulador.
    if (r.fase != "ground") base.freeze();
    if (r.baro_new) {
      base.add(r.p_pa);
      const float hb = baroAltitude(r.p_pa, base.p0());
      if (!kf_init) {
        kf.reset(hb, 0);
        kf_init = true;
      } else {
        kf.updateBaro(hb);
      }
    }
    if (!kf_init) continue;

    int k;
    if (r.fase == "ground" || r.fase == "ascent" || r.fase == "hover") {
      k = 0;
      if (r.t < 5.0f) continue;  // tiempo de asentamiento inicial
    } else if (r.fase == "fall" && !descent) {
      if (r.vz < -30.0f) terminal_seen = true;
      if (terminal_seen && r.vz > -12.0f) descent = true;
      k = 1;
      if (isnan(h_est_at_200) && r.h <= 200.0f) h_est_at_200 = kf.h();
    } else if (r.fase == "fall") {
      k = 2;
    } else {
      continue;  // impacto: transitorio de 8 g, no se evalúa
    }
    const float eh = fabsf(kf.h() - r.h), ev = fabsf(kf.vz() - r.vz);
    se_h[k] += eh * eh;
    se_v[k] += ev * ev;
    if (eh > max_eh[k]) max_eh[k] = eh;
    if (ev > max_ev[k]) max_ev[k] = ev;
    n[k]++;
    if (kf.h() > h_max_est) h_max_est = kf.h();
    if (r.h > h_max_true) h_max_true = r.h;
  }

  char msg[220];
  for (int k = 0; k < 3; k++) {
    snprintf(msg, sizeof msg,
             "%s fase %d: h rms=%.2f max=%.2f m | vz rms=%.2f max=%.2f m/s (n=%d)", name, k,
             sqrtf(se_h[k] / n[k]), max_eh[k], sqrtf(se_v[k] / n[k]), max_ev[k], n[k]);
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE(n[k] > 100);
  }
  snprintf(msg, sizeof msg, "%s h_max est=%.2f real=%.2f; h_est al cruzar 200 m=%.2f", name,
           h_max_est, h_max_true, h_est_at_200);
  TEST_MESSAGE(msg);

  // Tolerancias: ver docs/PREGUNTAS.md (P10)
  TEST_ASSERT_FLOAT_WITHIN(2.0f, h_max_true, h_max_est);
  TEST_ASSERT_FLOAT_WITHIN(3.0f, 200.0f, h_est_at_200);
  for (int k = 0; k < 3; k++) {
    TEST_ASSERT_TRUE(sqrtf(se_h[k] / n[k]) < 1.5f);
    TEST_ASSERT_TRUE(sqrtf(se_v[k] / n[k]) < 1.0f);
  }
  TEST_ASSERT_TRUE(max_ev[0] < 1.5f);
  TEST_ASSERT_TRUE(max_ev[1] < 3.0f);
}

static void test_flight_seed1() { run_flight("flight_seed1"); }
static void test_flight_seed7() { run_flight("flight_seed7"); }

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_baro_altitude);
  RUN_TEST(test_predict_constant_accel);
  RUN_TEST(test_baro_only_tracks_velocity);
  RUN_TEST(test_base_cal);
  RUN_TEST(test_flight_seed1);
  RUN_TEST(test_flight_seed7);
  return UNITY_END();
}
