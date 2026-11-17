// Pruebas del filtro Mahony y de la calibración del giroscopio, con datos sintéticos y
// con el vuelo de tools/flight_sim.py.
#include <math.h>
#include <stdio.h>
#include <unity.h>

#include "../sim_csv.h"
#include "attitude.h"

void setUp() {}
void tearDown() {}

static const float DT = 0.01f;

static void test_init_from_accel() {
  // roll 20°, pitch -10°: g_cuerpo = [-sin p, sin r cos p, cos r cos p]
  const float r = 20 * M_PI / 180, p = -10 * M_PI / 180;
  Mahony m;
  m.initFromAccel(-sinf(p), sinf(r) * cosf(p), cosf(r) * cosf(p));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, m.roll());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -10.0f, m.pitch());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.yaw());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, acosf(cosf(r) * cosf(p)) * 180 / M_PI, m.axisTiltDeg(2, 1));
}

static void test_gyro_integration_yaw() {
  // 90 °/s en z durante 1 s sin corrección (acelerómetro en 0 g) -> yaw 90°
  Mahony m;
  for (int i = 0; i < 100; i++) m.update(0, 0, 90, 0, 0, 0, DT);
  TEST_ASSERT_FALSE(m.accelUsed());
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 90.0f, m.yaw());
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, m.roll());
}

static void test_converges_from_wrong_init() {
  // Arranca nivelado pero el cuerpo está a roll 30°: debe converger con el acelerómetro.
  const float r = 30 * M_PI / 180;
  Mahony m;
  for (int i = 0; i < 500; i++) m.update(0, 0, 0, 0, sinf(r), cosf(r), DT);
  TEST_ASSERT_TRUE(m.accelUsed());
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, m.roll());
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, m.pitch());
  // Inclinación del eje +z respecto a la vertical = 30°; el eje -z queda a 150°.
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, m.axisTiltDeg(2, 1));
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 150.0f, m.axisTiltDeg(2, -1));
}

static void test_gyro_bias_cal() {
  GyroBiasCal c(200, 0.5f);
  for (int i = 0; i < 199; i++) TEST_ASSERT_FALSE(c.add(1.0f, -2.0f, 0.5f + 0.1f * (i % 2)));
  TEST_ASSERT_TRUE(c.add(1.0f, -2.0f, 0.55f));
  TEST_ASSERT_TRUE(c.ok());
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, c.bias[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -2.0f, c.bias[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 0.55f, c.bias[2]);
  // Con movimiento (oscilación de ±20 °/s) se rechaza
  GyroBiasCal m(200, 0.5f);
  for (int i = 0; i < 200; i++) m.add(20.0f * sinf(i * 0.1f), 0, 0);
  TEST_ASSERT_TRUE(m.done());
  TEST_ASSERT_FALSE(m.ok());
}

// Corre el filtro sobre un vuelo simulado y verifica errores por fase.
static void run_flight(const char *name) {
  const std::vector<SimRow> rows = loadSim(name);
  TEST_ASSERT_TRUE_MESSAGE(rows.size() > 10000, "no se pudo leer el CSV del simulador");

  // 1) Sesgo del giroscopio con las primeras 2 s (en tierra, en reposo)
  GyroBiasCal cal(200, 0.5f);
  size_t i = 0;
  while (!cal.add(rows[i].gyro[0], rows[i].gyro[1], rows[i].gyro[2])) i++;
  TEST_ASSERT_TRUE(cal.ok());

  Mahony m(2.0f, 0.02f, 0.15f);
  m.initFromAccel(rows[i].acc[0], rows[i].acc[1], rows[i].acc[2]);

  double se_rp[3] = {0, 0, 0};  // tierra/ascenso, caída libre, descenso
  int n_rp[3] = {0, 0, 0};
  float max_err[3] = {0, 0, 0};
  float max_tilt_err_fall = 0;
  bool terminal_seen = false, descent = false;
  for (i = i + 1; i < rows.size(); i++) {
    const SimRow &r = rows[i];
    m.update(r.gyro[0] - cal.bias[0], r.gyro[1] - cal.bias[1], r.gyro[2] - cal.bias[2],
             r.acc[0], r.acc[1], r.acc[2], DT);
    const float er = fabsf(angDiff(m.roll(), r.roll));
    const float ep = fabsf(angDiff(m.pitch(), r.pitch));
    const float e = er > ep ? er : ep;
    int k;
    if (r.fase == "ground" || r.fase == "ascent" || r.fase == "hover") {
      k = 0;
    } else if (r.fase == "fall" && !descent) {
      // Caída libre hasta que el autogiro frena a < 12 m/s
      if (r.vz < -30.0f) terminal_seen = true;
      if (terminal_seen && r.vz > -12.0f) descent = true;
      k = 1;
      const float tilt_true =
          acosf(cosf(r.roll * M_PI / 180) * cosf(r.pitch * M_PI / 180)) * 180 / M_PI;
      const float te = fabsf(m.axisTiltDeg(2, 1) - tilt_true);
      if (te > max_tilt_err_fall) max_tilt_err_fall = te;
    } else if (r.fase == "fall") {
      k = 2;  // descenso con autogiro
    } else {
      continue;  // impacto
    }
    se_rp[k] += e * e;
    n_rp[k]++;
    if (e > max_err[k]) max_err[k] = e;
  }
  char msg[200];
  for (int k = 0; k < 3; k++) {
    const float rms = sqrtf(se_rp[k] / (n_rp[k] ? n_rp[k] : 1));
    snprintf(msg, sizeof msg, "%s fase %d: rms=%.2f max=%.2f grados (n=%d)", name, k, rms,
             max_err[k], n_rp[k]);
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE(n_rp[k] > 100);
  }
  snprintf(msg, sizeof msg, "%s error max inclinacion en caida: %.2f", name, max_tilt_err_fall);
  TEST_MESSAGE(msg);

  // Tolerancias: ver docs/PREGUNTAS.md (P10)
  TEST_ASSERT_TRUE(sqrtf(se_rp[0] / n_rp[0]) < 1.5f);
  TEST_ASSERT_TRUE(max_err[0] < 5.0f);
  TEST_ASSERT_TRUE(max_err[1] < 5.0f);       // compuerta de 45°: sobra margen
  TEST_ASSERT_TRUE(max_tilt_err_fall < 5.0f);
  TEST_ASSERT_TRUE(sqrtf(se_rp[2] / n_rp[2]) < 3.0f);
}

static void test_flight_seed1() { run_flight("flight_seed1"); }
static void test_flight_seed7() { run_flight("flight_seed7"); }

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_init_from_accel);
  RUN_TEST(test_gyro_integration_yaw);
  RUN_TEST(test_converges_from_wrong_init);
  RUN_TEST(test_gyro_bias_cal);
  RUN_TEST(test_flight_seed1);
  RUN_TEST(test_flight_seed7);
  return UNITY_END();
}
