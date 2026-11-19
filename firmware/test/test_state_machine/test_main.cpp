// Pruebas unitarias de la máquina de estados, la secuencia del servo y el modo demo
// con entradas sintéticas (casos de borde que el simulador no produce con precisión).
// Los escenarios de vuelo completos están en test_flight.
#include <math.h>
#include <unity.h>

#include "config.h"
#include "servo_release.h"
#include "state_machine.h"

void setUp() {}
void tearDown() {}

// Entrada base: sensores bien, calibrado, en reposo en el suelo.
static SmInput base(uint32_t t) {
  SmInput in;
  in.t_ms = t;
  in.calib_done = true;
  in.base_ready = true;
  in.p0_pa = 100000;
  in.h = 0;
  in.vz = 0;
  in.tilt_deg = 5;
  in.acc_g = 1;
  return in;
}

// Sube a 6 m/s desde 30 m hasta h_top y suelta: baja g y vz cayendo durante hasta 2 s.
// Devuelve el tiempo actual. *h_out: altura final.
static uint32_t climbAndDrop(StateMachine &sm, float h_top, float *h_out = nullptr) {
  uint32_t t = 0;
  sm.begin(t);
  for (; t < 2000; t += 10) sm.step(base(t));  // 0 -> 1
  TEST_ASSERT_EQUAL(ST_TIERRA, sm.state());
  float h = 30;
  for (; h < h_top; t += 10, h += 0.06f) {
    SmInput in = base(t);
    in.h = h;
    in.vz = 6;
    sm.step(in);
  }
  TEST_ASSERT_EQUAL(ST_ASCENSO, sm.state());
  float vz = 0;
  for (int i = 0; i < 200 && sm.state() == ST_ASCENSO; i++, t += 10) {
    SmInput in = base(t);
    vz -= 0.098f;
    h += vz * 0.01f;
    in.h = h;
    in.vz = vz;
    in.acc_g = 0.05f;
    sm.step(in);
  }
  if (h_out) *h_out = h;
  return t;
}

// Lleva la máquina hasta CAIDA con una liberación a 450 m. Devuelve el tiempo actual.
static uint32_t toCaida(StateMachine &sm) {
  uint32_t t = 0;
  sm.begin(t);
  for (; t < 2000; t += 10) sm.step(base(t));  // 0 -> 1
  TEST_ASSERT_EQUAL(ST_TIERRA, sm.state());
  // Ascenso a 6 m/s desde 30 m
  float h = 30;
  for (; h < 450; t += 10, h += 0.06f) {
    SmInput in = base(t);
    in.h = h;
    in.vz = 6;
    sm.step(in);
  }
  TEST_ASSERT_EQUAL(ST_ASCENSO, sm.state());
  // Liberación: baja g y vz cayendo
  float vz = 0;
  for (int i = 0; i < 200 && sm.state() == ST_ASCENSO; i++, t += 10) {
    SmInput in = base(t);
    vz -= 0.098f;
    h += vz * 0.01f;
    in.h = h;
    in.vz = vz;
    in.acc_g = 0.05f;
    sm.step(in);
  }
  TEST_ASSERT_EQUAL(ST_CAIDA, sm.state());
  return t;
}

static void test_servo_sin_energia_hasta_caida() {
  StateMachine sm;
  sm.begin(0);
  sm.step(base(0));
  TEST_ASSERT_FALSE(sm.out().servo.power);  // AUTOPRUEBA
  uint32_t t = toCaida(sm);
  SmInput in = base(t);
  in.h = 400;
  in.vz = -20;
  in.acc_g = 0.5f;
  sm.step(in);
  TEST_ASSERT_TRUE(sm.out().servo.power);  // CAIDA: alimentado en bloqueo
  TEST_ASSERT_EQUAL_FLOAT(SERVO_ANG_BLOQUEO_DEG, sm.out().servo.angle_deg);
  TEST_ASSERT_EQUAL(1000 / TX_HZ_CAIDA, sm.out().tx_period_ms);
}

static void test_despegue_congela_p0_quieto() {
  StateMachine sm;
  sm.begin(0);
  uint32_t t = 0;
  for (; t < 1000; t += 10) sm.step(base(t));
  // El promedio de p0 empieza a bajar con la subida: se congela el valor "quieto".
  float h = 0;
  bool froze = false;
  float p0_frozen = 0;
  for (; t < 20000 && !froze; t += 10) {
    SmInput in = base(t);
    h += 0.06f;
    in.h = h;
    in.vz = 6;
    in.p0_pa = 100000 - h * 0.5f;  // p0 "contaminado" por la subida
    sm.step(in);
    if (sm.out().ev_freeze_p0) {
      froze = true;
      p0_frozen = sm.out().p0_freeze_pa;
    }
  }
  TEST_ASSERT_TRUE(froze);
  TEST_ASSERT_EQUAL(ST_ASCENSO, sm.state());
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 100000.0f, p0_frozen);
  // 20 m + 3 s a 6 m/s
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 20.0f + 3.0f * 6.0f, h);
  TEST_ASSERT_EQUAL(1, sm.record().mission);
  TEST_ASSERT_EQUAL_FLOAT(p0_frozen, sm.record().p0_pa);
}

static void test_despegue_requiere_3s() {
  StateMachine sm;
  sm.begin(0);
  uint32_t t = 0;
  for (; t < 1000; t += 10) sm.step(base(t));
  // 2.5 s cumpliendo, una muestra que no cumple, y otra vez 2.5 s: no despega
  for (int rep = 0; rep < 2; rep++) {
    for (int i = 0; i < 250; i++, t += 10) {
      SmInput in = base(t);
      in.h = 25;
      in.vz = 2;
      sm.step(in);
    }
    SmInput in = base(t);
    in.h = 25;
    in.vz = 0.5f;
    sm.step(in);
    t += 10;
  }
  TEST_ASSERT_EQUAL(ST_TIERRA, sm.state());
}

static void test_compuerta_espera_y_dispara() {
  StateMachine sm;
  uint32_t t = toCaida(sm);
  // Bajo h_disp con 60°: espera; a los 0.8 s el ángulo baja a 30°: dispara sin respaldo.
  float h = 215;
  uint32_t t_cross = 0;
  for (int i = 0; i < 300 && sm.state() == ST_CAIDA; i++, t += 10) {
    SmInput in = base(t);
    in.h = h;
    in.vz = -35;
    in.acc_g = 1.0f;
    in.tilt_deg = (t_cross && t - t_cross >= 800) ? 30 : 60;
    sm.step(in);
    if (!t_cross && sm.tHdisp()) t_cross = t;
    h -= 0.35f;
  }
  TEST_ASSERT_EQUAL(ST_DESPLIEGUE, sm.state());
  TEST_ASSERT_EQUAL_UINT32(t_cross + 800, sm.tFire());
  TEST_ASSERT_EQUAL(FLAG_DISPARO, sm.out().flags & (FLAG_DISPARO | FLAG_DISPARO_RESPALDO));
}

static void test_respaldo_160m_sintetico() {
  StateMachine sm;
  uint32_t t = toCaida(sm);
  // Caída a 60 m/s siempre inclinado: llega a 160 m antes de 1.5 s
  float h = 240;
  for (int i = 0; i < 300 && sm.state() == ST_CAIDA; i++, t += 10) {
    SmInput in = base(t);
    in.h = h;
    in.vz = -60;
    in.tilt_deg = 80;
    sm.step(in);
    h -= 0.6f;
  }
  TEST_ASSERT_EQUAL(ST_DESPLIEGUE, sm.state());
  TEST_ASSERT_TRUE(sm.out().flags & FLAG_DISPARO_RESPALDO);
  TEST_ASSERT_TRUE((t - 10) - sm.tHdisp() < 1500);
}

static void test_sin_imu_no_hay_compuerta() {
  StateMachine sm;
  uint32_t t = toCaida(sm);
  SmInput in = base(t);
  in.imu_ok = false;
  in.h = 210;
  in.vz = -30;
  in.tilt_deg = 90;  // ignorado
  sm.step(in);
  TEST_ASSERT_EQUAL(ST_DESPLIEGUE, sm.state());
  TEST_ASSERT_EQUAL(0, sm.out().flags & FLAG_DISPARO_RESPALDO);
  TEST_ASSERT_TRUE(sm.out().flags & FLAG_FALLO_IMU);
}

static void test_no_go() {
  StateMachine sm;
  sm.begin(0);
  SmInput in = base(0);
  in.imu_ok = in.baro_ok = false;
  sm.step(in);
  TEST_ASSERT_TRUE(sm.out().flags & FLAG_NO_GO);
}

static void test_reinicio_decide_por_vz() {
  MissionRecord rec;
  rec.mission = 1;
  rec.state = ST_ASCENSO;
  rec.p0_pa = 100000;
  rec.p0_valid = 1;
  rec.h_max_m = 440;
  // Reinició en ASCENSO pero ya cae a -25 m/s: pasa a CAIDA tras la espera.
  StateMachine sm;
  sm.beginRecovered(rec, 5000);
  uint32_t t = 5000;
  for (; t < 5000 + 900; t += 10) {
    SmInput in = base(t);
    in.h = 400;
    in.vz = -25;
    sm.step(in);
    TEST_ASSERT_EQUAL(ST_ASCENSO, sm.state());
    TEST_ASSERT_TRUE(sm.out().recuperando);
  }
  for (; t < 5000 + 1100; t += 10) {
    SmInput in = base(t);
    in.h = 395;
    in.vz = -25;
    sm.step(in);
  }
  TEST_ASSERT_EQUAL(ST_CAIDA, sm.state());
  TEST_ASSERT_TRUE(sm.out().flags & FLAG_LIBERACION);
  // Y si sube (cerca del máximo guardado), sigue en ASCENSO
  StateMachine sm2;
  sm2.beginRecovered(rec, 0);
  for (t = 0; t < 1100; t += 10) {
    SmInput in = base(t);
    in.h = 430;
    in.vz = 5;
    sm2.step(in);
  }
  TEST_ASSERT_EQUAL(ST_ASCENSO, sm2.state());
  TEST_ASSERT_FALSE(sm2.out().recuperando);
}

static void test_aterrizado_tasas_y_mision() {
  StateMachine sm;
  uint32_t t = toCaida(sm);
  SmInput in = base(t);
  in.h = 210;
  in.vz = -30;
  sm.step(in);  // dispara (compuerta 5°)
  TEST_ASSERT_EQUAL(ST_DESPLIEGUE, sm.state());
  // Confirmación por pico de aceleración
  t += 10;
  in = base(t);
  in.acc_g = 5;
  in.h = 200;
  in.vz = -20;
  sm.step(in);
  TEST_ASSERT_EQUAL(ST_DESCENSO, sm.state());
  TEST_ASSERT_TRUE(sm.out().servo_confirmado);
  TEST_ASSERT_FALSE(sm.out().servo.power);
  // Descenso a 10 m/s 5 s, impacto y reposo
  for (int i = 0; i < 500; i++) {
    t += 10;
    in = base(t);
    in.h = 100;
    in.vz = -10;
    in.rate_dps = 90;
    sm.step(in);
  }
  TEST_ASSERT_FALSE(sm.out().vz_fuera_rango);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -10.0f, sm.out().vz_prom);
  t += 10;
  in = base(t);
  in.acc_g = 8;
  sm.step(in);
  for (int i = 0; i < 110; i++) {
    t += 10;
    sm.step(base(t));
  }
  TEST_ASSERT_EQUAL(ST_ATERRIZADO, sm.state());
  TEST_ASSERT_EQUAL(0, sm.record().mission);
  TEST_ASSERT_EQUAL(1000 / TX_HZ_ATERRIZADO, sm.out().tx_period_ms);
  for (int i = 0; i < SM_ATERRIZADO_TX_S * 100 + 10; i++) {
    t += 10;
    sm.step(base(t));
  }
  TEST_ASSERT_EQUAL(SM_BALIZA_PERIODO_MS, sm.out().tx_period_ms);
}

static void test_descenso_fuera_de_rango() {
  StateMachine sm;
  uint32_t t = toCaida(sm);
  SmInput in = base(t);
  in.h = 210;
  in.vz = -30;
  sm.step(in);
  // Sin confirmación: la secuencia dura 1 + 2 * (0.3 + 1) s
  for (int i = 0; i < 400 && sm.state() == ST_DESPLIEGUE; i++) {
    t += 10;
    in = base(t);
    in.h = 150;
    in.vz = -15;
    sm.step(in);
  }
  TEST_ASSERT_EQUAL(ST_DESCENSO, sm.state());
  TEST_ASSERT_FALSE(sm.out().servo_confirmado);
  TEST_ASSERT_EQUAL(1 + SERVO_REINTENTOS, sm.out().servo_intentos);
  for (int i = 0; i < 150; i++) {
    t += 10;
    in = base(t);
    in.h = 120;
    in.vz = -15;  // demasiado rápido
    in.rate_dps = 90;
    sm.step(in);
  }
  TEST_ASSERT_TRUE(sm.out().vz_fuera_rango);
}

// ---------------------------------------------------------------------------
// Secuencia del servo
// ---------------------------------------------------------------------------
static void test_servo_reintentos() {
  ServoSequencer s;
  s.start(0);
  TEST_ASSERT_TRUE(s.cmd().power);
  TEST_ASSERT_EQUAL_FLOAT(SERVO_ANG_LIBERA_DEG, s.cmd().angle_deg);
  // Sin confirmación: -80 (1 s), -10 (0.3 s), -80 (1 s), -10 (0.3 s), -80 (1 s), fin
  struct {
    uint32_t t;
    bool power;
    float ang;
    int attempt;
  } exp[] = {{990, true, SERVO_ANG_LIBERA_DEG, 1},   {1000, true, SERVO_ANG_BLOQUEO_DEG, 2},
             {1290, true, SERVO_ANG_BLOQUEO_DEG, 2}, {1300, true, SERVO_ANG_LIBERA_DEG, 2},
             {2300, true, SERVO_ANG_BLOQUEO_DEG, 3}, {2600, true, SERVO_ANG_LIBERA_DEG, 3},
             {3590, true, SERVO_ANG_LIBERA_DEG, 3},  {3600, false, SERVO_ANG_LIBERA_DEG, 3}};
  size_t k = 0;
  for (uint32_t t = 10; t <= 3700; t += 10) {
    s.step(t, false);
    if (k < sizeof exp / sizeof exp[0] && t == exp[k].t) {
      TEST_ASSERT_EQUAL_MESSAGE(exp[k].power, s.cmd().power, "energía");
      TEST_ASSERT_EQUAL_FLOAT(exp[k].ang, s.cmd().angle_deg);
      TEST_ASSERT_EQUAL(exp[k].attempt, s.attempts());
      k++;
    }
  }
  TEST_ASSERT_EQUAL(sizeof exp / sizeof exp[0], k);
  TEST_ASSERT_TRUE(s.done());
  TEST_ASSERT_FALSE(s.confirmed());
  // Con confirmación en el primer intento
  ServoSequencer c;
  c.start(0);
  c.step(400, true);
  TEST_ASSERT_TRUE(c.done());
  TEST_ASSERT_TRUE(c.confirmed());
  TEST_ASSERT_EQUAL(1, c.attempts());
  TEST_ASSERT_FALSE(c.cmd().power);
}

static void test_servo_pulso() {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1500.0f, servoPulseUs(0));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1500.0f - 10 * 1000.0f / 90, servoPulseUs(-10));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1500.0f - 80 * 1000.0f / 90, servoPulseUs(-80));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 500.0f, servoPulseUs(-200));  // saturación
}

// ---------------------------------------------------------------------------
// Modo demo
// ---------------------------------------------------------------------------
static void test_demo() {
  DemoSeparation d;
  d.step(0, false);
  TEST_ASSERT_FALSE(d.cmd().power);  // sin armar: sin energía
  d.arm(1000);
  uint32_t t_fire = 0;
  for (uint32_t t = 1000; t <= 1000 + DEMO_DELAY_S * 1000 + 100; t += 10) {
    d.step(t, false);
    if (!d.fired()) {
      TEST_ASSERT_TRUE(d.cmd().power);
      TEST_ASSERT_EQUAL_FLOAT(SERVO_ANG_BLOQUEO_DEG, d.cmd().angle_deg);
    }
    if (d.evFire()) t_fire = t;
  }
  TEST_ASSERT_EQUAL_UINT32(1000 + DEMO_DELAY_S * 1000, t_fire);
  TEST_ASSERT_EQUAL_UINT32(t_fire, d.tFire());
  TEST_ASSERT_EQUAL_FLOAT(SERVO_ANG_LIBERA_DEG, d.cmd().angle_deg);
  // Confirmación y fin
  d.step(t_fire + 300, true);
  TEST_ASSERT_TRUE(d.seq().done());
  TEST_ASSERT_FALSE(d.cmd().power);
  TEST_ASSERT_TRUE(d.fired());
  // Desarmar antes de tiempo cancela
  DemoSeparation e;
  e.arm(0);
  e.step(5000, false);
  e.disarm();
  for (uint32_t t = 5000; t < 20000; t += 10) e.step(t, false);
  TEST_ASSERT_FALSE(e.fired());
  TEST_ASSERT_FALSE(e.cmd().power);
}

// ---------------------------------------------------------------------------
// Modos de despliegue
// ---------------------------------------------------------------------------
static void test_modos_parametros() {
  const DeployParams a = deployParams(DEPLOY_MODE_A), b = deployParams(DEPLOY_MODE_B),
                     c = deployParams(DEPLOY_MODE_C);
  TEST_ASSERT_EQUAL_FLOAT(200.0f, a.deploy_alt_m);
  TEST_ASSERT_EQUAL_FLOAT(240.0f, a.release_min_m);  // DEPLOY_ALT + 40
  TEST_ASSERT_EQUAL_FLOAT(50.0f, b.deploy_alt_m);
  TEST_ASSERT_EQUAL_FLOAT(90.0f, b.release_min_m);
  TEST_ASSERT_FALSE(a.immediate || b.immediate);
  TEST_ASSERT_TRUE(c.immediate);
  DeployMode m;
  TEST_ASSERT_TRUE(deployModeFromChar('b', m));
  TEST_ASSERT_EQUAL(DEPLOY_MODE_B, m);
  TEST_ASSERT_EQUAL('C', deployModeChar(DEPLOY_MODE_C));
  TEST_ASSERT_FALSE(deployModeFromChar('D', m));
  TEST_ASSERT_EQUAL(DEPLOY_MODE_A, deployModeDefault());
  // El modo es configuración: sobrevive a begin() y a beginRecovered()
  StateMachine sm;
  TEST_ASSERT_EQUAL(DEPLOY_MODE_A, sm.deployMode());
  sm.setDeployMode(DEPLOY_MODE_B);
  sm.begin(0);
  TEST_ASSERT_EQUAL(DEPLOY_MODE_B, sm.deployMode());
  MissionRecord rec;
  rec.mission = 1;
  rec.state = ST_ASCENSO;
  sm.beginRecovered(rec, 0);
  TEST_ASSERT_EQUAL(DEPLOY_MODE_B, sm.deployMode());
}

static void test_modo_b_altura_minima_90() {
  // Soltado a 95 m: liberación válida en B (> 90 m) y no en A (< 240 m).
  StateMachine b;
  b.setDeployMode(DEPLOY_MODE_B);
  climbAndDrop(b, 95);
  TEST_ASSERT_EQUAL(ST_CAIDA, b.state());
  StateMachine a;
  climbAndDrop(a, 95);
  TEST_ASSERT_EQUAL(ST_ASCENSO, a.state());
  TEST_ASSERT_TRUE(a.out().lowg_descartadas > 0);
  // Soltado a 85 m: en B tampoco.
  StateMachine b2;
  b2.setDeployMode(DEPLOY_MODE_B);
  climbAndDrop(b2, 85);
  TEST_ASSERT_EQUAL(ST_ASCENSO, b2.state());
}

static void test_modo_b_h_disp_y_respaldo() {
  StateMachine sm;
  sm.setDeployMode(DEPLOY_MODE_B);
  uint32_t t = climbAndDrop(sm, 100);
  TEST_ASSERT_EQUAL(ST_CAIDA, sm.state());
  // h_disp = 50 + |vz| * 0.5: con vz = -24, 62 m. A 62.5 m no dispara; a 61.9 sí.
  SmInput in = base(t);
  in.h = 62.5f;
  in.vz = -24;
  sm.step(in);
  TEST_ASSERT_EQUAL(ST_CAIDA, sm.state());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 62.0f, sm.out().h_disp);
  in = base(t += 10);
  in.h = 61.9f;
  in.vz = -24;
  sm.step(in);
  TEST_ASSERT_EQUAL(ST_DESPLIEGUE, sm.state());
  TEST_ASSERT_EQUAL(0, sm.out().flags & FLAG_DISPARO_RESPALDO);
  // Respaldo de altura en B: inclinado siempre, dispara en el primer ciclo bajo 40 m.
  StateMachine r;
  r.setDeployMode(DEPLOY_MODE_B);
  t = climbAndDrop(r, 100);
  float h = 45;
  for (int i = 0; i < 100 && r.state() == ST_CAIDA; i++, t += 10, h -= 0.6f) {
    in = base(t);
    in.h = h;
    in.vz = -60;  // h_disp = 80 m: ya por debajo, la compuerta (80°) no se cumple
    in.tilt_deg = 80;
    r.step(in);
  }
  TEST_ASSERT_EQUAL(ST_DESPLIEGUE, r.state());
  TEST_ASSERT_TRUE(r.out().flags & FLAG_DISPARO_RESPALDO);
  TEST_ASSERT_TRUE(h + 0.6f <= DEPLOY_RESPALDO_B_M && h + 0.6f > DEPLOY_RESPALDO_B_M - 0.7f);
}

static void test_modo_b_sin_baro() {
  // Barómetro muerto en B: disparo DEPLOY_T_SIN_BARO_B_S después de la liberación.
  StateMachine sm;
  sm.setDeployMode(DEPLOY_MODE_B);
  uint32_t t = climbAndDrop(sm, 100);
  TEST_ASSERT_EQUAL(ST_CAIDA, sm.state());
  const uint32_t t_rel = sm.tRelease();
  for (int i = 0; i < 500 && sm.state() == ST_CAIDA; i++, t += 10) {
    SmInput in = base(t);
    in.baro_ok = false;
    in.vz = -20;
    sm.step(in);
  }
  TEST_ASSERT_EQUAL(ST_DESPLIEGUE, sm.state());
  TEST_ASSERT_EQUAL_UINT32(t_rel + (uint32_t)(DEPLOY_T_SIN_BARO_B_S * 1000), sm.tFire());
  TEST_ASSERT_TRUE(sm.out().flags & FLAG_DISPARO_RESPALDO);
}

static void test_modo_c_desde_tierra() {
  // En C la caída libre se detecta en TIERRA (sin subir 20 m), sin confirmación por vz,
  // sin altura mínima y sin compuerta: CAIDA y, en el ciclo siguiente, el disparo.
  StateMachine sm;
  sm.setDeployMode(DEPLOY_MODE_C);
  sm.begin(0);
  uint32_t t = 0;
  for (; t < 2000; t += 10) sm.step(base(t));
  TEST_ASSERT_EQUAL(ST_TIERRA, sm.state());
  // Un toque breve de baja g (< 150 ms) no cuenta
  for (int i = 0; i < 10; i++, t += 10) {
    SmInput in = base(t);
    in.h = 3;
    in.acc_g = 0.1f;
    sm.step(in);
  }
  sm.step(base(t += 10));
  TEST_ASSERT_EQUAL(ST_TIERRA, sm.state());
  bool froze = false;
  int n = 0;
  for (; n < 30 && sm.state() == ST_TIERRA; n++, t += 10) {
    SmInput in = base(t);
    in.h = 5;
    in.vz = -0.5f;      // apenas cayendo: no hace falta vz < -8
    in.acc_g = 0.1f;
    in.tilt_deg = 90;   // la compuerta no se aplica
    sm.step(in);
    froze |= sm.out().ev_freeze_p0;
  }
  TEST_ASSERT_EQUAL(ST_CAIDA, sm.state());
  TEST_ASSERT_EQUAL(16, n);  // 150 ms de baja g (16 muestras a 10 ms)
  TEST_ASSERT_TRUE(froze);
  TEST_ASSERT_EQUAL(1, sm.record().mission);
  TEST_ASSERT_TRUE(sm.out().flags & FLAG_LIBERACION);
  SmInput in = base(t);
  in.h = 4.5f;
  in.vz = -1.5f;
  in.acc_g = 0.1f;
  in.tilt_deg = 90;
  sm.step(in);
  TEST_ASSERT_EQUAL(ST_DESPLIEGUE, sm.state());
  TEST_ASSERT_EQUAL(FLAG_DISPARO, sm.out().flags & (FLAG_DISPARO | FLAG_DISPARO_RESPALDO));
  TEST_ASSERT_EQUAL_UINT32(t, sm.tFire());
  // En los modos A y B la misma caída en TIERRA no hace nada
  StateMachine a;
  a.begin(0);
  for (t = 0; t < 2000; t += 10) a.step(base(t));
  for (int i = 0; i < 100; i++, t += 10) {
    SmInput x = base(t);
    x.h = 5;
    x.acc_g = 0.1f;
    a.step(x);
  }
  TEST_ASSERT_EQUAL(ST_TIERRA, a.state());
}

// ---------------------------------------------------------------------------
// Botón PRG y LED del modo demo
// ---------------------------------------------------------------------------
static void test_toque_largo() {
  LongPress b(1000);
  uint32_t t = 0;
  int hits = 0;
  // Toque corto (0.5 s): nada
  for (; t < 500; t += 10) hits += b.step(true, t);
  hits += b.step(false, t);
  TEST_ASSERT_EQUAL(0, hits);
  // Toque largo de 3 s: una sola detección, al cumplir 1 s
  const uint32_t t0 = t += 10;
  uint32_t t_hit = 0;
  for (; t < t0 + 3000; t += 10)
    if (b.step(true, t)) {
      hits++;
      t_hit = t;
    }
  TEST_ASSERT_EQUAL(1, hits);
  TEST_ASSERT_EQUAL_UINT32(t0 + 1000, t_hit);
  // Un rebote (suelto 10 ms) reinicia la cuenta
  b.step(false, t);
  for (t += 10; t < t0 + 3700; t += 10) hits += b.step(true, t);
  b.step(false, t);
  for (t += 10; t < t0 + 4000; t += 10) hits += b.step(true, t);
  TEST_ASSERT_EQUAL(1, hits);
}

static void test_demo_toggle_y_led() {
  DemoSeparation d;
  TEST_ASSERT_FALSE(d.ledOn(0));
  d.toggle(1000);  // primer toque largo: arma
  TEST_ASSERT_TRUE(d.counting());
  // Parpadeo rápido: medio periodo DEMO_LED_PARPADEO_MS
  TEST_ASSERT_TRUE(d.ledOn(1000));
  TEST_ASSERT_FALSE(d.ledOn(1000 + DEMO_LED_PARPADEO_MS));
  TEST_ASSERT_TRUE(d.ledOn(1000 + 2 * DEMO_LED_PARPADEO_MS));
  d.step(5000, false);
  d.toggle(5000);  // segundo toque antes de terminar: cancela
  TEST_ASSERT_FALSE(d.armed());
  for (uint32_t t = 5000; t < 20000; t += 10) d.step(t, false);
  TEST_ASSERT_FALSE(d.fired());
  TEST_ASSERT_FALSE(d.ledOn(20000));
  TEST_ASSERT_FALSE(d.cmd().power);
  // Arma otra vez y deja terminar: dispara a los DEMO_DELAY_S; el LED queda fijo
  d.toggle(20000);
  uint32_t t_fire = 0;
  for (uint32_t t = 20000; t < 20000 + DEMO_DELAY_S * 1000 + 3000; t += 10) {
    d.step(t, false);
    if (d.evFire()) t_fire = t;
  }
  TEST_ASSERT_EQUAL_UINT32(20000 + DEMO_DELAY_S * 1000, t_fire);
  TEST_ASSERT_TRUE(d.fired());
  TEST_ASSERT_TRUE(d.ledOn(t_fire + 3000));
  TEST_ASSERT_TRUE(d.ledOn(t_fire + 3000 + DEMO_LED_PARPADEO_MS));
  // Tras el disparo, un toque largo re-arma (nueva toma)
  d.toggle(40000);
  TEST_ASSERT_TRUE(d.counting());
  TEST_ASSERT_FALSE(d.fired());
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_servo_sin_energia_hasta_caida);
  RUN_TEST(test_despegue_congela_p0_quieto);
  RUN_TEST(test_despegue_requiere_3s);
  RUN_TEST(test_compuerta_espera_y_dispara);
  RUN_TEST(test_respaldo_160m_sintetico);
  RUN_TEST(test_sin_imu_no_hay_compuerta);
  RUN_TEST(test_no_go);
  RUN_TEST(test_reinicio_decide_por_vz);
  RUN_TEST(test_aterrizado_tasas_y_mision);
  RUN_TEST(test_descenso_fuera_de_rango);
  RUN_TEST(test_servo_reintentos);
  RUN_TEST(test_servo_pulso);
  RUN_TEST(test_demo);
  RUN_TEST(test_modos_parametros);
  RUN_TEST(test_modo_b_altura_minima_90);
  RUN_TEST(test_modo_b_h_disp_y_respaldo);
  RUN_TEST(test_modo_b_sin_baro);
  RUN_TEST(test_modo_c_desde_tierra);
  RUN_TEST(test_toque_largo);
  RUN_TEST(test_demo_toggle_y_led);
  return UNITY_END();
}
