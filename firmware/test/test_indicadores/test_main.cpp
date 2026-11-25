// Pruebas de la lógica pura de los patrones del LED y del texto de las pantallas.
#include <string.h>
#include <unity.h>

#include "indicadores.h"

void setUp() {}
void tearDown() {}

// Cuenta los destellos (flancos de subida) en una ventana, muestreando cada 10 ms.
static int destellos(uint8_t est, uint8_t fl, uint32_t t0, uint32_t dur) {
  int n = 0;
  bool prev = false;
  for (uint32_t t = t0; t < t0 + dur; t += 10) {
    const bool on = ledOn(est, fl, t);
    if (on && !prev) n++;
    prev = on;
  }
  return n;
}

static uint32_t msEncendido(uint8_t est, uint8_t fl, uint32_t dur) {
  uint32_t ms = 0;
  for (uint32_t t = 0; t < dur; t += 10) ms += ledOn(est, fl, t) ? 10 : 0;
  return ms;
}

static void test_led_autoprueba() {
  TEST_ASSERT_EQUAL(10, destellos(ST_AUTOPRUEBA, 0, 0, 1000));  // 10 Hz
}

static void test_led_tierra_ok_doble_destello() {
  TEST_ASSERT_EQUAL(2, destellos(ST_TIERRA, FLAG_BASE_LISTA, 0, 2000));   // 2 por periodo
  TEST_ASSERT_EQUAL(4, destellos(ST_TIERRA, FLAG_BASE_LISTA, 0, 4000));
  TEST_ASSERT_TRUE(ledOn(ST_TIERRA, 0, 50));
  TEST_ASSERT_FALSE(ledOn(ST_TIERRA, 0, 150));
  TEST_ASSERT_TRUE(ledOn(ST_TIERRA, 0, 300));
  TEST_ASSERT_FALSE(ledOn(ST_TIERRA, 0, 1000));
  TEST_ASSERT_FALSE(hayFallo(FLAG_BASE_LISTA | FLAG_LIBERACION));  // bits 0 a 4 no son fallo
}

static void test_led_tierra_con_fallo_5hz() {
  const uint8_t malos[] = {FLAG_FALLO_BARO, FLAG_FALLO_IMU, FLAG_NO_GO, 0xE0};
  for (uint8_t f : malos) {
    TEST_ASSERT_TRUE(hayFallo(f));
    TEST_ASSERT_EQUAL(5, destellos(ST_TIERRA, f, 0, 1000));  // 5 Hz
    TEST_ASSERT_EQUAL(500, msEncendido(ST_TIERRA, f, 1000));
  }
}

static void test_led_vuelo_apagado() {
  for (uint8_t e = ST_ASCENSO; e <= ST_DESCENSO; e++)
    TEST_ASSERT_EQUAL(0, msEncendido(e, 0xE0 * (e & 1), 4000));  // con o sin fallos
}

static void test_led_aterrizado_un_destello_por_segundo() {
  TEST_ASSERT_EQUAL(5, destellos(ST_ATERRIZADO, 0, 0, 5000));
  TEST_ASSERT_EQUAL(5, destellos(ST_ATERRIZADO, FLAG_ATERRIZADO, 0, 5000));
  TEST_ASSERT_EQUAL(100, msEncendido(ST_ATERRIZADO, FLAG_ATERRIZADO, 1000));
}

static void test_oled_solo_en_0_1_6() {
  TEST_ASSERT_TRUE(oledActiva(ST_AUTOPRUEBA) && oledActiva(ST_TIERRA) && oledActiva(ST_ATERRIZADO));
  for (uint8_t e = ST_ASCENSO; e <= ST_DESCENSO; e++) TEST_ASSERT_FALSE(oledActiva(e));
}

// ---- Framebuffer 64x32: se "dibuja" cada texto (celda de 6x8 por tamaño) y se cuentan los
// píxeles fuera del área visible. Es conservador: la celda contiene a los píxeles del glifo.
struct Fb {
  bool px[OLED_ALTO][OLED_ANCHO] = {};
  int fuera = 0;
  void pon(int x, int y) {
    if (x < 0 || y < 0 || x >= OLED_ANCHO || y >= OLED_ALTO) fuera++;
    else px[y][x] = true;
  }
};

static void dibuja(const Pantalla &p, Fb &fb) {
  for (uint8_t i = 0; i < p.n; i++) {
    const Texto &t = p.t[i];
    const int w = (int)strlen(t.s) * OLED_FUENTE_ANCHO * t.tam;
    const int h = OLED_FUENTE_ALTO * t.tam;
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) fb.pon(t.x + x, t.y + y);
    if (t.invertido) {  // el fondo blanco (w + 2 de ancho, como en oled_hw.cpp)
      const int x0 = (t.tam == 1 && t.x == 0) ? 0 : t.x - 1;
      for (int y = 0; y < h; y++)
        for (int x = (x0 < 0 ? 0 : x0); x < (x0 < 0 ? 0 : x0) + w + 2 && x < OLED_ANCHO; x++)
          fb.pon(x, t.y + y);
    }
  }
}

static void verifica(const Pantalla &p) {
  for (uint8_t i = 0; i < p.n; i++) {
    const Texto &t = p.t[i];
    if (t.tam == 1) TEST_ASSERT_TRUE(strlen(t.s) <= OLED_COLUMNAS);
    TEST_ASSERT_TRUE((int)strlen(t.s) * OLED_FUENTE_ANCHO * t.tam <= OLED_ANCHO);
    TEST_ASSERT_TRUE(t.x >= 0 && t.y >= 0 && t.y + OLED_FUENTE_ALTO * t.tam <= OLED_ALTO);
  }
  Fb fb;
  dibuja(p, fb);
  TEST_ASSERT_EQUAL(0, fb.fuera);
}

static void test_config_oled() {
  TEST_ASSERT_EQUAL(64, OLED_ANCHO);
  TEST_ASSERT_EQUAL(32, OLED_ALTO);
  TEST_ASSERT_EQUAL(32, OLED_DESPLAZ_X);  // 64 visibles centrados en 128 columnas
  TEST_ASSERT_EQUAL(10, OLED_COLUMNAS);
  TEST_ASSERT_EQUAL(4, OLED_FILAS);
}

static void test_pantalla_vuelo_go() {
  Pantalla p;
  pantallaVuelo(ST_TIERRA, FLAG_BASE_LISTA, 'B', p);
  TEST_ASSERT_EQUAL(3, p.n);
  TEST_ASSERT_EQUAL_STRING("GO", p.t[0].s);
  TEST_ASSERT_EQUAL(2, p.t[0].tam);        // fuente grande
  TEST_ASSERT_FALSE(p.t[0].invertido);
  TEST_ASSERT_EQUAL_STRING("MODO B  S1", p.t[1].s);
  TEST_ASSERT_EQUAL_STRING("I:OK B:OK", p.t[2].s);
  verifica(p);
  pantallaVuelo(ST_ATERRIZADO, FLAG_ATERRIZADO, 'D', p);
  TEST_ASSERT_EQUAL_STRING("MODO D  S6", p.t[1].s);
  verifica(p);
}

static void test_pantalla_vuelo_no_go() {
  Pantalla p;
  pantallaVuelo(ST_TIERRA, FLAG_FALLO_BARO, 'C', p);
  TEST_ASSERT_EQUAL_STRING("NO-GO", p.t[0].s);
  TEST_ASSERT_TRUE(p.t[0].invertido);
  TEST_ASSERT_EQUAL_STRING("I:OK B:NO", p.t[2].s);
  verifica(p);
  pantallaVuelo(ST_TIERRA, 0xE0, 'A', p);                     // sin sensores
  TEST_ASSERT_EQUAL_STRING("I:NO B:NO", p.t[2].s);
  verifica(p);
  pantallaVuelo(ST_AUTOPRUEBA, FLAG_NO_GO, 'B', p);            // NO-GO solo
  TEST_ASSERT_EQUAL_STRING("NO-GO", p.t[0].s);
  TEST_ASSERT_EQUAL_STRING("I:OK B:OK", p.t[2].s);
  verifica(p);
}

static void test_pantalla_vuelo_todas_las_combinaciones() {
  const char modos[] = {'A', 'B', 'C', 'D'};
  for (uint8_t e = 0; e <= ST_ATERRIZADO; e++)
    for (int f = 0; f < 256; f += 1)
      for (char m : modos) {
        Pantalla p;
        pantallaVuelo(e, (uint8_t)f, m, p);
        verifica(p);
      }
}

static void test_pantalla_rx() {
  Pantalla p;
  pantallaRx(0, false, 0, 0, 0, 0, 0, p);
  TEST_ASSERT_EQUAL_STRING("RX 0", p.t[0].s);
  TEST_ASSERT_EQUAL_STRING("SIN ENLACE", p.t[3].s);   // 10 caracteres, completo
  TEST_ASSERT_TRUE(p.t[3].invertido);
  verifica(p);
  pantallaRx(123, true, -45.4f, 12.0f, ST_TIERRA, 0.04f, 2300, p);
  TEST_ASSERT_EQUAL_STRING("RX 123", p.t[0].s);
  TEST_ASSERT_EQUAL_STRING("-45/12dB", p.t[1].s);
  TEST_ASSERT_EQUAL_STRING("ST1 H0.0m", p.t[2].s);
  TEST_ASSERT_EQUAL_STRING("ULT 2s", p.t[3].s);
  TEST_ASSERT_FALSE(p.t[3].invertido);
  verifica(p);
  pantallaRx(5, true, -45.0f, 9.0f, ST_DESCENSO, 1.0f, 5000, p);   // 5 s justos: aún enlace
  TEST_ASSERT_EQUAL_STRING("ULT 5s", p.t[3].s);
  pantallaRx(5, true, -45.0f, 9.0f, ST_DESCENSO, 1.0f, 5001, p);   // más de 5 s
  TEST_ASSERT_EQUAL_STRING("SIN ENLACE", p.t[3].s);
  verifica(p);
  pantallaRx(57, true, -88.0f, -7.0f, ST_ASCENSO, 450.14f, 800, p);
  TEST_ASSERT_EQUAL_STRING("ST2 H450m", p.t[2].s);
  verifica(p);
}

static void test_pantalla_rx_extremos_caben() {
  const uint32_t cuentas[] = {0, 9, 999999, 1000000, 4294967295u};
  const float rssi[] = {0, -45.5f, -120.5f, -999.0f}, snr[] = {-99.0f, -20.5f, 0, 14.25f};
  const float alt[] = {-3276.8f, -450.0f, -99.9f, -9.95f, 0, 9.95f, 99.9f, 450.1f, 3276.7f};
  const uint32_t desde[] = {0, 1000, 5000, 5001, 4294967295u};
  for (uint32_t c : cuentas)
    for (float r : rssi)
      for (float s : snr)
        for (float h : alt)
          for (uint32_t d : desde)
            for (uint8_t e = 0; e <= ST_ATERRIZADO; e++) {
              Pantalla p;
              pantallaRx(c, true, r, s, e, h, d, p);
              verifica(p);
            }
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_led_autoprueba);
  RUN_TEST(test_led_tierra_ok_doble_destello);
  RUN_TEST(test_led_tierra_con_fallo_5hz);
  RUN_TEST(test_led_vuelo_apagado);
  RUN_TEST(test_led_aterrizado_un_destello_por_segundo);
  RUN_TEST(test_oled_solo_en_0_1_6);
  RUN_TEST(test_config_oled);
  RUN_TEST(test_pantalla_vuelo_go);
  RUN_TEST(test_pantalla_vuelo_no_go);
  RUN_TEST(test_pantalla_vuelo_todas_las_combinaciones);
  RUN_TEST(test_pantalla_rx);
  RUN_TEST(test_pantalla_rx_extremos_caben);
  return UNITY_END();
}
