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

static void test_pantalla_vuelo_go() {
  PantallaVuelo p;
  pantallaVuelo(ST_TIERRA, FLAG_BASE_LISTA, 'B', 42, p);
  TEST_ASSERT_EQUAL_STRING("HELLMARY  MODO B", p.titulo);
  TEST_ASSERT_EQUAL_STRING("EST 1 TIERRA", p.estado);
  TEST_ASSERT_EQUAL_STRING("GO", p.go);
  TEST_ASSERT_TRUE(p.go_ok);
  TEST_ASSERT_EQUAL_STRING("IMU OK  BARO OK", p.sensores);
  TEST_ASSERT_EQUAL_STRING("TX 42", p.tx);
  pantallaVuelo(ST_ATERRIZADO, FLAG_ATERRIZADO, 'D', 123456, p);
  TEST_ASSERT_EQUAL_STRING("HELLMARY  MODO D", p.titulo);
  TEST_ASSERT_EQUAL_STRING("EST 6 ATERRIZADO", p.estado);
  TEST_ASSERT_EQUAL_STRING("TX 123456", p.tx);
}

static void test_pantalla_vuelo_no_go() {
  PantallaVuelo p;
  pantallaVuelo(ST_TIERRA, FLAG_FALLO_BARO, 'B', 0, p);
  TEST_ASSERT_EQUAL_STRING("NO-GO", p.go);
  TEST_ASSERT_FALSE(p.go_ok);
  TEST_ASSERT_EQUAL_STRING("IMU OK  BARO FALLA", p.sensores);
  pantallaVuelo(ST_TIERRA, 0xE0, 'A', 0, p);                 // sin sensores
  TEST_ASSERT_EQUAL_STRING("IMU FALLA  BARO FALLA", p.sensores);
  TEST_ASSERT_EQUAL_STRING("NO-GO", p.go);
  pantallaVuelo(ST_AUTOPRUEBA, FLAG_NO_GO, 'C', 0, p);        // NO-GO solo
  TEST_ASSERT_EQUAL_STRING("NO-GO", p.go);
  TEST_ASSERT_EQUAL_STRING("IMU OK  BARO OK", p.sensores);
}

static void test_pantalla_vuelo_cabe_en_128px() {
  // 6 px por carácter a tamaño 1: máximo 21 caracteres por línea
  PantallaVuelo p;
  pantallaVuelo(ST_ATERRIZADO, 0xE0, 'D', 4294967295u, p);
  TEST_ASSERT_TRUE(strlen(p.titulo) <= 21 && strlen(p.estado) <= 21);
  TEST_ASSERT_TRUE(strlen(p.sensores) <= 21 && strlen(p.tx) <= 21);
  TEST_ASSERT_TRUE(strlen(p.go) <= 5);  // tamaño 3: 18 px por carácter -> 5 caben (90 px)
}

static void test_pantalla_rx() {
  PantallaRx p;
  pantallaRx(0, false, 0, 0, 0, 0, 0, p);
  TEST_ASSERT_TRUE(p.sin_enlace);
  TEST_ASSERT_EQUAL_STRING("SIN ENLACE", p.ultimo);
  TEST_ASSERT_EQUAL_STRING("RX PAQ 0", p.titulo);
  pantallaRx(57, true, -45.5f, 9.75f, ST_DESCENSO, 123.46f, 2300, p);
  TEST_ASSERT_FALSE(p.sin_enlace);
  TEST_ASSERT_EQUAL_STRING("RX PAQ 57", p.titulo);
  TEST_ASSERT_EQUAL_STRING("RSSI -45.5 SNR 9.8", p.enlace);
  TEST_ASSERT_EQUAL_STRING("EST 5 DESCENSO", p.estado);
  TEST_ASSERT_EQUAL_STRING("ALT 123.5 m", p.altura);
  TEST_ASSERT_EQUAL_STRING("ULTIMO HACE 2 s", p.ultimo);
  pantallaRx(57, true, -45.5f, 9.75f, ST_DESCENSO, 1.0f, 5000, p);   // 5 s justos: aún enlace
  TEST_ASSERT_FALSE(p.sin_enlace);
  pantallaRx(57, true, -45.5f, 9.75f, ST_DESCENSO, 1.0f, 5001, p);   // más de 5 s
  TEST_ASSERT_TRUE(p.sin_enlace);
  TEST_ASSERT_EQUAL_STRING("SIN ENLACE", p.ultimo);
  TEST_ASSERT_TRUE(strlen(p.titulo) <= 21 && strlen(p.enlace) <= 21);
  pantallaRx(4294967295u, true, -120.5f, -20.5f, ST_ATERRIZADO, -123.4f, 4000, p);
  TEST_ASSERT_TRUE(strlen(p.titulo) <= 21 && strlen(p.enlace) <= 21 && strlen(p.estado) <= 21);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_led_autoprueba);
  RUN_TEST(test_led_tierra_ok_doble_destello);
  RUN_TEST(test_led_tierra_con_fallo_5hz);
  RUN_TEST(test_led_vuelo_apagado);
  RUN_TEST(test_led_aterrizado_un_destello_por_segundo);
  RUN_TEST(test_oled_solo_en_0_1_6);
  RUN_TEST(test_pantalla_vuelo_go);
  RUN_TEST(test_pantalla_vuelo_no_go);
  RUN_TEST(test_pantalla_vuelo_cabe_en_128px);
  RUN_TEST(test_pantalla_rx);
  return UNITY_END();
}
