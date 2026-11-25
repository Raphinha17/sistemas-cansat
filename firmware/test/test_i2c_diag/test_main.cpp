// Pruebas de la interpretación de direcciones I2C y del veredicto del comando I2C.
#include <initializer_list>
#include <string.h>
#include <unity.h>

#include "i2c_diag.h"

void setUp() {}
void tearDown() {}

static I2cEscaneo esc(std::initializer_list<uint8_t> l) {
  I2cEscaneo e;
  for (uint8_t d : l) e.agrega(d);
  return e;
}

static void test_nombres() {
  TEST_ASSERT_EQUAL_STRING("MPU6500", i2cNombre(0x68));
  TEST_ASSERT_EQUAL_STRING("MPU6500", i2cNombre(0x69));
  TEST_ASSERT_EQUAL_STRING("BMP280", i2cNombre(0x76));
  TEST_ASSERT_EQUAL_STRING("BMP280", i2cNombre(0x77));
  TEST_ASSERT_EQUAL_STRING("desconocido", i2cNombre(0x3C));
  TEST_ASSERT_EQUAL_STRING("desconocido", i2cNombre(0x08));
}

static void test_deteccion_alternas() {
  TEST_ASSERT_TRUE(i2cHayImu(esc({0x69})));
  TEST_ASSERT_TRUE(i2cHayBaro(esc({0x77})));
  TEST_ASSERT_FALSE(i2cHayImu(esc({0x76})));
  TEST_ASSERT_FALSE(i2cHayBaro(esc({0x68})));
}

static char v[160];
static const char *ver(I2cEscaneo n, I2cEscaneo c, bool sda, bool scl) {
  i2cVeredicto(n, c, sda, scl, v, sizeof v);
  return v;
}

static void test_ok() {
  TEST_ASSERT_EQUAL_STRING("OK: IMU y BARO detectados",
                           ver(esc({0x68, 0x76}), esc({}), true, true));
  TEST_ASSERT_EQUAL_STRING("OK: IMU y BARO detectados",
                           ver(esc({0x69, 0x77, 0x3C}), esc({}), true, true));
}

static void test_cruzado() {
  ver(esc({}), esc({0x68, 0x76}), true, true);
  TEST_ASSERT_NOT_NULL(strstr(v, "CRUZADOS"));
}

static void test_linea_baja() {
  ver(esc({}), esc({}), false, true);
  TEST_ASSERT_NOT_NULL(strstr(v, "BAJO"));
  ver(esc({}), esc({}), true, false);
  TEST_ASSERT_NOT_NULL(strstr(v, "corto a GND"));
}

static void test_nada() {
  ver(esc({}), esc({}), true, true);
  TEST_ASSERT_NOT_NULL(strstr(v, "ningun sensor"));
}

static void test_falta_uno() {
  ver(esc({0x76}), esc({}), true, true);
  TEST_ASSERT_NOT_NULL(strstr(v, "falta IMU"));
  ver(esc({0x68}), esc({}), true, true);
  TEST_ASSERT_NOT_NULL(strstr(v, "falta BARO"));
  ver(esc({0x3C}), esc({}), true, true);
  TEST_ASSERT_NOT_NULL(strstr(v, "faltan IMU y BARO"));
}

static void test_truncado_seguro() {
  char c[8];
  i2cVeredicto(esc({}), esc({}), true, true, c, sizeof c);
  TEST_ASSERT_EQUAL(7, strlen(c));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_nombres);
  RUN_TEST(test_deteccion_alternas);
  RUN_TEST(test_ok);
  RUN_TEST(test_cruzado);
  RUN_TEST(test_linea_baja);
  RUN_TEST(test_nada);
  RUN_TEST(test_falta_uno);
  RUN_TEST(test_truncado_seguro);
  return UNITY_END();
}
