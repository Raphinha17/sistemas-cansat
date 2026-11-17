// Pruebas del armado del paquete (mismas escalas que protocol/telemetry.py).
#include <string.h>
#include <unity.h>

#include "telemetry_pack.h"

void setUp() {}
void tearDown() {}

static void test_size() { TEST_ASSERT_EQUAL(32, sizeof(TelemetryPacket)); }

static void test_scaling_and_layout() {
  TelemetryValues v;
  v.estado = ST_TIERRA;
  v.secuencia = 0x1234;
  v.t_ms = 0xAABBCCDD;
  v.presion_pa = 95500.4f;
  v.temp_c = -12.34f;
  v.altura_m = 245.7f;
  v.vz_ms = -35.12f;
  v.acc_g[0] = 0.123f;
  v.acc_g[1] = -0.456f;
  v.acc_g[2] = 1.0f;
  v.roll_deg = -45.67f;
  v.pitch_deg = 12.34f;
  v.yaw_deg = 179.99f;
  v.banderas = FLAG_BASE_LISTA | FLAG_NO_GO;
  v.bateria_v = 8.7f;
  const TelemetryPacket p = packTelemetry(v);
  uint8_t b[32];
  memcpy(b, &p, 32);
  // Little-endian (el ESP32 y el PC x86 lo son)
  TEST_ASSERT_EQUAL_HEX8(PACKET_VERSION, b[0]);
  TEST_ASSERT_EQUAL_HEX8(1, b[1]);
  TEST_ASSERT_EQUAL_HEX8(0x34, b[2]);
  TEST_ASSERT_EQUAL_HEX8(0x12, b[3]);
  TEST_ASSERT_EQUAL_HEX8(0xDD, b[4]);
  TEST_ASSERT_EQUAL_UINT32(95500, p.presion_Pa);
  TEST_ASSERT_EQUAL_INT16(-1234, p.temp_centi);
  TEST_ASSERT_EQUAL_INT16(2457, p.altura_dm);
  TEST_ASSERT_EQUAL_INT16(-3512, p.vz_cms);
  TEST_ASSERT_EQUAL_INT16(123, p.ax_mg);
  TEST_ASSERT_EQUAL_INT16(-456, p.ay_mg);
  TEST_ASSERT_EQUAL_INT16(1000, p.az_mg);
  TEST_ASSERT_EQUAL_INT16(-4567, p.roll_centi);
  TEST_ASSERT_EQUAL_INT16(17999, p.yaw_centi);
  TEST_ASSERT_EQUAL_HEX8(0x81, b[30]);
  TEST_ASSERT_EQUAL_UINT8(87, b[31]);
}

static void test_saturation() {
  TelemetryValues v;
  v.altura_m = 1e6f;
  v.vz_ms = -1e6f;
  v.bateria_v = 100.0f;
  v.presion_pa = -5.0f;
  const TelemetryPacket p = packTelemetry(v);
  TEST_ASSERT_EQUAL_INT16(32767, p.altura_dm);
  TEST_ASSERT_EQUAL_INT16(-32768, p.vz_cms);
  TEST_ASSERT_EQUAL_UINT8(255, p.bateria_dv);
  TEST_ASSERT_EQUAL_UINT32(0, p.presion_Pa);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_size);
  RUN_TEST(test_scaling_and_layout);
  RUN_TEST(test_saturation);
  return UNITY_END();
}
