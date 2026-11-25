// Receptor de la estación terrena (segunda Heltec V3).
// Imprime por USB una línea por paquete recibido:
//
//   <hex del paquete>,<RSSI dBm>,<SNR dB>
//   p. ej.  0100050010270000...59,-48.5,9.75
//
// Si el paquete llega con CRC inválido u otro error:  ERR,<codigo RadioLib>,<RSSI>,<SNR>
// Mensajes informativos empiezan con '#', para que el backend los ignore.
// Formato documentado en protocol/packet.md.
#include <Arduino.h>

#include "indicadores.h"
#include "lora_setup.h"
#include "oled_hw.h"
#include "packet.h"

static SX1262 radio(loraModule());
static volatile bool rx_flag = false;
// Datos de la pantalla (OLED): último paquete recibido y contadores
static uint32_t rx_count = 0, last_rx_ms = 0, last_draw_ms = 0;
static bool have_pkt = false;
static float last_rssi = 0, last_snr = 0, last_h = 0;
static uint8_t last_estado = 0;

static void IRAM_ATTR onDio1() { rx_flag = true; }

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("# CanSat Hellmary - receptor rx_station");
  const bool oled_ok = oled::begin();  // sin pantalla o sin respuesta: sigue normal
  Serial.printf("# OLED %s\n", oled_ok ? "OK" : "no responde (sigue sin pantalla)");
  const int16_t st = loraBegin(radio);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("# ERROR radio %d\n", st);
    for (;;) delay(1000);
  }
  radio.setDio1Action(onDio1);
  const int16_t rs = radio.startReceive();
  Serial.printf("# radio OK, escuchando (%d)\n", rs);
}

static void actualizaPantalla() {
  const uint32_t now = millis();
  if (!oled::present() || now - last_draw_ms < 500) return;  // 2 Hz como máximo
  last_draw_ms = now;
  Pantalla p;
  pantallaRx(rx_count, have_pkt, last_rssi, last_snr, last_estado, last_h, now - last_rx_ms, p);
  oled::draw(p);
}

void loop() {
  actualizaPantalla();
  if (!rx_flag) return;
  rx_flag = false;

  uint8_t buf[256];
  const size_t len = radio.getPacketLength();
  const int16_t st = radio.readData(buf, len < sizeof buf ? len : sizeof buf);
  const float rssi = radio.getRSSI();
  const float snr = radio.getSNR();

  if (st == RADIOLIB_ERR_NONE) {
    char hex[2 * sizeof buf + 1];
    const size_t n = len < sizeof buf ? len : sizeof buf;
    for (size_t i = 0; i < n; i++) snprintf(&hex[2 * i], 3, "%02X", buf[i]);
    hex[2 * n] = '\0';
    Serial.printf("%s,%.1f,%.2f\n", hex, rssi, snr);
    if (n == sizeof(TelemetryPacket) && buf[0] == PACKET_VERSION && buf[1] <= ST_ATERRIZADO) {
      TelemetryPacket tp;
      memcpy(&tp, buf, sizeof tp);
      rx_count++;
      have_pkt = true;
      last_rx_ms = millis();
      last_rssi = rssi;
      last_snr = snr;
      last_estado = tp.estado;
      last_h = tp.altura_dm / 10.0f;
    }
  } else {
    Serial.printf("ERR,%d,%.1f,%.2f\n", st, rssi, snr);
  }
  radio.startReceive();
}
