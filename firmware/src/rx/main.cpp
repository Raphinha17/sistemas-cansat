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

#include "lora_setup.h"

static SX1262 radio(loraModule());
static volatile bool rx_flag = false;

static void IRAM_ATTR onDio1() { rx_flag = true; }

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("# CanSat Hellmary - receptor rx_station");
  const int16_t st = loraBegin(radio);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("# ERROR radio %d\n", st);
    for (;;) delay(1000);
  }
  radio.setDio1Action(onDio1);
  const int16_t rs = radio.startReceive();
  Serial.printf("# radio OK, escuchando (%d)\n", rs);
}

void loop() {
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
  } else {
    Serial.printf("ERR,%d,%.1f,%.2f\n", st, rssi, snr);
  }
  radio.startReceive();
}
