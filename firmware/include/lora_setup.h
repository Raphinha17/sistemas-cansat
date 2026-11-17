// Configuración LoRa compartida por el firmware de vuelo y el receptor (rx_station).
// Los pines del SX1262 salen de la variante de la placa (pins_arduino.h de
// heltec_wifi_lora_32_V3): SS, SCK, MISO, MOSI, RST_LoRa, BUSY_LoRa y DIO0
// (en la Heltec V3, "DIO0" es la línea DIO1 del SX1262, la de interrupciones).
#pragma once

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

#include "config.h"

inline Module *loraModule() {
  static SPIClass spi(FSPI);
  spi.begin(SCK, MISO, MOSI, SS);
  static Module mod(SS, DIO0, RST_LoRa, BUSY_LoRa, spi);
  return &mod;
}

// Inicia el radio con los parámetros de la SPEC. Devuelve el código de RadioLib
// (RADIOLIB_ERR_NONE = 0 si todo bien).
inline int16_t loraBegin(SX1262 &radio) {
  int16_t st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR, LORA_SYNC_WORD,
                           LORA_TX_DBM, LORA_PREAMBLE, LORA_TCXO_V);
  if (st != RADIOLIB_ERR_NONE) return st;
  // La Heltec V3 conmuta la antena con DIO2 del SX1262 (PREGUNTAS P4).
  st = radio.setDio2AsRfSwitch(true);
  if (st != RADIOLIB_ERR_NONE) return st;
  return radio.setCRC(true);
}
