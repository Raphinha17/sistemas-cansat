// Diagnóstico del bus I2C de los sensores: interpretación de direcciones y veredicto.
// Funciones puras (sin Arduino), probadas en native. El escaneo real está en
// src/flight/sensors.cpp (comando serial I2C).
#pragma once

#include <stddef.h>
#include <stdint.h>

constexpr uint8_t I2C_DIR_MIN = 0x08;
constexpr uint8_t I2C_DIR_MAX = 0x77;

// Direcciones que respondieron en un escaneo.
struct I2cEscaneo {
  uint8_t dir[16];
  uint8_t n = 0;
  void agrega(uint8_t d) {
    if (n < sizeof dir) dir[n++] = d;
  }
  bool tiene(uint8_t d) const {
    for (uint8_t i = 0; i < n; i++)
      if (dir[i] == d) return true;
    return false;
  }
};

// "MPU6500" (0x68, 0x69), "BMP280" (0x76, 0x77) o "desconocido".
const char *i2cNombre(uint8_t dir);

bool i2cHayImu(const I2cEscaneo &e);   // 0x68 o 0x69
bool i2cHayBaro(const I2cEscaneo &e);  // 0x76 o 0x77

// Veredicto de una línea. normal = SDA 41 / SCL 42; cruzado = SDA 42 / SCL 41;
// sda_alto / scl_alto = nivel en reposo con pull-up interno.
void i2cVeredicto(const I2cEscaneo &normal, const I2cEscaneo &cruzado, bool sda_alto,
                  bool scl_alto, char *out, size_t cap);
