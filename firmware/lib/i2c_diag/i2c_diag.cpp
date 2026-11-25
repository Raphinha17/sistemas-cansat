#include "i2c_diag.h"

#include <stdio.h>

const char *i2cNombre(uint8_t dir) {
  if (dir == 0x68 || dir == 0x69) return "MPU6500";
  if (dir == 0x76 || dir == 0x77) return "BMP280";
  return "desconocido";
}

bool i2cHayImu(const I2cEscaneo &e) { return e.tiene(0x68) || e.tiene(0x69); }
bool i2cHayBaro(const I2cEscaneo &e) { return e.tiene(0x76) || e.tiene(0x77); }

void i2cVeredicto(const I2cEscaneo &normal, const I2cEscaneo &cruzado, bool sda_alto,
                  bool scl_alto, char *out, size_t cap) {
  const bool imu = i2cHayImu(normal), baro = i2cHayBaro(normal);
  const char *msg;
  if (imu && baro) {
    msg = "OK: IMU y BARO detectados";
  } else if (normal.n == 0 && cruzado.n > 0) {
    msg = "FALLA: sin sensores; responden con SDA/SCL CRUZADOS (intercambiar 41 y 42)";
  } else if (!sda_alto || !scl_alto) {
    msg = "FALLA: linea en BAJO (posible corto a GND o sin pull-up); revisar SDA/SCL y 3V3";
  } else if (normal.n == 0) {
    msg = "FALLA: ningun sensor responde; revisar alimentacion 3V3, GND y cableado SDA/SCL";
  } else if (!imu && !baro) {
    msg = "FALLA: faltan IMU y BARO (solo responden otros); revisar sensores y direcciones";
  } else if (!imu) {
    msg = "FALLA: falta IMU (MPU6500); revisar su alimentacion, cableado y pin AD0";
  } else {
    msg = "FALLA: falta BARO (BMP280); revisar su alimentacion, cableado y pin SDO";
  }
  snprintf(out, cap, "%s", msg);
}
