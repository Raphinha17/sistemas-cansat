// Lectura de sensores por I2C: MPU6500 (driver mínimo propio) y BMP280 (Adafruit).
#pragma once

#include <Arduino.h>

struct ImuSample {
  float acc_g[3];     // fuerza específica en g, ejes del sensor
  float gyro_dps[3];  // °/s, ejes del sensor (sin corregir sesgo)
};

struct BaroSample {
  float p_pa;
  float temp_c;
};

class Sensors {
 public:
  // Inicializa el bus I2C y ambos sensores. Devuelve true si al menos uno responde.
  bool begin();
  bool readImu(ImuSample &s);
  bool readBaro(BaroSample &s);

  bool imuOk() const { return imu_ok_; }
  bool baroOk() const { return baro_ok_; }
  uint8_t imuWhoAmI() const { return who_am_i_; }

 private:
  bool beginImu();
  bool beginBaro();
  bool imu_ok_ = false;
  bool baro_ok_ = false;
  uint8_t who_am_i_ = 0;
};
