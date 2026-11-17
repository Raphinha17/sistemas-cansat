// Filtro de actitud Mahony (giroscopio + acelerómetro, sin magnetómetro).
// Código portable (sin Arduino) para poder probarlo en el entorno native.
//
// Convenciones:
//  - Cuerpo: eje z "arriba" en reposo; el acelerómetro mide fuerza específica en g
//    (en reposo y nivelado: ax=0, ay=0, az=+1).
//  - Giroscopio en °/s.
//  - Cuaternión q = cuerpo -> mundo (mundo z hacia arriba). Euler ZYX (yaw, pitch, roll).
//  - El yaw solo es relativo (no hay magnetómetro), deriva con el tiempo.
#pragma once

#include <stdint.h>

class Mahony {
 public:
  Mahony(float kp = 2.0f, float ki = 0.02f, float acc_gate_g = 0.15f)
      : kp_(kp), ki_(ki), acc_gate_g_(acc_gate_g) {}

  void reset();
  // Inicializa roll/pitch a partir de una lectura del acelerómetro (en reposo).
  void initFromAccel(float ax, float ay, float az);
  // Un paso del filtro. gyro en °/s, acc en g, dt en s.
  void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

  float roll() const;   // grados
  float pitch() const;  // grados
  float yaw() const;    // grados, relativo
  // Componente vertical (mundo) de la fuerza específica medida, en g. En reposo ~1.
  float verticalSpecificForce(float ax, float ay, float az) const;
  // Ángulo (°) entre un eje del cuerpo (índice 0..2, signo +-1) y la vertical hacia arriba.
  float axisTiltDeg(int axis, int sign) const;
  // Verdadero si la última actualización usó la corrección del acelerómetro.
  bool accelUsed() const { return acc_used_; }

  float q[4] = {1, 0, 0, 0};

 private:
  float kp_, ki_, acc_gate_g_;
  float ix_ = 0, iy_ = 0, iz_ = 0;  // término integral (rad/s)
  bool acc_used_ = false;
};

// Estimación del sesgo del giroscopio en reposo: media de N muestras y verificación
// de que no hubo movimiento (desviación estándar menor que un umbral).
class GyroBiasCal {
 public:
  explicit GyroBiasCal(int n_samples = 200, float max_std_dps = 0.5f)
      : n_(n_samples), max_std_(max_std_dps) {}
  void reset();
  // Agrega una muestra. Devuelve true cuando la ventana se completó (ver ok()).
  bool add(float gx, float gy, float gz);
  bool done() const { return count_ >= n_; }
  bool ok() const { return ok_; }  // ventana completa y sin movimiento
  float bias[3] = {0, 0, 0};

 private:
  int n_;
  float max_std_;
  int count_ = 0;
  double sum_[3] = {0, 0, 0}, sum2_[3] = {0, 0, 0};
  bool ok_ = false;
};
