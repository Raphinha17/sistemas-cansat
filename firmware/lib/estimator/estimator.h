// Estimador vertical (Kalman de 2 estados: altura h y velocidad vz) y calibración de
// la presión base p0. Código portable (sin Arduino) para pruebas en native.
//
// Entrada del modelo: aceleración vertical del mundo (m/s^2, positiva hacia arriba),
// obtenida rotando el acelerómetro con la actitud y restando 1 g.
// Medición: altura barométrica relativa a p0.
#pragma once

#include <stdint.h>

static constexpr float kG = 9.80665f;

// Altura (m) a partir de presión p y presión base p0 (fórmula barométrica estándar).
float baroAltitude(float p_pa, float p0_pa);

class VerticalKF {
 public:
  VerticalKF(float acc_sigma = 0.5f, float baro_sigma = 0.4f)
      : acc_sigma_(acc_sigma), baro_var_(baro_sigma * baro_sigma) {}

  void reset(float h0 = 0.0f, float vz0 = 0.0f);
  // Cambia el ruido de proceso (p. ej. más alto si el IMU falla y a_up = 0).
  void setAccSigma(float s) { acc_sigma_ = s; }
  // Predicción con aceleración vertical a_up (m/s^2) durante dt (s).
  void predict(float a_up, float dt);
  // Corrección con altura barométrica (m).
  void updateBaro(float h_baro);

  float h() const { return x_[0]; }
  float vz() const { return x_[1]; }
  float P[2][2] = {{1, 0}, {0, 1}};

 private:
  float acc_sigma_, baro_var_;
  float x_[2] = {0, 0};
};

// Presión base p0 por promedio móvil de ~N segundos. Se alimenta con muestras de
// presión; cada segundo se guarda la media de ese segundo en un anillo de N valores.
class BaseCal {
 public:
  static constexpr int kMaxWindow = 60;
  explicit BaseCal(int window_s = 30, float sample_hz = 25.0f);
  void reset();
  void add(float p_pa);
  // Congela p0 (al despegar) o lo fija desde NVS tras un reinicio en vuelo.
  void freeze() { frozen_ = true; }
  void setFrozen(float p0) { p0_ = p0; frozen_ = true; has_ = true; }
  bool frozen() const { return frozen_; }
  bool ready() const { return frozen_ || filled_ >= window_; }  // ventana completa
  bool hasValue() const { return has_; }
  float p0() const { return p0_; }

 private:
  int window_;
  int per_sec_;
  double acc_ = 0;
  int acc_n_ = 0;
  float ring_[kMaxWindow] = {0};
  int head_ = 0, filled_ = 0;
  float p0_ = 101325.0f;
  bool has_ = false, frozen_ = false;
};
