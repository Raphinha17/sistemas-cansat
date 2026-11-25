// Señales de vida: patrón del LED y texto de las pantallas OLED. Funciones puras (sin
// Arduino), probadas en native. El manejo de pines y de la pantalla está en src/.
#pragma once

#include <stdint.h>

#include "config.h"
#include "packet.h"

// ---------------------------------------------------------------------------
// LED de la placa (solo firmware de vuelo; el modo demo/banco usa su propio patrón)
//   AUTOPRUEBA              parpadeo muy rápido (10 Hz)
//   TIERRA sin fallos       doble destello cada 2 s
//   TIERRA con fallo/NO-GO  parpadeo rápido 5 Hz
//   ASCENSO..DESCENSO       apagado
//   ATERRIZADO              un destello por segundo
// "Fallo" = bandera de fallo de barómetro, de IMU o NO-GO (bits 5, 6 y 7).
// ---------------------------------------------------------------------------
bool ledOn(uint8_t estado, uint8_t banderas, uint32_t t_ms);

// Hay un fallo (bits 5, 6 o 7 de las banderas).
bool hayFallo(uint8_t banderas);

// ---------------------------------------------------------------------------
// Pantallas OLED de 64x32 píxeles visibles (Wireless Stick V3), fuente 6x8: 10 caracteres por
// línea y 4 líneas. Cada pantalla es una lista de textos con posición y tamaño; el dibujo
// real (src/common/oled_hw.cpp) los pinta tal cual. Los textos nunca pasan de OLED_COLUMNAS
// caracteres (a tamaño 1) y todo cae dentro del área visible (probado en native).
// Firmware de vuelo: solo en los estados 0, 1 y 6; en vuelo se apaga y no se escribe.
// ---------------------------------------------------------------------------
bool oledActiva(uint8_t estado);

const char *nombreEstado(uint8_t estado);

struct Texto {
  int8_t x, y;       // esquina superior izquierda (píxeles)
  uint8_t tam;       // 1 = 6x8, 2 = 12x16
  bool invertido;    // texto negro sobre fondo blanco (un rectángulo del ancho del texto)
  char s[OLED_COLUMNAS + 1];
};

struct Pantalla {
  Texto t[6];
  uint8_t n = 0;
};

// Pantalla del firmware de vuelo. modo: 'A'..'D'.
//   GO / NO-GO grande (NO-GO = banderas 5, 6 o 7), "MODO B  S1" y "I:OK B:OK"
//   (I = IMU, B = barómetro; "I:NO" si falla).
void pantallaVuelo(uint8_t estado, uint8_t banderas, char modo, Pantalla &out);

// ---------------------------------------------------------------------------
// Pantalla de rx_station
// ---------------------------------------------------------------------------
constexpr uint32_t RX_SIN_ENLACE_MS = 5000;

//   "RX 123"       paquetes recibidos
//   "-45/12dB"     RSSI (dBm) / SNR (dB)
//   "ST1 H0.0m"    último estado y altura
//   "ULT 2s"       segundos desde el último paquete, o "SIN ENLACE" (> 5 s o sin paquetes)
void pantallaRx(uint32_t recibidos, bool hay_paquete, float rssi, float snr, uint8_t estado,
                float altura_m, uint32_t desde_ultimo_ms, Pantalla &out);
