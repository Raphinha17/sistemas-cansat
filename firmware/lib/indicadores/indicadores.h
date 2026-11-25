// Señales de vida: patrón del LED y texto de las pantallas OLED. Funciones puras (sin
// Arduino), probadas en native. El manejo de pines y de la pantalla está en src/.
#pragma once

#include <stdint.h>

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
// Pantalla OLED 128x64 del firmware de vuelo: solo en AUTOPRUEBA, TIERRA y ATERRIZADO;
// en vuelo (2 a 5) se apaga y no se escribe.
// ---------------------------------------------------------------------------
bool oledActiva(uint8_t estado);

const char *nombreEstado(uint8_t estado);

struct PantallaVuelo {
  char titulo[32];    // "HELLMARY  MODO B"
  char estado[32];    // "EST 1 TIERRA"
  char go[8];         // "GO" o "NO-GO" (se dibuja en grande)
  bool go_ok;
  char sensores[32];  // "IMU OK  BARO OK"
  char tx[32];        // "TX 123"
};

// modo: 'A'..'D'. enviados: paquetes de telemetría enviados.
void pantallaVuelo(uint8_t estado, uint8_t banderas, char modo, uint32_t enviados,
                   PantallaVuelo &out);

// ---------------------------------------------------------------------------
// Pantalla OLED de rx_station
// ---------------------------------------------------------------------------
constexpr uint32_t RX_SIN_ENLACE_MS = 5000;

struct PantallaRx {
  char titulo[32];   // "RX PAQ 123"
  char enlace[32];   // "RSSI -45.5 SNR 9.8" o "SIN ENLACE"
  char estado[32];   // "EST 2 ASCENSO"
  char altura[32];   // "ALT 123.4 m"
  char ultimo[32];   // "ULTIMO HACE 2 s" o "SIN ENLACE"
  bool sin_enlace;
};

// hay_paquete = ya se recibió alguno. desde_ultimo_ms = ms desde el último paquete.
void pantallaRx(uint32_t recibidos, bool hay_paquete, float rssi, float snr, uint8_t estado,
                float altura_m, uint32_t desde_ultimo_ms, PantallaRx &out);
