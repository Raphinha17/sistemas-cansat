#include "indicadores.h"

#include <stdio.h>
#include <string.h>

bool hayFallo(uint8_t b) { return (b & (FLAG_FALLO_BARO | FLAG_FALLO_IMU | FLAG_NO_GO)) != 0; }

bool ledOn(uint8_t estado, uint8_t banderas, uint32_t t_ms) {
  switch (estado) {
    case ST_AUTOPRUEBA:
      return (t_ms % 100) < 50;                       // 10 Hz
    case ST_TIERRA:
      if (hayFallo(banderas)) return (t_ms % 200) < 100;  // 5 Hz
      {
        const uint32_t f = t_ms % 2000;               // doble destello cada 2 s
        return f < 100 || (f >= 250 && f < 350);
      }
    case ST_ATERRIZADO:
      return (t_ms % 1000) < 100;                     // un destello por segundo
    default:
      return false;                                   // vuelo: apagado
  }
}

bool oledActiva(uint8_t estado) {
  return estado == ST_AUTOPRUEBA || estado == ST_TIERRA || estado == ST_ATERRIZADO;
}

const char *nombreEstado(uint8_t e) {
  static const char *n[] = {"AUTOPRUEBA", "TIERRA", "ASCENSO", "CAIDA",
                            "DESPLIEGUE", "DESCENSO", "ATERRIZADO"};
  return e < 7 ? n[e] : "?";
}

void pantallaVuelo(uint8_t estado, uint8_t b, char modo, uint32_t enviados, PantallaVuelo &o) {
  snprintf(o.titulo, sizeof o.titulo, "HELLMARY  MODO %c", modo);
  snprintf(o.estado, sizeof o.estado, "EST %u %s", (unsigned)estado, nombreEstado(estado));
  o.go_ok = !hayFallo(b);
  snprintf(o.go, sizeof o.go, "%s", o.go_ok ? "GO" : "NO-GO");
  snprintf(o.sensores, sizeof o.sensores, "IMU %s  BARO %s",
           (b & FLAG_FALLO_IMU) ? "FALLA" : "OK", (b & FLAG_FALLO_BARO) ? "FALLA" : "OK");
  snprintf(o.tx, sizeof o.tx, "TX %lu", (unsigned long)enviados);
}

void pantallaRx(uint32_t rec, bool hay, float rssi, float snr, uint8_t estado, float h,
                uint32_t desde_ms, PantallaRx &o) {
  snprintf(o.titulo, sizeof o.titulo, "RX PAQ %lu", (unsigned long)rec);
  o.sin_enlace = !hay || desde_ms > RX_SIN_ENLACE_MS;
  if (!hay) {
    snprintf(o.enlace, sizeof o.enlace, "RSSI --  SNR --");
    snprintf(o.estado, sizeof o.estado, "EST --");
    snprintf(o.altura, sizeof o.altura, "ALT --");
    snprintf(o.ultimo, sizeof o.ultimo, "SIN ENLACE");
    return;
  }
  snprintf(o.enlace, sizeof o.enlace, "RSSI %.1f SNR %.1f", rssi, snr);
  snprintf(o.estado, sizeof o.estado, "EST %u %s", (unsigned)estado, nombreEstado(estado));
  snprintf(o.altura, sizeof o.altura, "ALT %.1f m", h);
  if (o.sin_enlace)
    snprintf(o.ultimo, sizeof o.ultimo, "SIN ENLACE");
  else
    snprintf(o.ultimo, sizeof o.ultimo, "ULTIMO HACE %lu s", (unsigned long)(desde_ms / 1000));
}
