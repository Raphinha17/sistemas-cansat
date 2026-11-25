#include "indicadores.h"

#include <stdarg.h>
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

static void agrega(Pantalla &p, int x, int y, int tam, bool inv, const char *fmt, ...)
    __attribute__((format(printf, 6, 7)));

static void agrega(Pantalla &p, int x, int y, int tam, bool inv, const char *fmt, ...) {
  if (p.n >= sizeof p.t / sizeof p.t[0]) return;
  Texto &t = p.t[p.n++];
  t.x = (int8_t)x;
  t.y = (int8_t)y;
  t.tam = (uint8_t)tam;
  t.invertido = inv;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(t.s, sizeof t.s, fmt, ap);  // recorta a OLED_COLUMNAS caracteres
  va_end(ap);
}

void pantallaVuelo(uint8_t estado, uint8_t b, char modo, Pantalla &o) {
  o.n = 0;
  const bool ok = !hayFallo(b);
  const char *go = ok ? "GO" : "NO-GO";
  const int w = (int)strlen(go) * OLED_FUENTE_ANCHO * 2;  // tamaño 2: 12x16
  agrega(o, (OLED_ANCHO - w) / 2, 0, 2, !ok, "%s", go);
  agrega(o, 0, 16, 1, false, "MODO %c  S%u", modo, (unsigned)estado);
  agrega(o, 0, 24, 1, false, "I:%s B:%s", (b & FLAG_FALLO_IMU) ? "NO" : "OK",
         (b & FLAG_FALLO_BARO) ? "NO" : "OK");
}

void pantallaRx(uint32_t rec, bool hay, float rssi, float snr, uint8_t estado, float h,
                uint32_t desde_ms, Pantalla &o) {
  o.n = 0;
  const bool sin = !hay || desde_ms > RX_SIN_ENLACE_MS;
  if (rec > 999999) agrega(o, 0, 0, 1, false, "RX 999999+");
  else agrega(o, 0, 0, 1, false, "RX %lu", (unsigned long)rec);
  if (!hay) {
    agrega(o, 0, 8, 1, false, "--/--dB");
    agrega(o, 0, 16, 1, false, "ST- H--");
  } else {
    agrega(o, 0, 8, 1, false, "%.0f/%.0fdB", rssi, snr);
    // altura: 1 decimal solo si |h| < 10 (para que quepa en 10 caracteres)
    if (h > -10.0f && h < 10.0f) agrega(o, 0, 16, 1, false, "ST%u H%.1fm", (unsigned)estado, h);
    else agrega(o, 0, 16, 1, false, "ST%u H%.0fm", (unsigned)estado, h);
  }
  if (sin) agrega(o, 0, 24, 1, true, "SIN ENLACE");
  else agrega(o, 0, 24, 1, false, "ULT %lus", (unsigned long)(desde_ms / 1000));
}
