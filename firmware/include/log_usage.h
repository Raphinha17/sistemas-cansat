// Ocupación del log (portable, probada en native). Umbral del aviso de arranque.
#pragma once

#include <stddef.h>

#define LOG_AVISO_PCT 50

// Espacio libre que debe haber al empezar el vuelo (50 Hz x 32 B x ~300 s = 480 KB, con
// margen). Si hay menos, se vacía el log de arranque/pruebas para no perder datos de vuelo.
#define LOG_RESERVA_VUELO_B (600u * 1024u)

// Porcentaje entero (redondeado hacia abajo) de used sobre cap; 0 si cap es 0.
inline unsigned logPercent(size_t used, size_t cap) {
  return cap ? (unsigned)((unsigned long long)used * 100ull / cap) : 0u;
}
// true si el espacio libre no alcanza para un vuelo completo.
inline bool logNeedsRoom(size_t used, size_t cap) {
  return used + LOG_RESERVA_VUELO_B > cap;
}
// El aviso se da cuando la ocupación supera el 50 % (estrictamente).
inline bool logNeedsWarning(size_t used, size_t cap) {
  return cap && (unsigned long long)used * 100ull > (unsigned long long)cap * LOG_AVISO_PCT;
}
