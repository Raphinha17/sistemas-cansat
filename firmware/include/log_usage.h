// Ocupación del log (portable, probada en native). Umbral del aviso de arranque.
#pragma once

#include <stddef.h>

#define LOG_AVISO_PCT 50

// Porcentaje entero (redondeado hacia abajo) de used sobre cap; 0 si cap es 0.
inline unsigned logPercent(size_t used, size_t cap) {
  return cap ? (unsigned)((unsigned long long)used * 100ull / cap) : 0u;
}
// El aviso se da cuando la ocupación supera el 50 % (estrictamente).
inline bool logNeedsWarning(size_t used, size_t cap) {
  return cap && (unsigned long long)used * 100ull > (unsigned long long)cap * LOG_AVISO_PCT;
}
