// Línea de entrada sintética del modo simulación por serial (docs/sim_input.md).
// Portable (sin Arduino) para probar el formato en native.
//
//   SIM,t_ms,ax,ay,az,gx,gy,gz,p_pa,temp_c
//
// Aceleración en g, giroscopio en °/s (crudo, con sesgo), presión en Pa, temperatura
// en °C. Un campo vacío o "nan" significa lectura fallida: si falta cualquiera de los
// seis del IMU, falla el IMU en ese ciclo; si falta p_pa, falla el barómetro.
#pragma once

#include <stdint.h>

#include "flight_core.h"

// Convierte la línea en la entrada de un ciclo (baro_sampled queda en false: lo decide
// quien llama, igual que con los sensores reales). Devuelve false si el formato es
// inválido (prefijo, número de campos o t_ms).
bool parseSimLine(const char *line, CoreInput &out);
