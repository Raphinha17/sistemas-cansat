// Línea de entrada sintética del modo simulación por serial (docs/sim_input.md).
// Portable (sin Arduino) para probar el formato en native.
//
//   SIM,t_ms,ax,ay,az,gx,gy,gz,p_pa,temp_c[*CRC]
//
// Aceleración en g, giroscopio en °/s (crudo, con sesgo), presión en Pa, temperatura
// en °C. Un campo vacío o "nan" significa lectura fallida: si falta cualquiera de los
// seis del IMU, falla el IMU en ese ciclo; si falta p_pa, falla el barómetro.
//
// *CRC (opcional): '*' y 4 dígitos hexadecimales del CRC-16/CCITT-FALSE (polinomio
// 0x1021, inicial 0xFFFF) de todo lo anterior al '*', empezando por "SIM,". Lo manda
// tools/sim_serial.py; con él la placa descarta las líneas dañadas en el puerto serie.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "flight_core.h"

// CRC-16/CCITT-FALSE de n bytes (igual que binascii.crc_hqx(datos, 0xFFFF) en Python).
uint16_t simCrc16(const char *s, size_t n);

// Agrega "*HHHH" al final de la línea (sin '\n'). false si no cabe en cap bytes.
bool simAppendCrc(char *line, size_t cap);

// Convierte la línea en la entrada de un ciclo (baro_sampled queda en false: lo decide
// quien llama, igual que con los sensores reales). Devuelve false si el formato es
// inválido (prefijo, número de campos, t_ms o CRC incorrecto). has_crc (si no es nulo)
// dice si la línea traía CRC (y era correcto).
bool parseSimLine(const char *line, CoreInput &out, bool *has_crc = nullptr);

// ---------------------------------------------------------------------------
// Integridad del enlace en el modo simulación (docs/sim_input.md, "Integridad").
//
// El puerto serie puede perder bytes (p. ej. mientras la flash está ocupada). Un hueco
// une la cabeza de una línea con la cola de otra; si los campos suman 9, la línea
// "pegada" pasa el formato con valores falsos (una presión de 10 886 Pa = 16 km).
// Además el lazo supone 10 ms entre líneas: una línea que falta es un hueco de tiempo.
//
// SimLink, en la placa, decide qué líneas pasan al lazo de control:
//  - Modo estricto (desde la primera línea con CRC correcto, como las de sim_serial.py):
//    pasan solo las líneas con CRC correcto y t_ms = el esperado (anterior + 10 ms). Una
//    línea dañada, sin CRC o con t_ms adelantado (faltan líneas) se descarta y la placa
//    pide el reenvío desde la esperada ("SIM_REPITE t_ms"); las repetidas (t_ms ya
//    procesado, tras un reenvío) se descartan sin aviso. Así el lazo recibe exactamente
//    la secuencia del CSV, sin huecos, y dt = 10 ms es exacto.
//  - Modo compatible (líneas sin CRC, p. ej. escritas a mano): como antes, pasan todas
//    las de formato válido; los huecos solo se cuentan.
// ---------------------------------------------------------------------------
#define SIM_PERIODO_MS (1000 / CONTROL_HZ)
#define SIM_REPITE_MS 250  // si el reenvío no llega en este tiempo, se vuelve a pedir

class SimLink {
 public:
  enum Result : uint8_t { ACEPTADA, INVALIDA, REPETIDA, ADELANTADA };

  void reset();  // SIM_INICIO / SIM_FIN / activación por la primera línea
  // Línea completa (sin '\n') que empieza con "SIM,". now_ms: reloj real de la placa
  // (solo para no repetir el pedido de reenvío). Con ACEPTADA, out tiene la entrada.
  Result feed(const char *line, uint32_t now_ms, CoreInput &out);
  // Línea descartada antes de llegar a feed(): demasiado larga, texto que no es un
  // comando (fragmento de una línea SIM) o la cola de la placa llena. Cuenta como
  // inválida y, en modo estricto, pide el reenvío.
  void reject(uint32_t now_ms);
  // Pedido de reenvío pendiente (una vez por pedido): t_ms desde el que hay que reenviar.
  bool takeResend(uint32_t &t_ms);

  bool strict() const { return strict_; }
  bool hasNext() const { return has_next_; }
  uint32_t next() const { return next_; }  // t_ms de la próxima línea esperada

  uint32_t accepted() const { return n_ok_; }
  uint32_t invalid() const { return n_bad_; }
  uint32_t duplicates() const { return n_dup_; }  // repetidas descartadas (modo estricto)
  uint32_t gaps() const { return n_gap_; }  // líneas con t_ms adelantado (faltaban otras)
  uint32_t resends() const { return n_resend_; }  // pedidos de reenvío emitidos

 private:
  void requestResend(uint32_t now_ms);

  bool strict_ = false, has_next_ = false;
  uint32_t next_ = 0;
  bool pending_ = false, asked_ = false;  // pedido por enviar / ya pedido para next_
  uint32_t asked_t_ = 0, asked_ms_ = 0;
  uint32_t n_ok_ = 0, n_bad_ = 0, n_dup_ = 0, n_gap_ = 0, n_resend_ = 0;
};
