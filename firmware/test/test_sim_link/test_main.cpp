// Integridad del modo simulación por serial (docs/sim_input.md, "Integridad"):
// CRC de línea, detección de huecos de t_ms, pedido de reenvío (SimLink) y un vuelo
// completo a través de un enlace que pierde bloques de bytes, como el puerto serie de la
// placa cuando la flash está ocupada (prueba (b) de docs/PRUEBAS_SISTEMAS.md).
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include <deque>
#include <map>
#include <string>
#include <vector>

#include "../sim_csv.h"
#include "config.h"
#include "flight_core.h"
#include "sim_input.h"

void setUp() {}
void tearDown() {}

static std::string withCrc(const char *body) {
  char b[SIM_LINEA_MAX + 1];
  snprintf(b, sizeof b, "%s", body);
  TEST_ASSERT_TRUE(simAppendCrc(b, sizeof b));
  return b;
}

// ---------------------------------------------------------------------------
// CRC y formato
// ---------------------------------------------------------------------------
static void test_crc_vector() {
  // Valor de referencia de CRC-16/CCITT-FALSE (igual a binascii.crc_hqx(b"123456789", 0xFFFF))
  TEST_ASSERT_EQUAL_HEX16(0x29B1, simCrc16("123456789", 9));
  char small[8] = "SIM,1";
  TEST_ASSERT_FALSE(simAppendCrc(small, sizeof small));  // no cabe
}

static void test_parse_con_crc() {
  const char *body = "SIM,1230,0.01,-0.02,1.003,0.5,-1.25,0.1,100912.5,24.87";
  const std::string ok = withCrc(body);
  CoreInput c;
  bool crc = false;
  TEST_ASSERT_TRUE(parseSimLine(ok.c_str(), c, &crc));
  TEST_ASSERT_TRUE(crc);
  TEST_ASSERT_EQUAL_UINT32(1230, c.t_ms);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 24.87f, c.temp_c);  // el CRC no se mezcla con temp_c
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100912.5f, c.p_pa);
  TEST_ASSERT_TRUE(parseSimLine((ok + "\r\n").c_str(), c, &crc));
  // Minúsculas también valen
  std::string low = ok;
  for (size_t i = low.find('*'); i < low.size(); i++) low[i] = (char)tolower(low[i]);
  TEST_ASSERT_TRUE(parseSimLine(low.c_str(), c, &crc));
  // Sin CRC: válida, pero has_crc = false
  TEST_ASSERT_TRUE(parseSimLine(body, c, &crc));
  TEST_ASSERT_FALSE(crc);
  // Un dígito cambiado en los datos, CRC incompleto, basura después o CRC no hexadecimal
  std::string bad = ok;
  bad[bad.find("100912")] = '2';
  TEST_ASSERT_FALSE(parseSimLine(bad.c_str(), c, &crc));
  TEST_ASSERT_FALSE(parseSimLine(ok.substr(0, ok.size() - 1).c_str(), c, &crc));
  TEST_ASSERT_FALSE(parseSimLine((ok + "7").c_str(), c, &crc));
  std::string nohex = ok;
  nohex[nohex.size() - 1] = 'G';
  TEST_ASSERT_FALSE(parseSimLine(nohex.c_str(), c, &crc));
}

// Lo que pasó en la prueba (b): se pierde un bloque de bytes y la cabeza de una línea se
// une con la cola de otra. Si el corte cae en el mismo campo de las dos, la línea pegada
// tiene 9 campos y, sin CRC, pasa con una presión falsa.
static void test_linea_pegada() {
  const char *a = "SIM,45000,0.0100,0.0200,1.0000,0.500,-1.200,0.100,100886,25.00";
  const char *b = "SIM,45150,0.0110,0.0210,1.0010,0.510,-1.210,0.110,99104,24.00";
  // cabeza de a hasta "...,0.100,10" y cola de b desde "104,24.00"
  const std::string sa(a), sb(b);
  const std::string head = sa.substr(0, sa.rfind(',', sa.rfind(',') - 1) + 3);
  const std::string tail_b = sb.substr(sb.rfind(',', sb.rfind(',') - 1) + 3);
  const std::string pegada = head + tail_b;
  TEST_ASSERT_EQUAL_STRING("SIM,45000,0.0100,0.0200,1.0000,0.500,-1.200,0.100,10104,24.00",
                           pegada.c_str());
  CoreInput c;
  TEST_ASSERT_TRUE(parseSimLine(pegada.c_str(), c));  // formato válido...
  TEST_ASSERT_FLOAT_WITHIN(1, 10104, c.p_pa);         // ...con 10 kPa (~16 km de altura)
  // Con CRC la cola trae el CRC de b, que no corresponde: se descarta.
  const std::string cb = withCrc(b);
  const std::string pegada_crc = head + cb.substr(cb.rfind(',', cb.rfind(',') - 1) + 3);
  TEST_ASSERT_FALSE(parseSimLine(pegada_crc.c_str(), c));
  // Línea truncada (se perdió el final y el '\n' de la siguiente la cerró)
  const std::string ca = withCrc(a);
  TEST_ASSERT_TRUE(parseSimLine(ca.substr(0, ca.size() - 8).c_str(), c));  // sin CRC: pasa
  SimLink L;
  CoreInput in;
  TEST_ASSERT_EQUAL(SimLink::ACEPTADA, L.feed(ca.c_str(), 0, in));        // modo estricto
  TEST_ASSERT_EQUAL(SimLink::INVALIDA, L.feed(ca.substr(0, ca.size() - 8).c_str(), 10, in));
}

// ---------------------------------------------------------------------------
// SimLink
// ---------------------------------------------------------------------------
static std::string lineAt(uint32_t t, bool crc = true) {
  char b[SIM_LINEA_MAX + 1];
  snprintf(b, sizeof b, "SIM,%lu,0.01,0.02,1.0,0.5,-1.2,0.1,%lu,25", (unsigned long)t,
           (unsigned long)(100000 + t % 97));
  if (crc) simAppendCrc(b, sizeof b);
  return b;
}

static void test_link_huecos_y_reenvio() {
  SimLink L;
  CoreInput in;
  uint32_t t = 0;
  TEST_ASSERT_FALSE(L.hasNext());
  TEST_ASSERT_EQUAL(SimLink::ACEPTADA, L.feed(lineAt(100).c_str(), 0, in));
  TEST_ASSERT_TRUE(L.strict());
  TEST_ASSERT_EQUAL_UINT32(110, L.next());
  TEST_ASSERT_EQUAL(SimLink::ACEPTADA, L.feed(lineAt(110).c_str(), 10, in));
  TEST_ASSERT_FALSE(L.takeResend(t));
  // Faltan 120..150: la de 160 se descarta y se pide el reenvío desde 120 (una vez)
  TEST_ASSERT_EQUAL(SimLink::ADELANTADA, L.feed(lineAt(160).c_str(), 20, in));
  TEST_ASSERT_TRUE(L.takeResend(t));
  TEST_ASSERT_EQUAL_UINT32(120, t);
  TEST_ASSERT_FALSE(L.takeResend(t));
  // Mientras el pedido viaja, siguen llegando líneas adelantadas: sin pedir otra vez...
  TEST_ASSERT_EQUAL(SimLink::ADELANTADA, L.feed(lineAt(170).c_str(), 30, in));
  TEST_ASSERT_FALSE(L.takeResend(t));
  // ...hasta que pasan SIM_REPITE_MS sin el reenvío
  TEST_ASSERT_EQUAL(SimLink::ADELANTADA, L.feed(lineAt(180).c_str(), 20 + SIM_REPITE_MS, in));
  TEST_ASSERT_TRUE(L.takeResend(t));
  TEST_ASSERT_EQUAL_UINT32(120, t);
  // Llega el reenvío desde 120: se acepta en orden
  for (uint32_t k = 120; k <= 190; k += 10)
    TEST_ASSERT_EQUAL(SimLink::ACEPTADA, L.feed(lineAt(k).c_str(), 300, in));
  TEST_ASSERT_EQUAL_UINT32(190, in.t_ms);
  // Un segundo reenvío en vuelo (duplicado): se descarta sin aviso
  TEST_ASSERT_EQUAL(SimLink::REPETIDA, L.feed(lineAt(150).c_str(), 310, in));
  TEST_ASSERT_FALSE(L.takeResend(t));
  // Línea dañada: inválida y pide la esperada
  std::string bad = lineAt(200);
  bad[6] = '9';
  TEST_ASSERT_EQUAL(SimLink::INVALIDA, L.feed(bad.c_str(), 320, in));
  TEST_ASSERT_TRUE(L.takeResend(t));
  TEST_ASSERT_EQUAL_UINT32(200, t);
  // En modo estricto una línea sin CRC no pasa
  TEST_ASSERT_EQUAL(SimLink::INVALIDA, L.feed(lineAt(200, false).c_str(), 330, in));
  // Fragmento o línea larga descartados antes de feed(): la esperada no cambia
  L.reject(340);
  TEST_ASSERT_EQUAL_UINT32(200, L.next());
  TEST_ASSERT_EQUAL(SimLink::ACEPTADA, L.feed(lineAt(200).c_str(), 350, in));
  TEST_ASSERT_EQUAL_UINT32(11, L.accepted());
  TEST_ASSERT_EQUAL_UINT32(3, L.invalid());
  TEST_ASSERT_EQUAL_UINT32(3, L.gaps());
  TEST_ASSERT_EQUAL_UINT32(1, L.duplicates());
  TEST_ASSERT_EQUAL_UINT32(3, L.resends());
  // SIM_INICIO
  L.reset();
  TEST_ASSERT_FALSE(L.hasNext());
  TEST_ASSERT_FALSE(L.strict());
  TEST_ASSERT_EQUAL_UINT32(0, L.accepted());
}

static void test_link_modo_compatible() {
  // Líneas sin CRC (escritas a mano o sim_serial viejo): como antes, pasan todas las de
  // formato válido; el hueco solo se cuenta y nunca se pide reenvío.
  SimLink L;
  CoreInput in;
  uint32_t t;
  TEST_ASSERT_EQUAL(SimLink::ACEPTADA, L.feed(lineAt(0, false).c_str(), 0, in));
  TEST_ASSERT_EQUAL(SimLink::ACEPTADA, L.feed(lineAt(50, false).c_str(), 10, in));
  TEST_ASSERT_EQUAL(SimLink::INVALIDA, L.feed("SIM,60,1,2", 20, in));
  L.reject(30);
  TEST_ASSERT_FALSE(L.takeResend(t));
  TEST_ASSERT_FALSE(L.strict());
  TEST_ASSERT_EQUAL_UINT32(1, L.gaps());
  TEST_ASSERT_EQUAL_UINT32(2, L.invalid());
  // Antes de la primera línea aceptada no se sabe qué pedir
  SimLink M;
  M.reject(0);
  TEST_ASSERT_FALSE(M.takeResend(t));
}

// ---------------------------------------------------------------------------
// Vuelo completo por un enlace con pérdidas
// ---------------------------------------------------------------------------
static const std::vector<SimRow> &flight() {
  static std::vector<SimRow> rows = loadSim("flight_seed1");
  TEST_ASSERT_TRUE_MESSAGE(rows.size() > 5000, "no se pudo leer el CSV del simulador");
  return rows;
}

// Igual que tools/sim_serial.py (4 decimales en g, 3 en °/s, presión entera en Pa).
static std::string fmtRow(const SimRow &r, bool crc) {
  char b[SIM_LINEA_MAX + 1];
  const unsigned long t = (unsigned long)lroundf(r.t * 1000);
  bool imu = false;
  for (int k = 0; k < 3; k++) imu |= r.acc[k] != 0 || r.gyro[k] != 0;
  if (imu)
    snprintf(b, sizeof b, "SIM,%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.0f,%.2f", t, r.acc[0],
             r.acc[1], r.acc[2], r.gyro[0], r.gyro[1], r.gyro[2], r.p_pa, r.temp_c);
  else
    snprintf(b, sizeof b, "SIM,%lu,,,,,,,%.0f,%.2f", t, r.p_pa, r.temp_c);
  if (crc) simAppendCrc(b, sizeof b);
  return b;
}

struct Lcg {
  uint32_t s;
  uint32_t range(uint32_t lo, uint32_t hi) {  // [lo, hi]
    s = s * 1664525u + 1013904223u;
    return lo + (s >> 8) % (hi - lo + 1);
  }
};

// Pierde bloques de bytes: cada ~6400 B (~1 s de líneas) se pierden 100 a 1100 B
// seguidos (~10 % del flujo), del orden de lo que mostró la prueba (b):
// 19 391 enviadas, 17 318 + 137 recibidas.
struct LossyLink {
  Lcg rng{12345};
  uint64_t pos = 0, next_drop = 3000;
  uint32_t left = 0;
  uint32_t events = 0;
  bool pass() {
    pos++;
    if (left) {
      left--;
      return false;
    }
    if (pos >= next_drop) {
      events++;
      left = rng.range(100, 1100) - 1;
      next_drop = pos + rng.range(5400, 7400);
      return false;
    }
    return true;
  }
};

// Arma líneas como commsTask (hasta SIM_LINEA_MAX caracteres; más larga = descartada).
struct LineAssembler {
  char line[SIM_LINEA_MAX + 1];
  size_t len = 0;
  bool overflow = false;
  template <class F>
  void byte(char c, F &&onLine) {
    if (c == '\n' || c == '\r') {
      if (len) {
        line[len] = 0;
        onLine(overflow ? nullptr : line);
      }
      len = 0;
      overflow = false;
    } else if (len < SIM_LINEA_MAX) {
      line[len++] = c;
    } else {
      overflow = true;
    }
  }
};

struct LinkRun {
  std::vector<CoreInput> delivered;  // lo que llegó al lazo de control, en orden
  uint32_t ticks = 0;                // tiempo total en periodos de 10 ms
  uint32_t rewinds = 0, loss_events = 0;
  SimLink link;
};

// PC (sim_serial.py) -> enlace con pérdidas -> placa (commsTask + SimLink).
// Una línea por tick de 10 ms; el SIM_REPITE llega a la PC kLat ticks después.
static LinkRun runLink(const std::vector<std::string> &lines, bool lossy) {
  static const uint32_t kLat = 3;
  LinkRun R;
  LossyLink net;
  LineAssembler rx;
  std::map<uint32_t, size_t> idx;
  for (size_t i = 0; i < lines.size(); i++) idx[(uint32_t)atol(lines[i].c_str() + 4)] = i;
  const uint32_t t_last = (uint32_t)atol(lines.back().c_str() + 4);
  struct Nak {
    uint32_t due, t;
  };
  std::deque<Nak> naks;
  uint32_t last_rw_t = 0xFFFFFFFF, last_rw_tick = 0;

  auto onLine = [&](const char *l) {
    const uint32_t now = R.ticks * 10;
    CoreInput in;
    if (!l || strncmp(l, "SIM,", 4) != 0)
      R.link.reject(now);  // línea larga o fragmento
    else if (R.link.feed(l, now, in) == SimLink::ACEPTADA)
      R.delivered.push_back(in);
    uint32_t t;
    if (R.link.takeResend(t)) naks.push_back({R.ticks + kLat, t});
  };

  size_t i = 0;
  while (R.ticks < 4 * lines.size()) {
    // Pedidos de reenvío que ya llegaron a la PC
    while (!naks.empty() && naks.front().due <= R.ticks) {
      const Nak k = naks.front();
      naks.pop_front();
      auto it = idx.find(k.t);
      // Igual que sim_serial.py: ignora el mismo pedido si ya retrocedió hace < 150 ms
      if (it != idx.end() && it->second < i &&
          !(k.t == last_rw_t && R.ticks - last_rw_tick < 15)) {
        i = it->second;
        last_rw_t = k.t;
        last_rw_tick = R.ticks;
        R.rewinds++;
      }
    }
    if (i < lines.size()) {
      const std::string s = lines[i++] + "\n";
      for (char c : s)
        if (!lossy || net.pass()) rx.byte(c, onLine);
      R.ticks++;
      continue;
    }
    if (!naks.empty()) {  // espera los pedidos en camino
      R.ticks++;
      continue;
    }
    // Fin del CSV: SIM_PROXIMA (consulta confiable: sim_serial.py la repite si no hay
    // respuesta). Si faltan las últimas líneas, se reenvían.
    if (!R.link.hasNext() || R.link.next() > t_last) break;
    i = idx.at(R.link.next());
    R.rewinds++;
    R.ticks += kLat;
  }
  R.loss_events = net.events;
  return R;
}

struct Summary {
  std::string states;
  float t_rel = -1, t_fire = -1, h_fire = 0, vz_fire = 0, h_max = 0, t_land = -1;
  float t_asc = -1;
  uint8_t flags = 0;
  int attempts = 0, lowg = 0;
  bool confirmed = false;
  float deploy() const { return h_fire - fabsf(vz_fire) * SM_T_LATENCIA_S; }
};

// Como controlTask en modo simulación: un ciclo por línea, baro en 1 de cada 4.
static Summary runCore(const std::vector<CoreInput> &v, const char *tag,
                       DeployMode mode = DEPLOY_MODE_A) {
  Summary S;
  FlightCore core;
  core.setDeployMode(mode);
  core.begin(v.front().t_ms, true, true);
  int last = -1;
  uint32_t cycle = 0;
  for (CoreInput in : v) {
    in.baro_sampled = (cycle % BARO_DECIM) == 0;
    core.step(in);
    cycle++;
    if (core.state() != last) {
      last = core.state();
      S.states += char('0' + last);
      if (last == ST_CAIDA) S.t_rel = in.t_ms * 0.001f;
      if (last == ST_ASCENSO) S.t_asc = in.t_ms * 0.001f;
      if (last == ST_ATERRIZADO) S.t_land = in.t_ms * 0.001f;
    }
    if (core.evFire() && S.t_fire < 0) {
      S.t_fire = in.t_ms * 0.001f;
      S.h_fire = core.kf().h();
      S.vz_fire = core.kf().vz();
    }
  }
  S.h_max = core.sm().out().h_max;
  S.flags = core.flags();
  S.attempts = core.sm().out().servo_intentos;
  S.confirmed = core.sm().out().servo_confirmado;
  S.lowg = core.sm().out().lowg_descartadas;
  char msg[220];
  snprintf(msg, sizeof msg,
           "%s: lineas=%u estados=%s liberacion=%.2f disparo=%.2f h=%.1f vz=%.1f "
           "h_despliegue=%.1f h_max=%.1f banderas=0x%02X",
           tag, (unsigned)v.size(), S.states.c_str(), S.t_rel, S.t_fire, S.h_fire, S.vz_fire,
           S.deploy(), S.h_max, S.flags);
  TEST_MESSAGE(msg);
  return S;
}

static bool sameInput(const CoreInput &a, const CoreInput &b) {
  if (a.t_ms != b.t_ms || a.imu_valid != b.imu_valid || a.baro_valid != b.baro_valid ||
      a.p_pa != b.p_pa || a.temp_c != b.temp_c)
    return false;
  for (int k = 0; k < 3; k++)
    if (a.acc_g[k] != b.acc_g[k] || a.gyro_dps[k] != b.gyro_dps[k]) return false;
  return true;
}

static void test_vuelo_con_perdidas_crc_y_reenvio() {
  std::vector<std::string> lines;
  for (const SimRow &r : flight()) lines.push_back(fmtRow(r, true));
  const LinkRun clean = runLink(lines, false);
  TEST_ASSERT_EQUAL(lines.size(), clean.delivered.size());
  const LinkRun lossy = runLink(lines, true);

  char msg[220];
  snprintf(msg, sizeof msg,
           "enlace con perdidas: eventos=%u invalidas=%u huecos=%u repetidas=%u "
           "pedidos=%u retrocesos=%u ticks=%u (CSV %u lineas, +%.1f %%)",
           lossy.loss_events, lossy.link.invalid(), lossy.link.gaps(),
           lossy.link.duplicates(), lossy.link.resends(), lossy.rewinds, lossy.ticks,
           (unsigned)lines.size(), 100.0 * lossy.ticks / lines.size() - 100.0);
  TEST_MESSAGE(msg);
  TEST_ASSERT_TRUE(lossy.loss_events > 150);  // ~1 por segundo en 194 s
  TEST_ASSERT_TRUE(lossy.rewinds > 100);
  // El lazo recibe exactamente la misma secuencia que sin pérdidas, sin huecos
  TEST_ASSERT_EQUAL(clean.delivered.size(), lossy.delivered.size());
  for (size_t k = 0; k < clean.delivered.size(); k++)
    if (!sameInput(clean.delivered[k], lossy.delivered[k]))
      TEST_FAIL_MESSAGE("entrada distinta a la del enlace sin perdidas");
  // A tiempo casi real: los reenvíos alargan el envío menos de 30 %
  TEST_ASSERT_TRUE(lossy.ticks < lines.size() * 13 / 10);

  const Summary a = runCore(clean.delivered, "sin perdidas");
  const Summary b = runCore(lossy.delivered, "con perdidas + CRC/reenvio");
  TEST_ASSERT_EQUAL_STRING("0123456", b.states.c_str());
  TEST_ASSERT_EQUAL_FLOAT(a.t_fire, b.t_fire);
  TEST_ASSERT_EQUAL_FLOAT(a.h_fire, b.h_fire);
  TEST_ASSERT_EQUAL_FLOAT(a.h_max, b.h_max);
  TEST_ASSERT_FLOAT_WITHIN(3.0f, 200.0f, b.deploy());
  TEST_ASSERT_EQUAL_HEX8(0x17, b.flags);
}

// El mismo enlace con el formato anterior (sin CRC): reproduce la falla de la prueba (b).
static void test_vuelo_con_perdidas_sin_crc() {
  std::vector<std::string> lines;
  for (const SimRow &r : flight()) lines.push_back(fmtRow(r, false));
  const LinkRun clean = runLink(lines, false);
  const LinkRun lossy = runLink(lines, true);
  std::map<uint32_t, size_t> by_t;
  for (size_t k = 0; k < clean.delivered.size(); k++) by_t[clean.delivered[k].t_ms] = k;
  uint32_t corrupt = 0;
  float worst_p = 0;
  for (const CoreInput &in : lossy.delivered) {
    auto it = by_t.find(in.t_ms);
    if (it == by_t.end() || !sameInput(in, clean.delivered[it->second])) {
      corrupt++;
      if (it != by_t.end())
        worst_p = fmaxf(worst_p, fabsf(in.p_pa - clean.delivered[it->second].p_pa));
    }
  }
  char msg[200];
  snprintf(msg, sizeof msg,
           "sin CRC: enviadas=%u aceptadas=%u invalidas=%u huecos=%u pegadas_aceptadas=%u "
           "(peor error de presion %.0f Pa)",
           (unsigned)lines.size(), (unsigned)lossy.delivered.size(), lossy.link.invalid(),
           lossy.link.gaps(), corrupt, worst_p);
  TEST_MESSAGE(msg);
  TEST_ASSERT_FALSE(lossy.link.strict());
  TEST_ASSERT_EQUAL_UINT32(0, lossy.rewinds);  // sin CRC la placa nunca pide reenvío
  TEST_ASSERT_TRUE(lossy.delivered.size() < lines.size() * 95 / 100);
  TEST_ASSERT_TRUE(lossy.link.gaps() > 100);
  TEST_ASSERT_TRUE(corrupt > 0);  // líneas pegadas que pasaron el formato
  runCore(lossy.delivered, "con perdidas, sin CRC (formato anterior)");
}

// Solo huecos (líneas completas que faltan, sin líneas pegadas), formato anterior: el lazo
// supone 10 ms por línea aunque el t_ms salte. Se pierden 10 de cada 100 líneas.
static void test_huecos_sin_lineas_pegadas() {
  std::vector<CoreInput> all, holes;
  for (const SimRow &r : flight()) {
    CoreInput in;
    TEST_ASSERT_TRUE(parseSimLine(fmtRow(r, false).c_str(), in));
    all.push_back(in);
  }
  for (size_t k = 0; k < all.size(); k++)
    if (k % 100 < 90 || k < 3000) holes.push_back(all[k]);  // calibración completa
  const Summary a = runCore(all, "completo");
  const Summary b = runCore(holes, "10 % de lineas faltantes (huecos de 100 ms)");
  // Sin valores falsos el vuelo sigue en orden; el error queda en la altura de despliegue.
  TEST_ASSERT_EQUAL_STRING("0123456", b.states.c_str());
  TEST_ASSERT_FLOAT_WITHIN(3.0f, a.deploy(), b.deploy());  // ~1 m: no explica el +130 m/s
  char msg[120];
  snprintf(msg, sizeof msg, "diferencia de h_despliegue por los huecos: %.1f m",
           b.deploy() - a.deploy());
  TEST_MESSAGE(msg);
}

// Los vuelos de docs/PRUEBAS_SISTEMAS.md con el mismo texto que manda sim_serial.py (con
// CRC, sin pérdidas): de aquí salen los números esperados de la guía.
static Summary guia(const char *csv, const char *prueba, DeployMode mode) {
  const std::vector<SimRow> rows = loadSim(csv);
  TEST_ASSERT_TRUE_MESSAGE(rows.size() > 5000, "no se pudo leer el CSV del simulador");
  std::vector<CoreInput> v;
  for (const SimRow &r : rows) {
    CoreInput in;
    TEST_ASSERT_TRUE(parseSimLine(fmtRow(r, true).c_str(), in));
    v.push_back(in);
  }
  const Summary S = runCore(v, prueba, mode);
  char msg[300];
  snprintf(msg, sizeof msg,
           "GUIA %s (%s, modo %c): lineas=%u duracion=%.1f s ASCENSO=%.2f CAIDA=%.2f "
           "DISPARO=%.2f (h=%.1f vz=%.1f h_despliegue=%.1f) lib->disp=%.2f s ATERRIZADO=%.2f "
           "h_max=%.1f banderas=0x%02X intentos=%d confirmado=%d descartadas=%d",
           prueba, csv, deployModeChar(mode), (unsigned)rows.size(), rows.back().t, S.t_asc,
           S.t_rel, S.t_fire, S.h_fire, S.vz_fire, S.deploy(), S.t_fire - S.t_rel, S.t_land,
           S.h_max, S.flags, S.attempts, (int)S.confirmed, S.lowg);
  TEST_MESSAGE(msg);
  return S;
}

static void test_guia_pruebas_sistemas() {
  const Summary b = guia("mode_b_seed1", "(b) B nominal", DEPLOY_MODE_B);
  TEST_ASSERT_EQUAL_STRING("0123456", b.states.c_str());
  TEST_ASSERT_FLOAT_WITHIN(3.0f, 80.0f, b.deploy());
  TEST_ASSERT_EQUAL_HEX8(0x17, b.flags);
  const float alts[] = {90, 95, 105};
  const char *csvs[] = {"mode_b_90", "mode_b_95", "mode_b_105"};
  for (int k = 0; k < 3; k++) {
    char tag[40];
    snprintf(tag, sizeof tag, "(b2) B suelta %.0f m", alts[k]);
    const Summary x = guia(csvs[k], tag, DEPLOY_MODE_B);
    TEST_ASSERT_EQUAL_STRING("0123456", x.states.c_str());
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 80.0f, x.deploy());
    TEST_ASSERT_EQUAL_HEX8(0x17, x.flags);
  }
  const Summary ti = guia("mode_b_tilt", "(b3) B caida inclinada", DEPLOY_MODE_B);
  TEST_ASSERT_EQUAL_HEX8(0x1F, ti.flags);
  const Summary lo = guia("mode_b_slow", "(b4) B ascenso lento", DEPLOY_MODE_B);
  TEST_ASSERT_EQUAL_STRING("0123456", lo.states.c_str());
  TEST_ASSERT_FLOAT_WITHIN(3.0f, 80.0f, lo.deploy());
  const Summary a = guia("flight_seed1", "(c) A nominal", DEPLOY_MODE_A);
  TEST_ASSERT_FLOAT_WITHIN(3.0f, 200.0f, a.deploy());
  TEST_ASSERT_EQUAL_HEX8(0x17, a.flags);
  const Summary ab = guia("mode_b_seed1", "(c) contraprueba: vuelo B en modo A", DEPLOY_MODE_A);
  TEST_ASSERT_EQUAL_STRING("012", ab.states.c_str());
  const Summary c = guia("tinaco", "(d) C tinaco", DEPLOY_MODE_C);
  TEST_ASSERT_EQUAL_STRING("013456", c.states.c_str());
  const Summary e = guia("mode_b_false_hover", "(e) B falsa liberacion", DEPLOY_MODE_B);
  TEST_ASSERT_EQUAL(1, e.lowg);
  TEST_ASSERT_FLOAT_WITHIN(3.0f, 80.0f, e.deploy());
  const Summary f = guia("mode_b_fail_baro", "(f) B falla baro", DEPLOY_MODE_B);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, DEPLOY_T_SIN_BARO_B_S, f.t_fire - f.t_rel);
  TEST_ASSERT_EQUAL_HEX8(0x3F, f.flags);
  const Summary g = guia("mode_b_fail_imu", "(g) B falla IMU", DEPLOY_MODE_B);
  TEST_ASSERT_FLOAT_WITHIN(3.0f, 80.0f, g.deploy());
  TEST_ASSERT_EQUAL_HEX8(0x57, g.flags);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_crc_vector);
  RUN_TEST(test_parse_con_crc);
  RUN_TEST(test_linea_pegada);
  RUN_TEST(test_link_huecos_y_reenvio);
  RUN_TEST(test_link_modo_compatible);
  RUN_TEST(test_vuelo_con_perdidas_crc_y_reenvio);
  RUN_TEST(test_vuelo_con_perdidas_sin_crc);
  RUN_TEST(test_huecos_sin_lineas_pegadas);
  RUN_TEST(test_guia_pruebas_sistemas);
  return UNITY_END();
}
