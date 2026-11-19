import gs_core as C
import telemetry as T


def rx(seq, t_ms=None, h=0.0, vz=0.0, estado=1, flags=0, rssi=-50.0):
    t_ms = seq * 1000 if t_ms is None else t_ms
    return C.RxPacket(T.Packet(estado=estado, secuencia=seq & 0xFFFF, t_ms=t_ms, altura_m=h,
                               vz_ms=vz, banderas=flags), rssi, 8.0)


def feed(st, packets, t0=100.0, dt=1.0):
    now, evs = t0, []
    for p in packets:
        evs += st.on_packet(p, now, 1_700_000_000 + now)
        now += dt
    return evs


def test_sin_perdidas():
    st = C.Stats()
    feed(st, [rx(i) for i in range(50)])
    assert (st.received, st.lost, st.loss_pct) == (50, 0, 0.0)


def test_perdidas_por_secuencia():
    st = C.Stats()
    seqs = [0, 1, 2, 5, 6, 10]            # faltan 3,4 y 7,8,9
    feed(st, [rx(s) for s in seqs])
    assert st.received == 6 and st.lost == 5
    assert abs(st.loss_pct - 100 * 5 / 11) < 1e-9


def test_perdidas_con_vuelta_de_u16():
    st = C.Stats()
    feed(st, [rx(s) for s in (65534, 65535, 0, 3)])  # faltan 1 y 2 tras la vuelta
    assert st.lost == 2 and st.received == 4


def test_duplicados_y_reinicio_de_placa_no_cuentan_como_perdida():
    st = C.Stats()
    feed(st, [rx(10), rx(10), rx(11)])
    assert st.dups == 1 and st.lost == 0
    feed(st, [rx(500), rx(0), rx(1)])            # la placa reinició: secuencia desde 0
    assert st.seq_restarts == 1
    assert st.lost == 488                         # solo el salto 11 -> 500


def test_altura_maxima():
    st = C.Stats()
    assert st.h_max is None
    feed(st, [rx(i, h=h) for i, h in enumerate([0, 10, 452.3, 300, 5])])
    assert st.h_max == 452.3
    assert st.snapshot(110)["h_max"] == 452.3


def test_altura_maxima_con_alturas_negativas():
    st = C.Stats()
    feed(st, [rx(0, h=-3.0), rx(1, h=-1.5)])
    assert st.h_max == -1.5


def test_vz_promedio_de_1s():
    st = C.Stats()
    # 5 Hz: vz = -10 durante 2 s, luego -20 durante 1 s
    pk = [rx(i, t_ms=i * 200, vz=-10.0) for i in range(10)]
    pk += [rx(10 + i, t_ms=(10 + i) * 200, vz=-20.0) for i in range(5)]
    feed(st, pk, dt=0.2)
    # ventana de 1 s por t_ms: solo cuenta lo de los últimos 1000 ms (6 paquetes a 5 Hz)
    assert len(st._vz) == 6
    assert abs(st.vz_avg() - (-10.0 + 5 * -20.0) / 6) < 1e-9


def test_pps():
    st = C.Stats()
    feed(st, [rx(i) for i in range(11)], dt=0.2)    # 5 Hz
    assert abs(st.pps(100.0 + 2.0) - 5.0) < 1e-6
    assert st.pps(200.0) == 0.0                      # callado


def test_enlace_perdido_y_recuperado():
    st = C.Stats()
    assert st.link == "esperando"
    assert st.tick(5.0, 1.0) == []                   # sin paquetes aún: no es "perdido"
    feed(st, [rx(0), rx(1)], t0=10.0)
    assert st.link == "ok"
    assert st.tick(12.5, 2.0) == []                  # 1.5 s de silencio: todavía ok
    ev = st.tick(14.5, 3.0)                          # 3.5 s desde el último (t=11)
    assert st.link == "perdido" and ev and ev[0]["level"] == "bad"
    assert st.tick(20.0, 4.0) == []                  # el aviso sale una sola vez
    ev = st.on_packet(rx(9), 30.0, 5.0)
    assert st.link == "ok"
    assert any(e["kind"] == "enlace" and "recuperado" in e["text"] for e in ev)


def test_eventos_de_estado_y_banderas():
    st = C.Stats()
    ev = feed(st, [rx(0, estado=1, flags=0x01), rx(1, estado=2, flags=0x01),
                   rx(2, estado=3, flags=0x03), rx(3, estado=4, flags=0x07),
                   rx(4, estado=6, flags=0x17)])
    textos = [e["text"] for e in ev]
    for esperado in ("Estado: ASCENSO", "Liberación detectada", "Disparo del servo",
                     "Aterrizaje detectado", "Estado: ATERRIZADO"):
        assert esperado in textos
    assert textos.count("Liberación detectada") == 1   # solo al encenderse
    assert set(st.state_times) == {1, 2, 3, 4, 6}
    assert st.state_times[3]["m"] == 2.0


def test_tiempo_de_mision_y_reinicio_de_placa():
    st = C.Stats()
    feed(st, [rx(0, t_ms=5000), rx(1, t_ms=6000), rx(2, t_ms=9000)])
    assert st.mission_s == 4.0
    ev = st.on_packet(rx(3, t_ms=500), 200.0, 0.0)       # t_ms retrocede: reinició
    assert any(e["kind"] == "placa" for e in ev)
    assert st.mission_s == 4.0                           # continúa desde donde iba
    st.on_packet(rx(4, t_ms=1500), 201.0, 0.0)
    assert st.mission_s == 5.0


def test_invalidos_y_errores_de_radio_se_cuentan():
    st = C.Stats()
    st.on_invalid()
    st.on_invalid()
    st.on_radio_error()
    s = st.snapshot(0.0)
    assert (s["invalid"], s["radio_errors"], s["received"]) == (2, 1, 0)


def test_reset():
    st = C.Stats()
    feed(st, [rx(0, h=50), rx(5, h=60)])
    st.reset()
    s = st.snapshot(1.0)
    assert (s["received"], s["lost"], s["h_max"], s["link"]) == (0, 0, None, "esperando")
