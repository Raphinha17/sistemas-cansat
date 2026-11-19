import csv
import os

import gs_core as C
import gs_sources as SRC
import telemetry as T


def _rx(i):
    return C.RxPacket(T.Packet(estado=2, secuencia=i, t_ms=1000 + i * 500, altura_m=10.5 + i,
                               vz_ms=5.25, presion_pa=100000 - i, temp_c=22.5, ax_g=0.01,
                               roll_deg=-12.34, banderas=1, bateria_v=0.0), -61.5, 9.25 + i)


def test_sesion_csv_y_crudo(tmp_path):
    r = C.Recorder(str(tmp_path / "data"))
    assert not r.active
    r.write_packet(_rx(0), 1.0)          # sin grabar: no se escribe nada
    info = r.start(1_700_000_000.0)
    assert info["active"] and os.path.isfile(r.csv_path) and os.path.isfile(r.raw_path)
    for i in range(3):
        r.write_raw(C.make_line(_rx(i).pkt, -61.5, 9.25) + "\r\n")
        r.write_packet(_rx(i), 1_700_000_001.0 + i)
    r.write_raw("linea basura")           # el crudo guarda también lo inválido
    r.stop()
    assert not r.active and r.packets == 3 and r.raw_lines == 4

    rows = list(csv.DictReader(open(r.csv_path, encoding="utf-8")))
    assert list(rows[0].keys()) == C.CSV_FIELDS
    assert len(rows) == 3
    assert float(rows[1]["altura_m"]) == 11.5 and rows[1]["estado_nombre"] == "ASCENSO"
    assert float(rows[2]["rssi_dbm"]) == -61.5 and float(rows[2]["snr_db"]) == 11.25
    assert float(rows[0]["epoch_pc"]) == 1_700_000_001.0 and "T" in rows[0]["hora_pc"]

    raw = open(r.raw_path, encoding="utf-8").read().splitlines()
    assert raw[-1] == "linea basura" and len(raw) == 4
    assert all(C.parse_line(x)[0] == "packet" for x in raw[:3])


def test_dos_sesiones_no_se_pisan(tmp_path):
    r = C.Recorder(str(tmp_path))
    r.start(1_700_000_000.0)
    a = r.csv_path
    r.stop()
    r.start(1_700_000_000.0)             # mismo segundo
    assert r.csv_path != a and os.path.isfile(a)
    r.stop()


def test_csv_se_relee_y_se_repite(tmp_path):
    """Una grabación vuelve a producir las mismas líneas con --replay."""
    r = C.Recorder(str(tmp_path))
    r.start(1_700_000_000.0)
    originales = [_rx(i) for i in range(5)]
    for i, x in enumerate(originales):
        r.write_packet(x, 1_700_000_001.0 + i)
    r.stop()
    rows = C.load_csv(r.csv_path)
    assert [(p, a, b) for p, a, b in rows] == [(x.pkt, x.rssi, x.snr) for x in originales]
    lines = list(SRC.replay_lines(r.csv_path))
    assert [t for t, _ in lines] == [0.0, 0.5, 1.0, 1.5, 2.0]      # reloj de la placa
    assert all(C.parse_line(l)[0] == "packet" for _, l in lines)


def test_csv_con_filas_danadas(tmp_path):
    r = C.Recorder(str(tmp_path))
    r.start()
    r.write_packet(_rx(0), 1.0)
    r.stop()
    with open(r.csv_path, "a", encoding="utf-8") as f:
        f.write("basura,1\n")
    assert len(C.load_csv(r.csv_path)) == 1


def test_simulador_genera_lineas_validas():
    n = 0
    lines = []
    for t, line in SRC.sim_lines(seed=1):
        lines.append((t, line))
        n += 1
        if n >= 300:
            break
    assert all(C.parse_line(l)[0] == "packet" for _, l in lines)
    # con pérdidas y apagón se pierden paquetes y el resto sigue siendo válido
    full = sum(1 for t, _ in SRC.sim_lines(seed=1) if t < 60)
    con = [x for x in SRC.sim_lines(seed=1, loss_pct=20, blackout=(30, 10)) if x[0] < 60]
    assert len(con) < 0.8 * full
    assert not any(30 <= t < 40 for t, _ in con)
