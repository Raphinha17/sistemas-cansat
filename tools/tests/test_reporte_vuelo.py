"""Pruebas de tools/reporte_vuelo.py: cálculos de la tabla, datos ruidosos, log interno y
reporte completo con vuelos de tools/flight_sim.py (modo B)."""
import csv
import math
import os
import random

import pytest

import flight_sim as S
import reporte_vuelo as R
import resumen_enlace as RE
import telemetry as T

CAB = ["hora_pc", "epoch_pc"] + T.field_names() + ["rssi_dbm", "snr_db", "estado_nombre"]
EPOCH0 = 1795130820.0


def _reg(t, estado, h, vz, a=1.0, seq=0, flags=0, rssi=-80.0, snr=9.0, p=100000):
    return {"t_ms": int(round(t * 1000)), "estado": estado, "secuencia": seq, "banderas": flags,
            "hora_pc": "", "presion_pa": p, "temp_c": 20.0, "altura_m": h, "vz_ms": vz,
            "ax_g": 0.0, "ay_g": 0.0, "az_g": a, "roll_deg": 1.0, "pitch_deg": 2.0,
            "yaw_deg": 3.0, "bateria_v": 9.0, "rssi_dbm": rssi, "snr_db": snr,
            "epoch_pc": EPOCH0 + t}


def vuelo_sintetico(fin=95.8, perder=range(100, 105)):
    """Vuelo a 5 Hz con números redondos:
    TIERRA 0-10 s; ASCENSO a 5 m/s hasta 250 m (t=60) y 2 s quieto; CAIDA a 30 m/s hasta
    100 m (t=67); DESPLIEGUE a 20 m/s hasta 80 m (t=68); DESCENSO a 10 m/s hasta el suelo
    (t=76, impacto de 5 g); ATERRIZADO hasta `fin`. Se pierden las secuencias `perder`."""
    regs = []
    for k in range(int(round(fin * 5)) + 1):
        t = k / 5
        if t < 10:
            e, h, vz = T.TIERRA, 0.0, 0.0
        elif t < 60:
            e, h, vz = T.ASCENSO, 5 * (t - 10), 5.0
        elif t < 62:
            e, h, vz = T.ASCENSO, 250.0, 0.0
        elif t < 67:
            e, h, vz = T.CAIDA, 250 - 30 * (t - 62), -30.0
        elif t < 68:
            e, h, vz = T.DESPLIEGUE, 100 - 20 * (t - 67), -20.0
        elif t < 76:
            e, h, vz = T.DESCENSO, 80 - 10 * (t - 68), -10.0
        else:
            e, h, vz = T.ATERRIZADO, 0.0, 0.0
        a = 5.0 if abs(t - 76) < 1e-9 else 1.0
        if k not in perder:
            regs.append(_reg(t, e, round(h, 3), vz, a=a, seq=k))
    return regs


def test_numeros_de_la_tabla():
    d = R.construir(vuelo_sintetico())
    r = R.analizar(d)
    assert r["h_max"]["altura_m"] == 250 and r["h_max"]["t"] == 60
    assert (r["liberacion"]["t"], r["liberacion"]["altura_m"]) == (62, 250)
    assert r["despliegue"]["t"] == 67 and r["despliegue"]["altura_m"] == pytest.approx(100)
    assert r["fin_despliegue"]["t"] == 68 and r["fin_despliegue"]["altura_m"] == pytest.approx(80)
    assert r["despegue"]["t"] == 10
    assert r["aterrizaje"]["t"] == 76 and r["aterrizaje"]["metodo"] == "pico de aceleración"
    assert r["duracion_mision_s"] == pytest.approx(66)
    assert r["tiempo_descenso_s"] == pytest.approx(8)
    assert r["tiempo_caida_total_s"] == pytest.approx(14)
    assert r["vz_max_caida"]["vz_ms"] == 30
    ds = r["descenso"]
    assert ds["media_ms"] == pytest.approx(10) and ds["mediana_ms"] == pytest.approx(10)
    assert ds["en_banda"] and ds["pct_en_banda"] == 100 and ds["n"] == 40
    assert ds["media_altura_tiempo_ms"] == pytest.approx(10)
    assert r["impacto"]["g"] == 5
    assert r["post_aterrizaje"]["s"] == pytest.approx(19.8) and r["post_aterrizaje"]["cumple"]
    e = r["enlace"]
    assert (e["recibidos"], e["esperados"], e["perdidos"]) == (475, 480, 5)
    assert e["perdida_pct"] == pytest.approx(100 * 5 / 480)
    assert r["rssi"]["prom"] == -80 and r["snr"]["prom"] == 9
    assert r["huecos"]["n"] == 0            # 5 paquetes perdidos = 1.2 s: no es hueco
    filas = dict(R.filas_tabla(r, R.Reloj(d)))
    assert filas["¿Descenso dentro de 8 a 12 m/s?"].startswith("Sí")
    assert filas["¿Siguió transmitiendo ≥ 10 s después de aterrizar?"] == "Sí, 19.8 s"
    assert filas["Paquetes recibidos / esperados"] == "475 / 480"


def test_descenso_fuera_de_banda_y_poca_transmision():
    regs = vuelo_sintetico(fin=80.0, perder=())
    for x in regs:                       # autogiro lento: 14 m/s
        if x["estado"] == T.DESCENSO:
            x["vz_ms"] = -14.0
    r = R.analizar(R.construir(regs))
    assert r["descenso"]["media_ms"] == pytest.approx(14) and not r["descenso"]["en_banda"]
    assert r["post_aterrizaje"]["s"] == pytest.approx(4) and not r["post_aterrizaje"]["cumple"]
    assert r["enlace"]["perdidos"] == 0
    filas = dict(R.filas_tabla(r, R.Reloj(R.construir(regs))))
    assert filas["¿Siguió transmitiendo ≥ 10 s después de aterrizar?"] == "No, 4.0 s"


def test_toque_sin_pico_de_aceleracion():
    regs = vuelo_sintetico(perder=())
    for x in regs:
        x["az_g"] = 1.0
    r = R.analizar(R.construir(regs))
    # primera muestra a 2 m o menos del suelo: t = 75.8 (h = 2.0)
    assert r["aterrizaje"]["t"] == pytest.approx(75.8)
    assert r["aterrizaje"]["metodo"].startswith("altura")


def test_datos_ruidosos_y_faltantes():
    regs = vuelo_sintetico(perder=())
    regs[300]["altura_m"] = 900.0               # pico de un paquete (t=60, quieto a 250 m)
    for x in regs[:20]:
        x["presion_pa"] = 0                     # sin barómetro en tierra
        x["rssi_dbm"] = math.nan
    for x in regs[340:345]:                     # estimador corrompido (vz saturada)
        x["vz_ms"], x["altura_m"] = -327.68, -2197.2
    regs[200]["ax_g"] = regs[200]["ay_g"] = regs[200]["az_g"] = 0.0
    regs.insert(10, dict(regs[9]))              # duplicado
    d = R.construir(regs)
    q = d.calidad
    assert q["duplicados"] == 1 and q["presion_invalida"] == 20
    assert q["estimador_fuera_de_rango"] == 5 and q["saltos_de_altura"] >= 1
    assert q["imu_sin_dato"] == 1
    r = R.analizar(d)
    assert r["h_max"]["altura_m"] == 250        # el pico de 900 m se descarta
    assert r["rssi"]["prom"] == -80             # los NaN no cuentan
    assert r["descenso"]["en_banda"]
    notas = R.notas_calidad(d, r)
    assert any("duplicados" in n for n in notas) and any("saltos" in n for n in notas)


def test_salto_largo_tras_hueco_se_acepta():
    # 12 s sin paquetes en la caída: la altura baja 300 m de golpe, pero es real
    regs = [_reg(t / 5, T.CAIDA, 400 - 25 * (t / 5), -25.0, seq=t) for t in range(0, 101)
            if not 10 <= t < 70]
    d = R.construir(regs)
    assert not d.calidad.get("saltos_de_altura")
    assert R.huecos(d.t) == [(pytest.approx(1.8), pytest.approx(14.0))]


def test_sesion_sin_vuelo_no_falla():
    regs = [_reg(k, T.TIERRA, 0.0, 0.0, seq=k, flags=0xE0, p=0) for k in range(30)]
    d = R.construir(regs)
    r = R.analizar(d)
    for k in ("liberacion", "despliegue", "aterrizaje", "descenso", "post_aterrizaje"):
        assert not r.get(k)
    filas = dict(R.filas_tabla(r, R.Reloj(d)))
    assert filas["Liberación: hora · altura"] == "no detectada"
    assert filas["Duración de la misión (despegue → aterrizaje)"] == "—"
    assert R.analizar(R.construir([]))["n"] == 0


def _fila(estado=1, seq=0, t_ms=0, h=0.0, rssi=-70.0, snr=8.0):
    """Fila del CSV de la estación (orden de CAB)."""
    q = dict(estado=estado, secuencia=seq, t_ms=t_ms, presion_pa=100000, temp_c=20,
             altura_m=h, vz_ms=0, ax_g=0, ay_g=0, az_g=1, roll_deg=0, pitch_deg=0, yaw_deg=0,
             banderas=0, bateria_v=9, version=1)
    return ["", ""] + [q[n] for n in T.field_names()] + [rssi, snr, "TIERRA"]


def test_enlace_igual_que_resumen_enlace(tmp_path):
    p = tmp_path / "s.csv"
    seqs = [65530, 65531, 65533, 65535, 2, 2, 3, 10, 0, 1]   # vuelta, duplicado y reinicio
    with open(p, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(CAB)
        for i, s in enumerate(seqs):
            w.writerow(_fila(seq=s, t_ms=1000 * i, rssi=-70 - i))
    a, b = RE.resumen(str(p)), R.analizar(R.cargar(str(p)))["enlace"]
    assert (b["recibidos"], b["esperados"]) == (9, 19)     # 1+1+2+2+3+1+7+1(reinicio)+1
    assert (a["recibidos"], a["esperados"]) == (b["recibidos"], b["esperados"])
    assert a["perdida_pct"] == pytest.approx(b["perdida_pct"])


def test_csv_con_filas_danadas(tmp_path):
    p = tmp_path / "s.csv"
    with open(p, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(CAB)
        w.writerow(_fila(seq=0, t_ms=1000, h=1.5))
        w.writerow(["basura"])
        w.writerow(_fila(estado=9, seq=1, t_ms=2000))              # estado 9: inválido
        w.writerow(_fila(seq=2, t_ms=3000, h="", rssi="", snr=""))
    d = R.cargar(str(p))
    assert d.n == 2 and d.calidad["filas_ilegibles"] == 2
    assert math.isnan(d["altura_m"][1]) and math.isnan(d["rssi_dbm"][1])


def test_leer_dump_hex(tmp_path):
    p = tmp_path / "dump.txt"
    ev = bytes([T.LOG_EVENT_MARK, T.LOG_EV_ARRANQUE, ord("B"), 0]) + (500).to_bytes(4, "little") + bytes(24)
    lines = ["DUMP", "LOG_INICIO", ev.hex().upper()]
    for k in range(5):
        lines.append(T.encode(T.Packet(estado=2, secuencia=k, t_ms=1000 + 20 * k,
                                       altura_m=10 + k)).hex().upper())
    lines += ["zz", lines[-1][:40], "LOG_FIN"]
    p.write_text("\n".join(lines) + "\n")
    regs, bad = R.leer_registros(str(p))
    assert len(regs) == 5 and bad == 1           # el hex cortado; "DUMP" y "zz" se ignoran
    assert regs[4]["altura_m"] == pytest.approx(14) and regs[0]["t_ms"] == 1000


def test_log_alineado_y_no_recibido():
    log = R.construir([_reg(k / 50, T.DESCENSO, 100 - k / 5, -10, seq=k) for k in range(500)])
    est_regs = [_reg(k / 5, T.DESCENSO, 100 - 2 * k, -10, seq=k) for k in range(50)
                if not 20 <= k < 30]             # la estación no recibió de t=4 a t=6
    est = R.construir(est_regs)
    assert R.alinear_log(est, log) == pytest.approx(0)
    m = R.no_recibido(est, log.t)
    tr = R.tramos(log.t, m)
    assert tr[0][0] == pytest.approx(3.82) and tr[0][1] == pytest.approx(5.98)
    assert tr[-1][0] == pytest.approx(9.82)      # después del último paquete recibido


# ---------------------------------------------------------------------------
# Vuelos de tools/flight_sim.py -> CSV como los graba la estación (+ log interno)
# ---------------------------------------------------------------------------
def vuelo_simulado(carpeta, nombre, perdida=5.0, apagon=None, log=True, **kw):
    """Corre flight_sim a 50 Hz (el log interno de la placa) y submuestrea con la
    frecuencia de transmisión por estado lo que llegaría a la estación, con pérdidas."""
    rnd = random.Random(7)
    est_rows, log_rows = [], []
    next_tx, seq = 0.0, 0
    for t, p in S.simulate(seed=3, rate=50, **kw):
        log_rows.append(p)
        if t + 1e-9 < next_tx:
            continue
        next_tx = t + 1.0 / S.RATES_HZ[p.estado]
        q = T.decode(T.encode(p))
        q.secuencia = seq & 0xFFFF
        seq += 1
        if apagon and apagon[0] <= t < apagon[1]:
            continue
        if rnd.random() * 100 < perdida:
            continue
        est_rows.append((t, q, -60 - 0.05 * q.altura_m + rnd.gauss(0, 1), 9 + rnd.gauss(0, 0.5)))
    csv_path = os.path.join(carpeta, f"sesion_{nombre}.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(CAB)
        for t, q, rssi, snr in est_rows:
            wall = EPOCH0 + t + 0.05
            from datetime import datetime
            w.writerow([datetime.fromtimestamp(wall).isoformat(timespec="milliseconds"),
                        f"{wall:.3f}"] + [getattr(q, n) for n in T.field_names()]
                       + [round(rssi, 1), round(snr, 2), T.STATE_NAMES[q.estado]])
    log_path = None
    if log:
        log_path = os.path.join(carpeta, f"log_{nombre}.csv")
        with open(log_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(T.field_names())
            for p in log_rows:
                q = T.decode(T.encode(p))
                w.writerow([getattr(q, n) for n in T.field_names()])
    return csv_path, log_path


def test_reporte_modo_b_100m_con_log_y_pdf(tmp_path):
    pytest.importorskip("matplotlib")
    csv_path, log_path = vuelo_simulado(str(tmp_path), "20261201_100000", perdida=5,
                                        apagon=(40.0, 42.0), **S.MODES["B"])
    carpeta, r = R.generar_reporte(csv_path, log_path, con_pdf=True, dpi=50)
    assert os.path.basename(carpeta) == "reporte_20261201_100000"
    for f in R.ARCHIVOS:
        assert os.path.isfile(os.path.join(carpeta, f)), f
    modo = S.MODES["B"]                          # liberación a 100 m, autogiro a 80 m
    assert r["h_max"]["altura_m"] == pytest.approx(modo["h_top"], abs=4)
    assert modo["deploy_alt"] - 10 <= r["despliegue"]["altura_m"] <= modo["deploy_alt"] + 15
    assert r["liberacion"]["altura_m"] > r["despliegue"]["altura_m"]
    assert 8 <= r["descenso"]["mediana_ms"] <= 12 and r["descenso"]["en_banda"]
    assert r["post_aterrizaje"]["cumple"] and r["enlace"]["perdidos"] > 0
    html = open(os.path.join(carpeta, "reporte.html"), encoding="utf-8").read()
    assert "data:image/png;base64," in html and 'src="http' not in html   # autocontenido
    assert "Project Hellmary".upper() in html and "Log interno" in html
    md = open(os.path.join(carpeta, "reporte.md"), encoding="utf-8").read()
    assert "no recibió" in md and "| Concepto | Estación terrena | Log interno |" in md


def test_reporte_modo_b_liberacion_80m(tmp_path):
    """Soltado a 80 m (debajo de la liberación mínima del modo B): la placa no detecta la liberación; el
    reporte lo dice y aun así encuentra el aterrizaje y la altura máxima."""
    pytest.importorskip("matplotlib")
    kw = dict(S.MODES["B"], h_top=80.0)
    csv_path, _ = vuelo_simulado(str(tmp_path), "20261201_110000", perdida=0, log=False, **kw)
    carpeta, r = R.generar_reporte(csv_path, dpi=50)
    assert r["h_max"]["altura_m"] == pytest.approx(80, abs=4)
    assert r["liberacion"] is None and r["despliegue"] is None and "descenso" not in r
    assert r["aterrizaje"] is not None and r["vz_max_caida"]["vz_ms"] > 20
    md = open(os.path.join(carpeta, "reporte.md"), encoding="utf-8").read()
    assert "no detectada" in md


def test_cli_sesion_mas_reciente(tmp_path, monkeypatch):
    pytest.importorskip("matplotlib")
    vacia = tmp_path / "sesion_20261201_130000.csv"
    vacia.write_text(",".join(CAB) + "\n")
    csv_path, _ = vuelo_simulado(str(tmp_path), "20261201_120000", log=False,
                                 **S.MODES["B"])
    os.utime(vacia, (2e9, 2e9))                  # la vacía es la más nueva: se salta
    assert R.sesion_reciente(str(tmp_path)) == csv_path
    monkeypatch.setattr(R, "DATA_DIR", str(tmp_path))
    monkeypatch.setattr(R, "sesion_reciente", lambda: csv_path)
    assert R.main(["--dpi", "40", "--json"]) == 0
    assert os.path.isdir(tmp_path / "reporte_20261201_120000")
    assert R.main([str(vacia)]) == 2
