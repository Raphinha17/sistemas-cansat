import csv

import resumen_enlace as R


def _csv(path, filas):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["secuencia", "rssi_dbm", "snr_db"])
        w.writerows(filas)


def test_resumen_con_perdidas_y_vuelta_de_secuencia(tmp_path):
    p = tmp_path / "a.csv"
    # 65534, 65535, 0, (faltan 1 y 2), 3 -> 5 recibidos de 6 esperados... y un salto de 2
    _csv(p, [(65534, -80, 8), (65535, -82, 7), (0, -84, 6), (3, -86, 5)])
    r = R.resumen(str(p))
    assert (r["recibidos"], r["esperados"]) == (4, 6)
    assert abs(r["perdida_pct"] - 100 * 2 / 6) < 1e-9
    assert r["rssi_prom"] == -83 and r["rssi_min"] == -86 and r["snr_min"] == 5


def test_resumen_vacio_y_reinicio(tmp_path):
    p = tmp_path / "b.csv"
    _csv(p, [])
    assert R.resumen(str(p)) is None
    _csv(p, [(10, -70, 9), (11, -70, 9), (0, -70, 9), (1, -70, 9)])   # la placa reinició
    r = R.resumen(str(p))
    assert (r["recibidos"], r["esperados"], r["perdida_pct"]) == (4, 4, 0.0)
    assert R.main([str(p)]) == 0
