import asyncio
import csv
import json
import os

from fastapi.testclient import TestClient

import gs_core as C
import gs_sources as SRC
import server as SV
import telemetry as T


class FakeSource(SRC.Source):
    """Fuente quieta: las pruebas meten las líneas con station.handle_line."""
    name = "FAKE"

    async def run(self, on_line):
        await asyncio.Event().wait()


def _line(i, h=0.0, estado=1, flags=0):
    p = T.Packet(estado=estado, secuencia=i, t_ms=i * 1000, altura_m=h, vz_ms=-9.0,
                 banderas=flags, presion_pa=99000, temp_c=20.0, bateria_v=0.0)
    return C.make_line(p, -55.0, 9.0)


def _client(tmp_path):
    st = SV.Station(FakeSource(), str(tmp_path / "data"))
    return st, TestClient(SV.create_app(st))


def feed(client, st, lines):
    for l in lines:
        client.portal.call(st.handle_line, l)


def test_rest_estado_y_paginas(tmp_path):
    st, app = _client(tmp_path)
    with app as c:
        r = c.get("/api/status").json()
        assert r["s"]["link"] == "esperando" and r["rec"]["active"] is False
        assert c.get("/").status_code == 200 and "PROJECT HELLMARY" in c.get("/").text
        assert c.get("/static/vendor/uPlot.iife.min.js").status_code == 200   # sin CDN
        assert c.get("/static/app.js").status_code == 200


def test_logo_marcador_de_posicion(tmp_path, monkeypatch):
    st, app = _client(tmp_path)
    real_isfile = SV.os.path.isfile
    with app as c:
        # Con el archivo (ground/static/logo.png está en el repositorio): se sirve el PNG
        if real_isfile(SV.os.path.join(SV.STATIC_DIR, "logo.png")):
            r = c.get("/logo")
            assert r.status_code == 200 and r.headers["content-type"] == "image/png"
        # Sin el archivo: marcador de posición que dice dónde ponerlo
        monkeypatch.setattr(SV.os.path, "isfile",
                            lambda p: False if p.endswith("logo.png") else real_isfile(p))
        r = c.get("/logo")
        assert r.status_code == 200 and "svg" in r.headers["content-type"]
        assert "ground/static/logo.png" in r.text


def test_lineas_invalidas_no_tumban_el_servidor(tmp_path):
    st, app = _client(tmp_path)
    with app as c:
        feed(c, st, [_line(0), "basura", "", "# info", "ERR,-7,-90,-3", _line(1), "1,2,3"])
        s = c.get("/api/status").json()["s"]
        assert (s["received"], s["invalid"], s["radio_errors"]) == (2, 2, 1)


def test_grabacion_y_exportacion(tmp_path):
    st, app = _client(tmp_path)
    with app as c:
        assert c.get("/api/export.csv").status_code == 404            # nada grabado aún
        feed(c, st, [_line(0)])                                        # antes de grabar: no se guarda
        info = c.post("/api/record/start").json()
        assert info["active"] and info["csv"].endswith(".csv")
        feed(c, st, [_line(1, 10.5), "basura", _line(2, 20.5)])
        assert c.get("/api/status").json()["rec"]["packets"] == 2
        info = c.post("/api/record/stop").json()
        assert not info["active"] and info["packets"] == 2
        r = c.get("/api/export.csv")
        assert r.status_code == 200 and "attachment" in r.headers["content-disposition"]
        rows = list(csv.DictReader(r.text.splitlines()))
        assert [float(x["altura_m"]) for x in rows] == [10.5, 20.5]
        raw = (tmp_path / "data" / info["raw"]).read_text().splitlines()
        assert len(raw) == 3 and raw[1] == "basura"


def test_websocket_hello_paquetes_y_eventos(tmp_path):
    st, app = _client(tmp_path)
    with app as c:
        feed(c, st, [_line(0, estado=1), _line(1, estado=2, flags=0x01)])
        with c.websocket_connect("/ws") as ws:
            hello = json.loads(ws.receive_text())
            assert hello["type"] == "hello" and len(hello["history"]) == 2
            assert hello["states"][3] == "CAIDA" and len(hello["flags"]) == 8
            assert any(e["text"] == "Estado: ASCENSO" for e in hello["events"])
            feed(c, st, [_line(2, h=33.3, estado=3, flags=0x03)])
            tipos = []
            while True:
                m = json.loads(ws.receive_text())
                tipos.append(m["type"])
                if m["type"] == "packet":
                    assert m["p"]["h"] == 33.3 and m["s"]["received"] == 3
                if "event" in tipos and "packet" in tipos and any(
                        t == "event" for t in tipos[tipos.index("packet"):]):
                    break
            assert "packet" in tipos


def test_reset(tmp_path):
    st, app = _client(tmp_path)
    with app as c:
        feed(c, st, [_line(0), _line(1)])
        assert c.post("/api/reset").json()["ok"]
        s = c.get("/api/status").json()["s"]
        assert s["received"] == 0 and s["link"] == "esperando"


def test_replay_a_traves_del_servidor(tmp_path):
    """Graba una sesión simulada y la repite con --replay a gran velocidad."""
    rec = C.Recorder(str(tmp_path))
    rec.start()
    n = 0
    for t, line in SRC.sim_lines(seed=1):
        kind, rx = C.parse_line(line)
        rec.write_packet(rx, 1_700_000_000 + t)
        n += 1
        if n >= 120:
            break
    rec.stop()
    st = SV.Station(SRC.ReplaySource(rec.csv_path, speed=200.0), str(tmp_path / "d"))
    with TestClient(SV.create_app(st)) as c:
        for _ in range(100):
            if c.get("/api/status").json()["s"]["received"] >= 120:
                break
            import time
            time.sleep(0.1)
        s = c.get("/api/status").json()["s"]
        assert s["received"] == 120 and s["lost"] == 0 and s["invalid"] == 0


def test_generar_reporte(tmp_path):
    """«Generar reporte»: corre tools/reporte_vuelo.py sobre la sesión actual (sin abrir
    la carpeta en la prueba) y lo anota en el registro de eventos."""
    import pytest
    pytest.importorskip("matplotlib")
    st = SV.Station(FakeSource(), str(tmp_path / "data"), open_reports=False)
    with TestClient(SV.create_app(st)) as c:
        r = c.post("/api/report")
        assert r.status_code == 404 and "toma de datos" in r.json()["error"]
        c.post("/api/record/start")
        estados = [1] * 5 + [2] * 10 + [3] * 3 + [4] * 2 + [5] * 6 + [6] * 12
        alturas = [0] * 5 + [10 * k for k in range(1, 11)] + [90, 75, 60, 50, 45] \
            + [40 - 7 * k for k in range(6)] + [0] * 12
        feed(c, st, [_line(i, h=h, estado=e) for i, (e, h) in enumerate(zip(estados, alturas))])
        r = c.post("/api/report")
        assert r.status_code == 200, r.text
        j = r.json()
        assert j["ok"] and os.path.isdir(j["carpeta"])
        assert "reporte.html" in j["archivos"] and "01_altura.png" in j["archivos"]
        assert os.path.dirname(j["carpeta"]) == str(tmp_path / "data")
        assert any("Reporte generado" in e["text"] for e in st.events)
        assert st.report_busy is False


def test_letrero_pausado_no_tapa_el_boton():
    """El letrero PAUSADO va dentro de la barra de herramientas (no flotando encima del
    botón Pausar) y no intercepta clics."""
    import re
    html = open(os.path.join(SV.STATIC_DIR, "index.html"), encoding="utf-8").read()
    css = open(os.path.join(SV.STATIC_DIR, "style.css"), encoding="utf-8").read()
    js = open(os.path.join(SV.STATIC_DIR, "app.js"), encoding="utf-8").read()
    bar = html[html.index('id="chartbar"'):]
    bar = bar[:bar.index("</div>")]
    assert 'id="pausetag"' in bar and 'id="pausebtn"' in bar
    assert bar.index('id="pausetag"') < bar.index('id="pausebtn"')     # junto al botón, no encima
    regla = re.search(r"#pausetag\s*\{([^}]*)\}", css).group(1)
    assert "pointer-events: none" in regla and "position: absolute" not in regla
    assert '"▶ Reanudar" : "⏸ Pausar"' in js and "hidden = !p" in js


def test_tema_plano_sin_red(tmp_path):
    """El tema 'plano técnico' y sus fuentes se sirven desde el propio servidor (sin CDN) y
    el tema clásico sigue intacto como respaldo (?tema=clasico, tecla T)."""
    import re
    st, app = _client(tmp_path)
    with app as c:
        html = c.get("/").text
        assert "/static/tema-plano.css" in html and "hellmary_tema" in html
        css = c.get("/static/tema-plano.css")
        assert css.status_code == 200
        urls = re.findall(r"url\(\"?([^)\"]+)", css.text)
        assert urls and all(u.startswith("/static/fonts/") for u in urls)   # nada de red
        for u in set(urls):
            assert c.get(u).status_code == 200, u
        assert c.get("/static/fonts/LICENSE-IBM-Plex-OFL.txt").status_code == 200
        assert c.get("/static/style.css").status_code == 200               # el clásico, aparte
        js = c.get("/static/app.js").text
        assert "hellmary_tema" in js and 'e.key === "t"' in js
    for f in (html, css.text, js):
        assert not re.search(r"https?://(?!127\.0\.0\.1)", f.replace("http://www.w3.org", ""))
