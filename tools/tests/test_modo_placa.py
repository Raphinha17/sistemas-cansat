import os
import subprocess

import modo_placa as M

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def test_confirma():
    assert M.confirma("despliegue=D [tinaco: solo carga primaria, SIN disparo]", "D")
    assert not M.confirma("despliegue=C [tinaco]", "D")
    assert not M.confirma("ERR el modo solo se cambia en los estados 0 y 1", "B")
    assert not M.confirma("", "B")


def test_elegir_puerto():
    assert M.elegir_puerto("/dev/x", prueba=lambda p: False) == ("/dev/x", "")
    assert M.elegir_puerto(None, [])[0] is None
    p, err = M.elegir_puerto(None, ["/dev/ttyUSB0", "/dev/ttyUSB1"], lambda p: p.endswith("1"))
    assert p == "/dev/ttyUSB1" and not err
    p, err = M.elegir_puerto(None, ["/dev/ttyUSB0"], lambda p: False)
    assert p is None and "respondió" in err
    p, err = M.elegir_puerto(None, ["/dev/ttyUSB0", "/dev/ttyUSB1"], lambda p: True)
    assert p is None and "Varias" in err


def test_scripts_sin_placa_salen_con_error():
    for nombre, modo in (("tinaco_separacion", "C"), ("tinaco_primaria", "D"), ("modo_dron", "B")):
        sh = os.path.join(ROOT, "tools", nombre + ".sh")
        assert os.access(sh, os.X_OK)
        r = subprocess.run([sh, "/dev/ttyNOPE"], capture_output=True, text=True)
        assert r.returncode == 2 and "No se pudo abrir" in r.stderr
    assert "TINACO SOLO CARGA PRIMARIA: el servo NO disparará" in M.MENSAJES["D"][1]
    assert "TINACO CON SEPARACIÓN: el servo SÍ disparará" in M.MENSAJES["C"][1]
