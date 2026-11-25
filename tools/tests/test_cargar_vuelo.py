import os
import subprocess

import cargar_vuelo as C

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SH = os.path.join(ROOT, "tools", "cargar_vuelo.sh")
ARR_OK = "IMU OK (WHO_AM_I=0x70)  BARO OK\nLittleFS OK\n"
STAT_OK = ("despliegue=B [lanzamiento real] liberacion valida > 85 m\nestado=1\n"
           "ciclos=10 sobretiempos=0 max_us=900 imu_err=0 baro_err=0\n")


def ok(rev):
    return {n: o for n, o, _ in rev}


def test_todo_bien():
    r = C.analizar(ARR_OK, STAT_OK)
    assert all(ok(r).values())
    txt, listo = C.resumen(r, color=False)
    assert listo and txt.endswith("Placa lista para vuelo en MODO B")


def test_cada_falla_se_detecta():
    assert not ok(C.analizar(ARR_OK, STAT_OK.replace("despliegue=B", "despliegue=A")))[
        "despliegue=B confirmado"]
    assert not ok(C.analizar(ARR_OK, STAT_OK.replace("imu_err=0", "imu_err=3")))["imu_err en 0"]
    assert not ok(C.analizar(ARR_OK, STAT_OK.replace("baro_err=0", "baro_err=1")))["baro_err en 0"]
    for arr in ("IMU FALLA (WHO_AM_I=0x00)  BARO OK", "IMU OK (WHO_AM_I=0x70)  BARO FALLA"):
        assert not ok(C.analizar(arr, STAT_OK))["arranque sin IMU FALLA ni BARO FALLA"]
    r = C.analizar("", "")                      # sin respuesta: todo falla
    assert not any(ok(r).values())
    txt, listo = C.resumen(r)
    assert not listo and "NO lista" in txt and "\033[31m" in txt


def _sh(patron, *args):
    env = dict(os.environ, CARGAR_VUELO_PATRON=patron)
    return subprocess.run([SH, *args], capture_output=True, text=True, env=env)


def test_sh_puerto_ninguno_o_varios(tmp_path):
    r = _sh(str(tmp_path / "ttyUSB*"))
    assert r.returncode != 0 and "No hay ningún puerto" in r.stderr
    for n in (0, 1):
        (tmp_path / f"ttyUSB{n}").touch()
    r = _sh(str(tmp_path / "ttyUSB*"))
    assert r.returncode != 0 and "varios puertos" in r.stderr and "Compilando" not in r.stdout
    r = _sh("x", str(tmp_path / "noexiste"))
    assert r.returncode != 0 and "No existe el puerto" in r.stderr
