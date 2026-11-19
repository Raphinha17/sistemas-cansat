import pytest

import flight_sim as S
import telemetry as T


def run(**kw):
    return list(S.simulate(**kw))


def test_all_states_in_order():
    pk = [p for _, p in run(seed=1)]
    states = [p.estado for p in pk]
    assert set(states) == set(range(7))
    assert states == sorted(states)  # sin retrocesos


def test_sizes_and_roundtrip():
    for _, p in run(seed=2)[:200]:
        raw = T.encode(p)
        assert len(raw) == 32
        assert T.decode(raw).estado == p.estado


def test_profile():
    pk = [p for _, p in run(seed=1)]
    assert max(p.altura_m for p in pk) == pytest.approx(450, abs=5)
    assert min(p.vz_ms for p in pk) < -30
    desc = [p.vz_ms for p in pk if p.estado == T.DESCENSO][-20:]
    assert 8 <= sum(desc) / len(desc) * -1 <= 12
    fire = next(p for p in pk if p.estado == T.DESPLIEGUE)
    assert 190 <= fire.altura_m <= 260
    assert pk[-1].banderas & T.F_ATERRIZADO


def test_fixed_rate_and_determinism():
    a = run(seed=5, rate=10)
    assert len(a) > 1000
    assert [p for _, p in a] == [p for _, p in run(seed=5, rate=10)]


@pytest.mark.parametrize("fail,flag", [("baro", T.F_FALLO_BARO), ("imu", T.F_FALLO_IMU)])
def test_failures_still_complete(fail, flag):
    pk = [p for _, p in run(seed=1, fail=fail, fail_at=30)]
    assert pk[-1].banderas & flag
    assert set(p.estado for p in pk) == set(range(7))


def test_cli_loss(tmp_path):
    out = tmp_path / "f.bin"
    assert S.main(["--out", str(out), "--loss", "30", "--seed", "1"]) == 0
    full = tmp_path / "g.bin"
    S.main(["--out", str(full), "--seed", "1"])
    n_loss, n_full = out.stat().st_size // 32, full.stat().st_size // 32
    assert out.stat().st_size % 32 == 0
    assert 0.55 * n_full < n_loss < 0.85 * n_full


def test_raw_csv(tmp_path):
    import csv
    out = tmp_path / "raw.csv"
    assert S.main(["--raw-csv", str(out), "--seed", "1"]) == 0
    rows = list(csv.DictReader(open(out)))
    assert list(rows[0].keys()) == S.RAW_COLUMNS
    dt = float(rows[1]["t"]) - float(rows[0]["t"])
    assert abs(dt - 0.01) < 1e-6                       # 100 Hz
    assert {r["fase"] for r in rows} >= {"ground", "ascent", "hover", "fall", "impact"}
    # En tierra el giroscopio solo mide sesgo + ruido (constante en promedio)
    g = [float(r["gx_dps"]) for r in rows[:500]]
    assert max(g) - min(g) < 1.5
    # El barómetro se actualiza a 25 Hz
    assert abs(sum(int(r["baro_new"]) for r in rows[:1000]) - 250) <= 2


def _raw(**kw):
    rows = []
    list(S.simulate(raw_cb=rows.append, **kw))
    return rows  # tuplas en el orden de S.RAW_COLUMNS


def test_escenario_h_top():
    rows = _raw(seed=1, h_top=200)
    assert max(r[10] for r in rows) == pytest.approx(200, abs=0.5)


def test_escenario_falsa_liberacion():
    rows = _raw(seed=1, false_release_at=70)
    win = [r for r in rows if 70.05 <= r[0] < 70.2]
    assert win and all(abs(r[3]) < 0.3 for r in win)      # baja g durante la caída breve
    vz_min = min(r[11] for r in rows if 69 < r[0] < 72)
    assert 3.0 < vz_min < 4.0                               # 6 m/s - 0.25 s * g: no cae
    assert max(r[10] for r in rows) == pytest.approx(450, abs=0.5)  # el vuelo sigue


def test_escenario_fall_tilt():
    rows = _raw(seed=1, fall_tilt=70)
    t_fall = next(r[0] for r in rows if r[15] == "fall")
    roll = [r[12] for r in rows if r[15] == "fall" and t_fall + 5 < r[0] < t_fall + 7]
    assert all(55 < x < 85 for x in roll)
    # hasta una altura: después se endereza
    rows = _raw(seed=1, fall_tilt=60, fall_tilt_until_h=214)
    late = [abs(r[12]) for r in rows if r[15] == "fall" and 150 < r[10] < 180]
    assert sum(late) / len(late) < 40                       # vuelve a la oscilación normal


def test_escenario_v_terminal():
    rows = _raw(seed=1, v_terminal=60)
    assert min(r[11] for r in rows) < -45


def _states(pk):
    out = []
    for p in pk:
        if not out or out[-1] != p.estado:
            out.append(p.estado)
    return out


def test_modo_b():
    rows = []
    pk = [p for _, p in S.simulate(seed=1, raw_cb=rows.append, **S.MODES["B"])]
    assert max(r[10] for r in rows) == pytest.approx(100, abs=0.5)
    assert _states(pk) == list(range(7))
    fire = next(p for p in pk if p.estado == T.DESPLIEGUE)
    assert 50 <= fire.altura_m <= 70           # h_disp = 50 + |vz| * 0.5
    assert not pk[-1].banderas & T.F_DISPARO_RESPALDO


def test_modo_c_tinaco():
    rows = []
    pk = [p for _, p in S.simulate(seed=1, raw_cb=rows.append, **S.MODES["C"])]
    assert max(r[10] for r in rows) == pytest.approx(8, abs=0.5)
    st = _states(pk)
    assert T.ASCENSO not in st and st[-1] == T.ATERRIZADO
    t_fall = next(r[0] for r in rows if r[15] == "fall")
    t_fire = next(t for t, p in S.simulate(seed=1, **S.MODES["C"]) if p.banderas & T.F_DISPARO)
    assert t_fire - t_fall < 0.5               # 150 ms de baja g + la siguiente transmisión


def test_cli_release_y_deploy_alt(tmp_path):
    import csv
    out = tmp_path / "b.csv"
    assert S.main(["--raw-csv", str(out), "--seed", "1", "--mode", "B", "--release-alt", "120",
                   "--deploy-alt", "60"]) == 0
    rows = list(csv.DictReader(open(out)))
    assert max(float(r["h_true"]) for r in rows) == pytest.approx(120, abs=0.5)
    # --h-top sigue funcionando (alias de --release-alt)
    out2 = tmp_path / "a.csv"
    assert S.main(["--raw-csv", str(out2), "--seed", "1", "--h-top", "300"]) == 0
    rows = list(csv.DictReader(open(out2)))
    assert max(float(r["h_true"]) for r in rows) == pytest.approx(300, abs=0.5)
