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
