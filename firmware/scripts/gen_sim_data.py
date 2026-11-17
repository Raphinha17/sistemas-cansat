# Script previo de PlatformIO (entorno native): genera con tools/flight_sim.py los datos
# crudos de sensores a 100 Hz que usan las pruebas native (attitude, estimator, flight), y pasa la
# ruta al código C++ como SIM_DATA_DIR.
import os
import subprocess

Import("env")  # noqa: F821  (lo inyecta PlatformIO/SCons)

project_dir = env.subst("$PROJECT_DIR")  # noqa: F821
sim = os.path.normpath(os.path.join(project_dir, "..", "tools", "flight_sim.py"))
out_dir = os.path.join(env.subst("$PROJECT_WORKSPACE_DIR"), "simdata")  # noqa: F821
os.makedirs(out_dir, exist_ok=True)

# (nombre, argumentos extra del simulador). Los escenarios de la máquina de estados
# usan las opciones de flight_sim.py descritas en su encabezado.
CASES = [
    ("flight_seed1", ["--seed", "1"]),
    ("flight_seed7", ["--seed", "7"]),
    ("fail_baro", ["--seed", "1", "--fail", "baro", "--fail-at", "40"]),
    ("fail_imu", ["--seed", "1", "--fail", "imu", "--fail-at", "40"]),
    ("false_release", ["--seed", "1", "--false-release", "70"]),
    ("low_release", ["--seed", "1", "--h-top", "200"]),
    ("gate_delay", ["--seed", "1", "--fall-tilt", "60", "--fall-tilt-until-h", "214"]),
    ("tilt_backup", ["--seed", "1", "--fall-tilt", "70"]),
    ("fast_fall", ["--seed", "1", "--fall-tilt", "70", "--v-terminal", "60"]),
]

for name, extra in CASES:
    out = os.path.join(out_dir, name + ".csv")
    args_file = out + ".args"
    args_txt = " ".join(extra)
    old_args = open(args_file).read() if os.path.exists(args_file) else None
    # Se regenera si falta, si el simulador cambió después de generarlo o si cambiaron
    # los argumentos del caso.
    if (not os.path.exists(out) or os.path.getmtime(out) < os.path.getmtime(sim)
            or old_args != args_txt):
        print("gen_sim_data: generando", out)
        subprocess.check_call([env.subst("$PYTHONEXE"), sim, "--raw-csv", out] + extra)  # noqa: F821
        with open(args_file, "w") as f:
            f.write(args_txt)

env.Append(CPPDEFINES=[("SIM_DATA_DIR", '\\"%s/\\"' % out_dir.replace("\\", "/"))])  # noqa: F821
