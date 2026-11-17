# Script previo de PlatformIO (entorno native): genera con tools/flight_sim.py los datos
# crudos de sensores a 100 Hz que usan las pruebas de attitude y estimator, y pasa la
# ruta al código C++ como SIM_DATA_DIR.
import os
import subprocess

Import("env")  # noqa: F821  (lo inyecta PlatformIO/SCons)

project_dir = env.subst("$PROJECT_DIR")  # noqa: F821
sim = os.path.normpath(os.path.join(project_dir, "..", "tools", "flight_sim.py"))
out_dir = os.path.join(env.subst("$PROJECT_WORKSPACE_DIR"), "simdata")  # noqa: F821
os.makedirs(out_dir, exist_ok=True)

# (nombre, argumentos extra del simulador)
CASES = [
    ("flight_seed1", ["--seed", "1"]),
    ("flight_seed7", ["--seed", "7"]),
]

for name, extra in CASES:
    out = os.path.join(out_dir, name + ".csv")
    # Se regenera si falta o si el simulador cambió después de generarlo.
    if not os.path.exists(out) or os.path.getmtime(out) < os.path.getmtime(sim):
        print("gen_sim_data: generando", out)
        subprocess.check_call([env.subst("$PYTHONEXE"), sim, "--raw-csv", out] + extra)  # noqa: F821

env.Append(CPPDEFINES=[("SIM_DATA_DIR", '\\"%s/\\"' % out_dir.replace("\\", "/"))])  # noqa: F821
