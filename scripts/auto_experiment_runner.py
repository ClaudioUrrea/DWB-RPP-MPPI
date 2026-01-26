#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lanza Nav2 con cada controlador (cerrando proceso previo),
ejecuta go_to_goal.py + return_to_start.py para N trials,
y organiza resultados en subcarpetas por controlador.

Ejemplos:
  # 5 en total, empezando desde el 3 => ejecuta 3,4,5
  python3 auto_experiment_runner.py --trials 5 --start-trial 3 --goal 3.5 2.0 0.0

  # subset controladores:
  python3 auto_experiment_runner.py --controllers mppi rpp --trials 2
"""

import os, sys, time, signal, argparse, subprocess
from pathlib import Path

NAMESPACE = "a200_0000"
USE_SIM_TIME = "true"
LAUNCH_PATH = "/home/kevin/ros2_ws/src/kevin_nav2_experiments/launch/nav2_local_only.launch.py"

CONTROLLERS = {
    #"mppi": "/home/kevin/ros2_ws/src/kevin_nav2_experiments/configs/fair/nav2_mppi_fair.yaml",
    #"rpp": "/home/kevin/ros2_ws/src/kevin_nav2_experiments/configs/fair/nav2_rpp_fair.yaml",
    "dwb": "/home/kevin/ros2_ws/src/kevin_nav2_experiments/configs/fair/nav2_dwb_fair.yaml",
}

TRIALS = 3
NAV2_WARMUP_SEC = 30

SCRIPTS_DIR = "/home/kevin/ros2_ws/src/kevin_nav2_experiments/scripts"
GO_SCRIPT = os.path.join(SCRIPTS_DIR, "go_to_goal.py")
RETURN_SCRIPT = os.path.join(SCRIPTS_DIR, "return_to_start.py")

RESULTS_DIR = "/home/kevin/ros2_ws/src/kevin_nav2_experiments/results"
DEFAULT_GOAL = [3.5, 2.0, 0.0]

def launch_nav2(params_file: str):
    cmd = [
        "ros2", "launch", LAUNCH_PATH,
        f"namespace:={NAMESPACE}",
        f"use_sim_time:={USE_SIM_TIME}",
        f"params_file:={params_file}",
    ]
    print(f"[INFO] Lanzando Nav2: {' '.join(cmd)}")
    # Si prefieres evitar bloqueos por logs, cambia a: return subprocess.Popen(cmd)
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

def terminate_process(p: subprocess.Popen, name="Nav2"):
    if p is None: return
    if p.poll() is None:
        print(f"[INFO] Enviando SIGINT a {name}...")
        try: p.send_signal(signal.SIGINT)
        except Exception: pass
        for _ in range(20):
            if p.poll() is not None: break
            time.sleep(0.5)
    if p.poll() is None:
        print(f"[WARN] {name} no respondió. SIGKILL.")
        try: p.kill()
        except Exception: pass

def run_python(script_path, args_list):
    cmd = [sys.executable, script_path] + args_list
    print(f"[INFO] Ejecutando: {' '.join(cmd)}")
    p = subprocess.Popen(cmd)
    return p.wait()

def main():
    parser = argparse.ArgumentParser(description="Runner de experimentos (Nav2 por controlador).")
    parser.add_argument("--trials", type=int, default=TRIALS,
                        help="Número total de trials (último índice). Ej: 5.")
    parser.add_argument("--start-trial", type=int, default=1,
                        help="Trial inicial (inclusive). Ej: 3 ejecuta 3..N.")
    parser.add_argument("--results-dir", type=str, default=RESULTS_DIR)
    parser.add_argument("--goal", type=float, nargs=3, default=DEFAULT_GOAL)
    parser.add_argument("--controllers", type=str, nargs="+", default=list(CONTROLLERS.keys()))
    parser.add_argument("--publish-initialpose", action="store_true",
                        help="(opcional) Forzar publicación de initialpose en go_to_goal SOLO si lo deseas.")
    args = parser.parse_args()

    # Validaciones básicas
    if args.start_trial < 1:
        print("[ERROR] --start-trial debe ser >= 1"); sys.exit(1)
    if args.trials < args.start_trial:
        print("[ERROR] --trials debe ser >= --start-trial"); sys.exit(1)

    Path(args.results_dir).mkdir(parents=True, exist_ok=True)

    for controller in args.controllers:
        assert controller in CONTROLLERS, f"Controlador desconocido: {controller}"
        params_file = CONTROLLERS[controller]

        print(f"\n========== CONTROLADOR: {controller.upper()} ==========")
        nav2_proc = launch_nav2(params_file)
        print(f"[INFO] Warmup {NAV2_WARMUP_SEC}s para Nav2...")
        time.sleep(NAV2_WARMUP_SEC)

        # Ejecuta desde start_trial hasta trials (inclusive)
        for trial in range(args.start_trial, args.trials + 1):
            print(f"\n--- Trial {trial}/{args.trials} - {controller} ---")

            # GO
            go_args = [
                "--controller", controller,
                "--trial", str(trial),
                "--results-dir", args.results_dir,
                "--goal", str(args.goal[0]), str(args.goal[1]), str(args.goal[2]),
            ]
            if args.publish_initialpose:
                go_args.append("--publish-initialpose")
            ret = run_python(GO_SCRIPT, go_args)
            if ret != 0:
                print(f"[ERROR] go_to_goal.py terminó con código {ret}. Abortando controlador {controller}.")
                break

            # RETURN
            return_args = [
                "--controller", controller,
                "--trial", str(trial),
                "--results-dir", args.results_dir,
            ]
            ret = run_python(RETURN_SCRIPT, return_args)
            if ret != 0:
                print(f"[ERROR] return_to_start.py terminó con código {ret}. Abortando controlador {controller}.")
                break

        terminate_process(nav2_proc, name=f"Nav2 ({controller})")
        print(f"[INFO] Controlador {controller} finalizado.")

    print("\n[OK] Experimentos completos.")

if __name__ == "__main__":
    main()

