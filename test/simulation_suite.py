#!/usr/bin/env python3
"""Run current host checks and sanitised module/navigation simulations.

This does not emulate the STM32 CPU, physical actuators, or camera images.
Returns nonzero if any current check fails, including existing failures.
"""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
import argparse
import json
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]


def run(command, name):
    started = time.monotonic()
    try:
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=180)
        return {"name": name, "exit_code": result.returncode,
                "seconds": round(time.monotonic() - started, 3),
                "stdout": result.stdout, "stderr": result.stderr}
    except subprocess.TimeoutExpired as error:
        return {"name": name, "exit_code": 124,
                "seconds": round(time.monotonic() - started, 3),
                "stdout": str(error.stdout or ""), "stderr": "TIMEOUT " + str(error.stderr or "")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "docs/simulation_20261119")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    jobs = []
    for path in sorted((ROOT / "test").glob("*_test.py")):
        command = ["python3", str(path)]
        if path.name == "grid_scenarios_test.py":
            command += ["--output", str(output)]
        jobs.append((command, path.name))
    results = []
    with ThreadPoolExecutor(max_workers=2) as executor:
        pending = [executor.submit(run, command, name) for command, name in jobs]
        for future in as_completed(pending):
            result = future.result()
            results.append(result)
            print("PASS" if result["exit_code"] == 0 else "FAIL", result["name"], flush=True)
    checks = {"total": len(results), "passed": sum(r["exit_code"] == 0 for r in results),
              "results": sorted(results, key=lambda r: r["name"])}
    (output / "existing_tests.json").write_text(json.dumps(checks, ensure_ascii=False, indent=2))

    specs = [
        ("competition_core", "native_competition_core.cpp", ["CompetitionCore"]),
        ("vision_protocol", "native_vision_protocol.cpp", ["VisionProtocol"]),
        ("mecanum_odometry", "native_mecanum_odometry.cpp", ["MecanumOdometry", "MecanumKinematics"]),
        ("protocol_stress", "native_grid_protocol_stress.cpp", ["QimingGrid"]),
        ("grid_reachability", "native_grid_reachability.cpp", ["QimingGrid"]),
    ]
    modules = []
    with tempfile.TemporaryDirectory(prefix="gc_sim_suite_") as directory:
        for name, source, libraries in specs:
            executable = str(Path(directory) / name)
            command = ["clang++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror",
                       "-fsanitize=address,undefined", "-g"]
            for library in libraries:
                command += ["-I", str(ROOT / "lib" / library / "src")]
            command += [str(ROOT / "test" / source)]
            for library in libraries:
                command += [str(p) for p in sorted((ROOT / "lib" / library / "src").glob("*.cpp"))]
            command += ["-o", executable]
            compiled = run(command, name + ":compile")
            tested = run([executable], name) if compiled["exit_code"] == 0 else None
            modules.append({"name": name, "compile": compiled, "run": tested})
            print("PASS" if tested and tested["exit_code"] == 0 else "FAIL", name, flush=True)
    (output / "native_modules.json").write_text(json.dumps(modules, ensure_ascii=False, indent=2))
    failed_modules = sum(not m["run"] or m["run"]["exit_code"] != 0 for m in modules)
    summary = {"script_total": checks["total"], "script_passed": checks["passed"],
               "script_failed": checks["total"] - checks["passed"],
               "native_total": len(modules), "native_failed": failed_modules,
               "simulation_scope": "host native modules and extracted production functions; event models for hardware"}
    (output / "suite_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    return 1 if summary["script_failed"] or failed_modules else 0


if __name__ == "__main__":
    raise SystemExit(main())
