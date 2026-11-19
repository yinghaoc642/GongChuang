#!/usr/bin/env python3
"""Batch the actual navigation FSM with independent radar and geometry models.

Use --output DIR to retain all cases. No files in the firmware are modified.
The detailed result counts safe rejection separately from task completion.
"""
from pathlib import Path
from collections import Counter
import argparse
import json
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
LIB = ROOT / "lib/QimingGrid/src"


def summarize(rows):
    result = {"total": len(rows), "groups": {}}
    for name in dict.fromkeys(r["group"] for r in rows):
        part = [r for r in rows if r["group"] == name]
        result["groups"][name] = {
            "total": len(part),
            "clear_handoff": sum(r["outcome"] == 1 for r in part),
            "six_stations_and_home": sum(r["outcome"] == 2 for r in part),
            "fault_stop": sum(r["outcome"] == 3 for r in part),
            "failure_codes": dict(Counter(str(r["error"]) for r in part if r["outcome"] == 3)),
        }
    result["static_by_obstacle_count"] = []
    for count in range(4):
        for zone in (1, 2):
            part = [r for r in rows if r["group"] == "static_0_to_3"
                    and bin(r["map1"]).count("1") == count and r["zone"] == zone]
            result["static_by_obstacle_count"].append({
                "obstacle_cells": count, "zone": zone, "total": len(part),
                "clear_handoff": sum(r["outcome"] == 1 for r in part),
                "completed": sum(r["outcome"] == 2 for r in part),
                "fault_stop": sum(r["outcome"] == 3 for r in part),
            })
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="gc_scenarios_") as directory:
        binary = Path(directory) / "simulation"
        command = ["clang++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror",
                   "-fsanitize=address,undefined", "-g", "-I", str(LIB),
                   str(ROOT / "test/native_grid_simulation.cpp"),
                   *(str(LIB / name) for name in ["GridNavigator.cpp", "QimingGridClient.cpp", "LinkProtocol.cpp"]),
                   "-o", str(binary)]
        subprocess.run(command, check=True)
        run = subprocess.run([str(binary)], text=True, capture_output=True)
        if args.output:
            args.output.mkdir(parents=True, exist_ok=True)
            (args.output / "grid_scenarios.jsonl").write_text(run.stdout)
            (args.output / "grid_run.json").write_text(json.dumps({
                "exit_code": run.returncode, "stderr": run.stderr,
                "compile_command": command,
            }, ensure_ascii=False, indent=2))
        if run.returncode:
            raise RuntimeError(f"simulation exit={run.returncode}\n{run.stderr}")
        rows = [json.loads(line) for line in run.stdout.splitlines()]
        report = summarize(rows)
        if args.output:
            (args.output / "grid_summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(f"PASS {len(rows)} navigation scenarios: independent radar BFS, "
              "swept footprint, six station order, departures, home, failure latching; ASan/UBSan")
        for name, counts in report["groups"].items():
            print(name, json.dumps(counts))


if __name__ == "__main__":
    main()
