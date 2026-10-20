"""Compatibility entry point for the current STORAGE2 batch pipeline checks."""
import runpy
from pathlib import Path
runpy.run_path(str(Path(__file__).with_name("storage_round2_prescan_test.py")), run_name="__main__")
