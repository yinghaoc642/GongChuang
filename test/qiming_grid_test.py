#!/usr/bin/env python3
"""Compile the actual pure C++ protocol and mission; simulate device frames."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
src = root / 'lib/QimingGrid/src'
with tempfile.TemporaryDirectory(prefix='gc_grid_') as tmp:
    binary = str(Path(tmp) / 'grid_test')
    subprocess.run(['clang++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-g', '-I', str(src),
                    str(root / 'test/native_qiming_grid.cpp'),
                    *(str(src / name) for name in ['LinkProtocol.cpp', 'QimingGridClient.cpp', 'GridNavigator.cpp']),
                    '-o', binary], check=True)
    subprocess.run([binary], check=True)
    bridge = str(Path(tmp) / 'drive_test')
    subprocess.run(['clang++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-g', '-I', str(src),
                    '-I', str(root / 'src'), '-DGONGCHUANG_GRID_NAVIGATION=1',
                    str(root / 'test/native_grid_drive.cpp'),
                    *(str(src / name) for name in ['LinkProtocol.cpp', 'QimingGridClient.cpp', 'GridNavigator.cpp']),
                    '-o', bridge], check=True)
    subprocess.run([bridge], check=True)
