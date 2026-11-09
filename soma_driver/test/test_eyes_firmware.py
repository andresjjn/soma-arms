"""firmware/eyes: the eye model and the line protocol, built for the host.

The firmware's pure parts (eye_model.h, protocol.h) compile with any C++17
compiler; firmware/eyes/test/test_eyes.cpp checks them. Skipped where no
compiler is found. The eyes never arm or move anything: a test here also
keeps the firmware free of any word of the driver's.
"""
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
EYES = REPO / 'firmware' / 'eyes'
CXX = shutil.which('c++') or shutil.which('g++') or shutil.which('clang++')


@pytest.mark.skipif(CXX is None, reason='no host C++ compiler')
def test_eye_model_and_protocol(tmp_path):
    exe = tmp_path / 'test_eyes'
    build = subprocess.run([CXX, '-std=c++17', '-O1', '-Wall', '-Wextra', '-o', str(exe),
                            str(EYES / 'test' / 'test_eyes.cpp')], capture_output=True, text=True)
    assert build.returncode == 0, build.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True, timeout=30)
    assert run.returncode == 0, run.stdout + run.stderr


@pytest.mark.skipif(CXX is None, reason='no host C++ compiler')
def test_the_fast_renderer_draws_the_reference_eye(tmp_path):
    exe = tmp_path / 'test_fast'
    build = subprocess.run([CXX, '-std=c++17', '-O2', '-Wall', '-Wextra', '-o', str(exe),
                            str(EYES / 'test' / 'test_fast_render.cpp')], capture_output=True, text=True)
    assert build.returncode == 0, build.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True, timeout=120)
    assert run.returncode == 0, run.stdout + run.stderr


def test_the_eyes_cannot_reach_the_driver():
    source = '\n'.join(p.read_text() for p in (EYES / 'eyes').glob('*.[hic]*') if 'gc9a01' not in p.name)
    for word in ('/soma/arm', 'allow_real', 'soma/command', 'rclpy', 'PCA9685'):
        assert word not in source


def test_the_compiled_default_design_is_style_json():
    # style.json is the single source of truth; style_default.h is generated
    # from it (firmware/eyes/tools/gen_style.py). Edit the JSON, run the script.
    import sys
    run = subprocess.run([sys.executable, str(EYES / 'tools' / 'gen_style.py'), '--check'])
    assert run.returncode == 0, 'style_default.h is stale: run firmware/eyes/tools/gen_style.py'
