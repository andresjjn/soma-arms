"""scripts/jetson_driver.sh recreates the driver container: tested on a fake docker.

The script is the only recipe for gate 1 on the Jetson (no docker compose
there), and removing an ARMED driver would kill it before it cuts the
signal. A stub `docker` on PATH records every call and plays back the
driver's log, so the refusal and the exact run flags are pinned here.
"""
import os
import shutil
import stat
import subprocess
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / 'scripts' / 'jetson_driver.sh'

FAKE_DOCKER = """#!/usr/bin/env bash
echo "$*" >> "$CALLS"
case "$1" in
  inspect) [ -n "$FAKE_EXISTS" ] ;;
  logs) printf '%b' "$FAKE_LOG" ;;
  *) exit 0 ;;
esac
"""


def run(tmp_path, arg, exists=True, log=''):
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir(exist_ok=True)
    docker = bin_dir / 'docker'
    docker.write_text(FAKE_DOCKER)
    docker.chmod(docker.stat().st_mode | stat.S_IEXEC)
    calls = tmp_path / 'calls.txt'
    env = dict(os.environ, PATH=f'{bin_dir}:{os.environ["PATH"]}', CALLS=str(calls),
               FAKE_EXISTS='1' if exists else '', FAKE_LOG=log)
    done = subprocess.run(['bash', str(SCRIPT)] + ([arg] if arg else []),
                          env=env, capture_output=True, text=True)
    lines = calls.read_text().splitlines() if calls.exists() else []
    return done, lines


pytestmark = pytest.mark.skipif(shutil.which('bash') is None, reason='needs bash')


def test_refuses_to_touch_an_armed_driver(tmp_path):
    log = r'SOMA driver up: MOCK backend, DISARMED\n[WARN] SOMA: ARMED: real PCA9685 output is live\n'
    done, calls = run(tmp_path, 'closed', log=log)
    assert done.returncode == 3
    assert 'REFUSED' in done.stderr
    assert not any(c.startswith(('stop', 'rm', 'run')) for c in calls)


def test_a_disarmed_driver_is_replaced_with_gate_1_closed(tmp_path):
    log = r'[WARN] SOMA: ARMED: real PCA9685 output is live\n[WARN] SOMA: DISARMED: signal cut, back on MOCK\n'
    done, calls = run(tmp_path, 'closed', log=log)
    assert done.returncode == 0, done.stderr
    verbs = [c.split()[0] for c in calls]
    assert verbs.index('stop') < verbs.index('rm') < verbs.index('run')
    run_call = next(c for c in calls if c.startswith('run'))
    assert 'allow_real:=false' in run_call
    for flag in ('--network host', '--ipc host', '--restart unless-stopped',
                 '--device /dev/i2c-7:/dev/i2c-7', '-v '):
        assert flag in run_call, flag


def test_open_launches_with_allow_real_true_and_nothing_else_changes(tmp_path):
    done, calls = run(tmp_path, 'open', exists=False)
    assert done.returncode == 0, done.stderr
    assert [c.split()[0] for c in calls] == ['inspect', 'run']
    assert 'allow_real:=true' in calls[-1]
    assert 'MOCK and DISARMED' in done.stdout


@pytest.mark.parametrize('arg', ['', 'true', 'arm'])
def test_anything_but_open_or_closed_is_refused(tmp_path, arg):
    done, calls = run(tmp_path, arg)
    assert done.returncode == 2
    assert calls == []


def test_the_script_and_the_compose_file_describe_the_same_container():
    # The Jetson has no docker compose, so the script is what actually runs;
    # the compose file stays as its readable twin and must not drift.
    yaml = pytest.importorskip('yaml')
    compose = yaml.safe_load((SCRIPT.parents[1] / 'docker-compose.jetson.yml').read_text())
    svc = compose['services']['driver']
    script = SCRIPT.read_text()
    assert svc['container_name'] == 'soma_driver' and '--name soma_driver' in script
    assert svc['network_mode'] == 'host' and '--network host' in script
    assert svc['ipc'] == 'host' and '--ipc host' in script
    assert svc['restart'] == 'unless-stopped' and '--restart unless-stopped' in script
    assert svc['mem_limit'] == '1g' and '--memory 1g' in script
    for device in svc['devices']:
        assert f'--device {device}' in script
    compose_cmd = ' '.join(svc['command'].split()).replace('${ALLOW_REAL:-false}', '$allow')
    assert compose_cmd.strip('"').replace('bash -c "', '') in ' '.join(script.split())
