"""scripts/hand_show.sh arms only on a typed word, from the hang: tested on fakes.

A stub `docker` on PATH records every call, plays back the driver's log
and answers the home check and the arm call; a stub tracker stands in for
the camera. Pinned here: nothing is armed off home, without "arm" typed,
or when the driver says no; and the arm call comes before the take.
"""
import os
import shutil
import stat
import subprocess
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / 'scripts' / 'hand_show.sh'

FAKE_DOCKER = r"""#!/usr/bin/env bash
echo "$*" >> "$CALLS"
case "$1" in
  inspect) exit 0 ;;
  logs) printf '%b' "$FAKE_LOG" ;;
  exec)
    case "$*" in
      *--home-check*) exit "$FAKE_HOME" ;;
      *"service call /soma/arm"*) printf '%b' "$FAKE_ARM_REPLY" ;;
      *--bridge*) cat > "$BRIDGE_IN" ;;
    esac ;;
esac
"""

FAKE_TRACKER = """#!/usr/bin/env bash
echo "tracker $*" >> "$CALLS"
echo 0.50
"""

DISARMED = r'SOMA driver up: MOCK backend, DISARMED\n'
ARMED = DISARMED + r'[WARN] SOMA: ARMED: real PCA9685 output is live\n'
YES = r"response:\nstd_srvs.srv.SetBool_Response(success=True, message='ARMED: real PCA9685 output is live')\n"
NO = r"response:\nstd_srvs.srv.SetBool_Response(success=False, message='REFUSED: node was started with allow_real:=false')\n"


def run(tmp_path, args, typed='', log=DISARMED, home=0, reply=YES):
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir(exist_ok=True)
    for name, body in (('docker', FAKE_DOCKER), ('tracker', FAKE_TRACKER)):
        f = bin_dir / name
        f.write_text(body)
        f.chmod(f.stat().st_mode | stat.S_IEXEC)
    calls = tmp_path / 'calls.txt'
    env = dict(os.environ, PATH=f'{bin_dir}:{os.environ["PATH"]}', CALLS=str(calls),
               FAKE_LOG=log, FAKE_HOME=str(home), FAKE_ARM_REPLY=reply,
               BRIDGE_IN=str(tmp_path / 'bridge_in.txt'),
               SOMA_DEPTHAI_PY=str(bin_dir / 'tracker'))
    done = subprocess.run(['bash', str(SCRIPT)] + args, input=typed, env=env,
                          capture_output=True, text=True)
    lines = calls.read_text().splitlines() if calls.exists() else []
    return done, lines


def armed_calls(calls):
    return [c for c in calls if 'service call /soma/arm' in c]


pytestmark = pytest.mark.skipif(shutil.which('bash') is None, reason='needs bash')


@pytest.mark.parametrize('args', [[], ['both'], ['--present']])
def test_a_missing_or_wrong_arm_is_refused_before_anything(tmp_path, args):
    done, calls = run(tmp_path, args)
    assert done.returncode == 2
    assert calls == []


def test_off_home_nothing_is_armed_and_no_take_starts(tmp_path):
    done, calls = run(tmp_path, ['left', '--present'], typed='arm\n', home=2)
    assert done.returncode == 3
    assert not armed_calls(calls)
    assert not any('--bridge' in c or c.startswith('tracker') for c in calls)


@pytest.mark.parametrize('typed', ['', '\n', 'yes\n', 'ARM\n', 'arm now\n'])
def test_without_the_word_arm_nothing_is_armed(tmp_path, typed):
    done, calls = run(tmp_path, ['left', '--present'], typed=typed)
    assert done.returncode == 1
    assert not armed_calls(calls)
    assert not any('--bridge' in c for c in calls)


def test_arm_typed_arms_once_then_runs_the_take(tmp_path):
    done, calls = run(tmp_path, ['left', '--present', '--lift', '0.785'], typed='arm\n')
    assert done.returncode == 0, done.stderr
    arms = armed_calls(calls)
    assert len(arms) == 1 and '{data: true}' in arms[0]
    bridge = next(i for i, c in enumerate(calls) if '--bridge' in c)
    home = next(i for i, c in enumerate(calls) if '--home-check' in c)
    assert home < calls.index(arms[0]) < bridge
    assert calls[bridge].endswith('--bridge left --present --lift 0.785')
    assert (tmp_path / 'bridge_in.txt').read_text() == '0.50\n'


def test_an_armed_driver_is_not_asked_or_armed_again(tmp_path):
    done, calls = run(tmp_path, ['left', '--present'], log=ARMED)
    assert done.returncode == 0, done.stderr
    assert not armed_calls(calls)
    assert not any('--home-check' in c for c in calls)
    assert any('--bridge left --present' in c for c in calls)


def test_a_driver_that_refuses_to_arm_stops_the_take(tmp_path):
    done, calls = run(tmp_path, ['left'], typed='arm\n', reply=NO)
    assert done.returncode == 3
    assert 'REFUSED by the driver' in done.stderr
    assert not any('--bridge' in c for c in calls)
