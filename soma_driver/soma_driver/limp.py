"""Which arm may hang with its signal cut, and when. Pure.

An arm servo is not self locking: cut its signal mid-air and the arm
falls, which is why no arm joint is in RELEASE_WHEN_SETTLED (CLAUDE.md
rule 3). Hanging at home is different: the arm already rests where
gravity puts it, so cutting the signal moves nothing. And holding it
there is worse than useless: the hanging zero is where gravity torque
crosses zero, the gear play has no side to rest on, and a held arm
hunts back and forth (the limit cycle of 2026-10-01). While one arm
works, the other can hang limp.

The driver's `limp_arm` parameter ('', 'left' or 'right') uses this:
it is refused unless every joint of that arm is at home, now and in its
target; while set, the arm's channels get FULL_OFF every tick and
commands for its joints are dropped. Clearing it ('') re-engages the arm
at home, the same small step as the first armed tick.
"""
from .servo_map import SERVO_MAP

ARMS = ('left', 'right')
AT_HOME_TOL = 1e-6


def arm_joints(arm: str) -> tuple[str, ...]:
    """The commanded joints of one arm, as SERVO_MAP names them."""
    return tuple(n for n in SERVO_MAP if n.startswith(f'{arm}_arm_'))


def limp_refusal(arm: str, current: dict[str, float],
                 target: dict[str, float]) -> str | None:
    """None if `arm` may go limp now, else why not."""
    if arm == '':
        return None
    if arm not in ARMS:
        return f"limp_arm must be '', 'left' or 'right', not {arm!r}"
    away = [n for n in arm_joints(arm)
            if abs(current[n] - SERVO_MAP[n].clamp(0.0)) > AT_HOME_TOL
            or abs(target[n] - SERVO_MAP[n].clamp(0.0)) > AT_HOME_TOL]
    if away:
        return (f'refused: the {arm} arm is not at home ({", ".join(away)}); '
                'released anywhere else it falls. Send it home first.')
    return None
