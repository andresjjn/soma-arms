#!/usr/bin/env python3
"""SOMA servo workbench: browser sliders to jog, zero and range-map each servo.

Runs on any Linux host with the PCA9685 boards on an I2C bus (Raspberry Pi
or Jetson Orin Nano, 40 pin header). Serves a single page web UI; move one
servo at a time, check which joint really answers on each output, capture
its mechanical zero and its physical min/max, and save everything to a JSON
file that later feeds the anchors in servo_map.py.

The joint list comes from SERVO_MAP, the single source of truth: board
address, channel, joint name and the calibrated zero all come from there,
so the page can never disagree with the driver about the wiring. (The old
version kept its own hand copied channel table, which went stale when the
harness changed.) The dormant L16 torso is left out on purpose.

Safety model (the project rules, encoded):
  - NEVER run it while the ROS driver is up: both would write the same
    boards. Disarm and stop the soma_driver container first.
  - Starts DISARMED with every output of every board in FULL_OFF. Arming
    is an explicit button, and nothing moves until a servo is selected.
  - ONE servo active at a time. Selecting another does not release the
    previous one (arm servos need holding torque), but only the active one
    accepts commands.
  - Pulses are clamped to 500-2500 us. The browser only sets TARGETS; a
    50 Hz server-side ramp walks the wire toward them at 400 us/s, so no
    slider gesture can snap a servo.
  - The very first pulse on an output snaps the servo from its unknown
    physical pose, once. Every slider therefore STARTS AT THE CALIBRATED
    ZERO from the map, which is where a hanging arm already rests, so that
    first snap is as small as the bench allows. Use "go ZERO" first.
  - Big ALL OFF button: every output of every board to FULL_OFF, disarmed.

Usage on the Jetson/Pi (driver container stopped):
    python3 scripts/servo_workbench.py            # autodetects the I2C bus
    python3 scripts/servo_workbench.py --bus 7    # or force one
Then open http://<host-ip>:8080 from any browser on the LAN.
"""
import argparse
import glob
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'soma_driver'))
from soma_driver.primitives import (  # noqa: E402
    HOME, SEQUENCES, pose_targets, settle_time_s)
from soma_driver.servo_map import SERVO_MAP  # noqa: E402

MODE1, PRESCALE, LED0, ALL_OFF_H = 0x00, 0xFE, 0x06, 0xFD
MIN_US, MAX_US = 500, 2500
RAMP_US_PER_S = 400.0     # smooth server-side ramp toward the slider target
TICK_S = 0.02             # 50 Hz ramp loop
US_PER_DEG = 2000.0 / 180.0
CAL_FILE = 'servo_calibration_2026-10-01.json'
SKIP = {'torso_lift_joint'}   # dormant, not on this bench


def joint_rows(servo_map=SERVO_MAP):
    """One row per commandable joint, ordered board by board, channel up.

    Pure, so the suite can pin it: every key unique, every zero inside its
    calibrated band, the dormant torso absent.
    """
    rows = []
    for name, spec in servo_map.items():
        if name in SKIP:
            continue
        lo_us, hi_us = sorted((spec.min_us, spec.max_us))
        rows.append({
            'key': f'{spec.address:02x}:{spec.channel}',
            'addr': spec.address, 'ch': spec.channel, 'name': name,
            'zero_us': round(spec.command_to_us(spec.clamp(0.0))),
            'band': [round(lo_us), round(hi_us)],
        })
    rows.sort(key=lambda r: (r['addr'], r['ch']))
    return rows


def raw_rows(addrs):
    """Every channel of every board, for testing outputs with a loose
    spare servo. Starts at the 1500 us neutral; no joint, no calibration."""
    return [{'key': f'{a:02x}:{ch}', 'addr': a, 'ch': ch,
             'name': f'raw output (spare servo test)',
             'zero_us': 1500, 'band': [MIN_US, MAX_US]}
            for a in sorted(addrs) for ch in range(16)]


def group_targets(rows, cal_data, value, cap, arms):
    """Master slider: one value in [-1, 1] drives every included joint.

    -1 walks each joint toward ITS captured MIN, +1 toward ITS captured MAX,
    0 is its zero; every joint uses its own measurements, so 50 % is half of
    that joint's own travel. |value| is capped (default 25 % in the page):
    twelve servos at full travel at once will collide (both yaws can swing
    about 180 deg inward). Falls back to the map zero and band when a
    capture is missing. Pure, so the suite can pin it.
    """
    v = max(-cap, min(cap, value))
    out = {}
    for r in rows:
        if r['name'].split('_')[0] not in arms:
            continue
        c = cal_data.get(str(r['ch']), {})
        zero = c.get('zero', r['zero_us'])
        lo = c.get('min', r['band'][0])
        hi = c.get('max', r['band'][1])
        us = zero + (v * (hi - zero) if v >= 0 else -v * (lo - zero))
        out[r['key']] = (zero, max(MIN_US, min(MAX_US, round(us))))
    return out


def output_key(spec):
    return f'{spec.address:02x}:{spec.channel}'


def plan_sequence(name):
    """Turn a primitives SEQUENCE into timed pulse steps. Pure.

    Same source of truth as the ROS driver (soma_driver/primitives.py):
    poses in radians, converted through SERVO_MAP (clamped), each step's
    move time is the driver's worst-case minimum-jerk travel at the joint
    rate cap, and the step lasts max(dwell, move). Joints a pose omits keep
    their previous target, exactly as on the driver.
    """
    current = dict(HOME)
    steps = []
    for pose, dwell in SEQUENCES[name]:
        targets = pose_targets(pose)
        move = settle_time_s(targets, current)
        current.update(targets)
        steps.append({
            'pose': pose, 'dwell': dwell, 'move_s': move,
            'us': {output_key(SERVO_MAP[j]): SERVO_MAP[j].command_to_us(v)
                   for j, v in targets.items()},
        })
    return steps


def min_jerk(t):
    """Normalized minimum-jerk position, t in [0, 1]."""
    t = max(0.0, min(1.0, t))
    return t * t * t * (10 - 15 * t + 6 * t * t)


class Fleet:
    """Every PCA9685 in the map on one bus, with the safety rules baked in."""

    def __init__(self, bus_num, rows):
        from smbus2 import SMBus
        self.bus = SMBus(bus_num)
        self.bus_num = bus_num
        self.rows = {r['key']: r for r in rows}
        self.addrs = sorted({r['addr'] for r in rows})
        self.armed = False
        self.active = None                 # key allowed to move
        self.last_us = {}                  # key -> pulse currently on the wire
        self.target_us = {}                # key -> where the slider wants it
        self.lock = threading.Lock()
        self.fault = None                  # set when a board reset under us
        self.player = None                 # sequence player status
        self.init_boards()
        self.all_off()
        threading.Thread(target=self._ramp_loop, daemon=True).start()
        threading.Thread(target=self._watchdog, daemon=True).start()

    def init_boards(self):
        for a in self.addrs:
            self.bus.write_byte_data(a, MODE1, 0x10)
            self.bus.write_byte_data(a, PRESCALE, 121)   # exactly 50.0 Hz
            self.bus.write_byte_data(a, MODE1, 0x20)
        time.sleep(0.01)

    def _watchdog(self):
        """Once a second: is every board still configured?

        Seen on the bench 2026-10-01: both boards lost their 3.3 V logic
        supply for an instant and came back at power-on defaults (MODE1
        0x11 = oscillator asleep, prescale 0x1E = 200 Hz). Every output
        went dead while this page still showed live pulses. A tool that
        says ARMED over sleeping boards is lying, so a reset disarms
        everything and says so; ARM re-configures the boards.
        """
        while True:
            time.sleep(1.0)
            bad = []
            for a in self.addrs:
                try:
                    mode1 = self.bus.read_byte_data(a, MODE1)
                    pre = self.bus.read_byte_data(a, PRESCALE)
                    if mode1 & 0x10 or pre != 121:
                        bad.append(f'0x{a:02x} reset (MODE1 0x{mode1:02x}, prescale 0x{pre:02x})')
                except OSError:
                    bad.append(f'0x{a:02x} not answering')
            if bad:
                with self.lock:
                    was_live = self.armed or bool(self.last_us)
                    self.armed = False
                    self.active = None
                    self.last_us.clear()
                    self.target_us.clear()
                    if was_live or self.fault is None:
                        self.fault = ('BOARD FAULT: ' + '; '.join(bad)
                                      + '. Disarmed. Check the 3.3 V logic supply, then ARM again.')
                        print(self.fault, flush=True)

    def _ramp_loop(self):
        """50 Hz: walk each commanded output smoothly toward its target.

        The browser can spam or reorder requests all it wants; the wire only
        ever sees this ramp. Same philosophy as rate_limit() in the driver.
        """
        while True:
            time.sleep(TICK_S)
            with self.lock:
                if not self.armed:
                    continue
                for key in list(self.target_us):
                    target, cur = self.target_us[key], self.last_us.get(key)
                    if cur is None or cur == target:
                        continue
                    step = RAMP_US_PER_S * TICK_S
                    new = target if abs(target - cur) <= step else (
                        cur + (step if target > cur else -step))
                    self._write_us(key, new)
                    self.last_us[key] = new

    def _write_us(self, key, us):
        r = self.rows[key]
        counts = round(us / 20000.0 * 4096.0)
        self.bus.write_i2c_block_data(
            r['addr'], LED0 + 4 * r['ch'], [0, 0, counts & 0xFF, counts >> 8])

    def command(self, key, us):
        with self.lock:
            if not self.armed:
                return 'refused: DISARMED'
            if key != self.active:
                return 'refused: not the active servo'
            us = max(MIN_US, min(MAX_US, float(us)))
            if key not in self.last_us:
                # First pulse on this output: the servo snaps to it from
                # wherever it physically is. One unavoidable jump; from here
                # on, everything is ramped. Start from the map zero.
                self._write_us(key, us)
                self.last_us[key] = us
            self.target_us[key] = us
            return f'target {us:.0f} us'

    def group(self, targets):
        """Set many targets at once; the 50 Hz ramp moves them together."""
        with self.lock:
            if not self.armed:
                return 'refused: DISARMED'
            for key, (zero, us) in targets.items():
                if key not in self.last_us:
                    # first pulse: snap to the zero (the hanging pose),
                    # never straight to a far target
                    self._write_us(key, zero)
                    self.last_us[key] = zero
                self.target_us[key] = us
            self.active = None
            return f'group: {len(targets)} joints ramping'

    # ---- sequence player -------------------------------------------------
    def play(self, name, step_mode):
        with self.lock:
            if not self.armed:
                return 'refused: DISARMED'
            if getattr(self, 'raw', False):
                return 'refused: no sequences in raw test mode.'
            if self.player and self.player.get('running'):
                return 'refused: a sequence is already playing.'
            try:
                steps = plan_sequence(name)
            except KeyError:
                return f'refused: unknown sequence {name}'
            missing = {k for st in steps for k in st['us']} - set(self.rows)
            if missing:
                return f'refused: sequence needs outputs not on the bus: {sorted(missing)}'
            self.active = None
            self.player = {'name': name, 'running': True, 'step': 0,
                           'of': len(steps), 'pose': None, 'step_mode': step_mode,
                           'waiting': False}
            self._next = threading.Event()
            self._stop = threading.Event()
        threading.Thread(target=self._play, args=(steps,), daemon=True).start()
        return f'playing {name} ({len(steps)} steps{", step by step" if step_mode else ""})'

    def player_next(self):
        if self.player and self.player.get('waiting'):
            self._next.set()
            return 'next step'
        return 'refused: the player is not waiting.'

    def player_stop(self):
        if self.player and self.player.get('running'):
            self._stop.set()
            self._next.set()
            return 'sequence stopped: every joint holds where it is.'
        return 'nothing is playing.'

    def _play(self, steps):
        try:
            for i, st in enumerate(steps):
                with self.lock:
                    if not self.armed or self._stop.is_set():
                        return
                    self.player.update(step=i + 1, pose=st['pose'], waiting=False)
                    start = {}
                    for key in st['us']:
                        if key not in self.last_us:
                            # first pulse: the hanging zero, never a far target
                            zero = self.rows[key]['zero_us']
                            self._write_us(key, zero)
                            self.last_us[key] = zero
                        start[key] = self.last_us[key]
                t0, move = time.monotonic(), max(st['move_s'], TICK_S)
                while True:
                    tau = (time.monotonic() - t0) / move
                    with self.lock:
                        if not self.armed or self._stop.is_set():
                            return
                        k = min_jerk(tau)
                        for key, goal in st['us'].items():
                            us = start[key] + k * (goal - start[key])
                            self._write_us(key, us)
                            self.last_us[key] = us
                            self.target_us[key] = us
                    if tau >= 1.0:
                        break
                    time.sleep(TICK_S)
                rest = max(st['dwell'], st['move_s']) - (time.monotonic() - t0)
                if self._stop.wait(max(0.0, rest)):
                    return
                if self.player['step_mode'] and i + 1 < len(steps):
                    self.player['waiting'] = True
                    self._next.wait()
                    self._next.clear()
                    if self._stop.is_set():
                        return
        finally:
            if self.player:
                self.player.update(running=False, waiting=False)

    def release(self, key):
        with self.lock:
            r = self.rows[key]
            self.bus.write_i2c_block_data(
                r['addr'], LED0 + 4 * r['ch'], [0, 0, 0, 0x10])
            self.last_us.pop(key, None)
            self.target_us.pop(key, None)

    def all_off(self):
        if getattr(self, '_stop', None) is not None:
            self._stop.set()
            self._next.set()
        # A kill switch must not depend on every board being healthy: cut
        # each board on its own, so a missing or flaky one cannot stop the
        # cut from reaching the rest.
        with self.lock:
            for a in self.addrs:
                try:
                    self.bus.write_byte_data(a, ALL_OFF_H, 0x10)
                except OSError as exc:
                    print(f'ALL OFF: board 0x{a:02x} did not answer ({exc})')
            self.last_us.clear()
            self.target_us.clear()
            self.armed = False
            self.active = None


def present_boards(addrs, forced=None):
    """Find the bus and which of the map's boards answer on it.

    Returns (bus_number, present_addresses). A bench with one board
    unplugged still gets a workbench for the boards that are there; the
    missing board's joints are simply left off the page.
    """
    from smbus2 import SMBus
    candidates = ([forced] if forced is not None else
                  sorted(int(p.rsplit('-', 1)[1]) for p in glob.glob('/dev/i2c-*')))
    for n in candidates:
        found = []
        try:
            with SMBus(n) as b:
                for a in addrs:
                    try:
                        b.read_byte_data(a, MODE1)
                        found.append(a)
                    except OSError:
                        pass
        except OSError:
            continue
        if found:
            return n, found
    raise SystemExit(
        'No board of the map answers on any bus: ' + ', '.join(hex(a) for a in addrs)
        + '. Check wiring, address bridges, and that your user can read '
        '/dev/i2c-* (or use sudo).')


class Cal:
    def __init__(self):
        self.data = {}
        if os.path.exists(CAL_FILE):
            with open(CAL_FILE) as f:
                self.data = json.load(f)

    def mark(self, row, kind, us):
        # Keyed by channel (unique across both boards in the current map),
        # the format scripts/apply_calibration.py already reads.
        entry = self.data.setdefault(str(row['ch']), {})
        # Always refresh the label: a name kept from the first capture went
        # stale on 2026-10-01 when the map changed under existing entries.
        entry['name'] = row['name']
        entry['address'] = hex(row['addr'])
        entry[kind] = round(us)
        entry['date'] = time.strftime('%Y-%m-%d')

    def save(self):
        with open(CAL_FILE, 'w') as f:
            json.dump(self.data, f, indent=2, sort_keys=True)
        return os.path.abspath(CAL_FILE)


PAGE = """<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SOMA servo workbench</title><style>
:root{--bg:#0f1113;--panel:#171a1d;--card:#14171a;--line:#262a2e;--ink:#e8eaec;--muted:#8b949e;
--go:#2f855a;--stop:#c53030;--accent:#3b82d8;--warn:#d69e2e;--right:#3b82d8;--left:#c2793a}
*{box-sizing:border-box}
body{font-family:system-ui,sans-serif;margin:0;background:var(--bg);color:var(--ink)}
header{display:flex;gap:.6rem;align-items:center;padding:.6rem 1rem;background:var(--panel);
position:sticky;top:0;z-index:3;border-bottom:1px solid var(--line)}
h1{font-size:1rem;margin:0;flex:1}
button{border:0;border-radius:8px;padding:.5rem .8rem;font-weight:700;cursor:pointer;color:#fff}
button:focus-visible,input:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
#arm{background:var(--go)}#arm.on{background:var(--stop)}
#alloff{background:var(--stop);font-size:1rem;padding:.6rem 1.1rem}
#msg{padding:.45rem 1rem;color:var(--warn);min-height:1.2rem;font-size:.85rem}
.master{margin:.4rem 1rem 1rem;padding:1rem 1.1rem;border:1px solid #3a3f45;border-radius:12px;
background:linear-gradient(0deg,#191c1f,#191c1f)}
.master h2{margin:0 0 .2rem;font-size:.95rem}
.master p{margin:0 0 .8rem;color:var(--muted);font-size:.8rem}
.mrow{display:grid;grid-template-columns:4.5rem 1fr 4.5rem;gap:.6rem;align-items:center}
.mrow .end{font-size:.75rem;color:var(--muted)}.mrow .end:last-child{text-align:right}
#mval{font-family:ui-monospace,monospace;font-size:1.2rem;text-align:center;margin:.3rem 0}
.mctl{display:flex;flex-wrap:wrap;gap:1rem;align-items:center;margin-top:.7rem;font-size:.85rem}
.mctl label{display:flex;gap:.35rem;align-items:center;cursor:pointer}
.mctl button{background:#2b3036;padding:.35rem .7rem}
#mgo{background:var(--go)}
.arms{display:grid;grid-template-columns:1fr 1fr;gap:1rem;padding:0 1rem 2rem}
.arm{border:1px solid var(--line);border-radius:12px;background:var(--card);overflow:hidden}
.arm>h2{margin:0;padding:.6rem .9rem;font-size:.9rem;display:flex;justify-content:space-between}
.arm.right>h2{border-bottom:3px solid var(--right)}.arm.left>h2{border-bottom:3px solid var(--left)}
.arm>h2 small{color:var(--muted);font-weight:400}
.servo{padding:.6rem .9rem;border-bottom:1px solid var(--line);display:grid;
grid-template-columns:1fr 7.5rem;gap:.3rem .7rem;align-items:center;opacity:.55}
.servo:last-child{border-bottom:0}
.servo.active{opacity:1;background:#13201a;box-shadow:inset 3px 0 0 var(--go)}
.servo.grp{opacity:1}
.nm b{font-family:ui-monospace,monospace;color:var(--accent)}
.nm small{display:block;color:var(--muted);font-size:.72rem}
.rd{font-family:ui-monospace,monospace;text-align:right;font-variant-numeric:tabular-nums}
.rd .us{font-size:1rem}.rd .deg{display:block;color:var(--muted);font-size:.75rem}
.servo input[type=range]{grid-column:1/3;width:100%}
input[type=range]{width:100%}
.row2{grid-column:1/3;display:flex;gap:.3rem;flex-wrap:wrap}
.row2 button{background:#2b3036;padding:.3rem .5rem;font-weight:600;font-size:.8rem}
.row2 .sel{background:var(--go)}.row2 .zero{background:#285e8e}.row2 .mark{background:#1c4587}
.cal{grid-column:1/3;font-family:ui-monospace,monospace;font-size:.72rem;color:#9ad1a5}
@media (max-width:900px){.arms{grid-template-columns:1fr}}
</style></head><body>
<header><h1>SOMA servo workbench</h1>
<button id="arm" onclick="toggleArm()">ARM</button>
<button id="alloff" onclick="api({action:'all_off'})">ALL OFF</button></header>
<div id="msg">DISARMED. Arm, then use the master slider or select ONE joint.</div>
<section class="master" aria-label="Master slider">
 <h2>Master: every joint at once</h2>
 <p>Left walks each joint toward its own captured MIN, right toward its own MAX, center is zero.
 Ramped at 400 us/s. Start small: at full travel the arms collide.</p>
 <div id="mval">0 %</div>
 <div class="mrow"><span class="end">MIN</span>
  <input type="range" id="msl" min="-100" max="100" step="5" value="0" aria-label="Master slider">
  <span class="end">MAX</span></div>
 <div class="mctl">
  <label><input type="checkbox" id="mr" checked> right arm</label>
  <label><input type="checkbox" id="ml" checked> left arm</label>
  <label>amplitude cap
   <select id="mcap"><option value="0.1">10 %</option><option value="0.25" selected>25 %</option>
   <option value="0.5">50 %</option><option value="0.75">75 %</option><option value="1">100 %</option></select></label>
  <button onclick="msl.value=0;mshow();msend()">all to ZERO</button>
 </div>
</section>
<section class="master" aria-label="Sequences">
 <h2>Sequences</h2>
 <p>Played exactly as the ROS driver would: poses from primitives.py, minimum-jerk at the 2.5 rad/s cap.
 Use step by step the first time and check every direction before the next click.</p>
 <div class="mctl">
  <select id="seq" aria-label="Sequence"></select>
  <button id="sstep" onclick="play(true)">Step by step</button>
  <button id="splay" onclick="play(false)" style="background:var(--go)">Play</button>
  <button id="snext" onclick="api({action:'next'})" style="background:#285e8e">Next step</button>
  <button id="sstop" onclick="api({action:'stop'})" style="background:var(--stop)">Stop</button>
  <span id="sstat" style="font-family:ui-monospace,monospace;color:var(--muted)"></span>
 </div>
</section>
<div class="arms">
 <section class="arm right"><h2>Right arm <small>board 0x40 · ch15-10</small></h2><div id="list-right"></div></section>
 <section class="arm left"><h2>Left arm <small>board 0x43 · ch9-4</small></h2><div id="list-left"></div></section>
</div>
<script>
let S={armed:false,active:null,servos:[]};
let built=false,dragging=null,pending={},timers={},mtimer=null;
const msl=document.getElementById('msl');
const deg=(us,z)=>((us-z)*0.09).toFixed(1);
function cap(){return +document.getElementById('mcap').value}
function mshow(){const v=Math.max(-cap()*100,Math.min(cap()*100,+msl.value));
 document.getElementById('mval').textContent=(v>0?'+':'')+v+' %'+(Math.abs(+msl.value)>cap()*100?'  (capped)':'');}
function msend(){const arms=[];if(document.getElementById('mr').checked)arms.push('right');
 if(document.getElementById('ml').checked)arms.push('left');
 api({action:'group',value:+msl.value/100,cap:cap(),arms:arms});}
msl.addEventListener('input',()=>{mshow();if(!mtimer)mtimer=setTimeout(()=>{mtimer=null;msend();},150);});
msl.addEventListener('pointerup',msend);
document.getElementById('mcap').addEventListener('change',()=>{mshow();msend();});
function build(){
 for(const side of ['right','left'])document.getElementById('list-'+side).innerHTML='';
 for(const s of S.servos){
  const side=s.name.startsWith('left')?'left':'right';
  const L=document.getElementById('list-'+side);
  const k=s.key,d=document.createElement('div');d.id='sv'+k;d.className='servo';
  d.innerHTML=`<div class="nm"><b>ch${s.ch}</b> ${s.name.replace(side+'_arm_','').replace('_joint','')}
   <small>zero ${s.zero_us} us · map band ${s.band[0]}-${s.band[1]}</small></div>
  <div class="rd"><span class="us" id="us${k}">off</span><span class="deg" id="dg${k}"></span></div>
  <input type="range" id="sl${k}" min="500" max="2500" step="5" value="${s.zero_us}" aria-label="${s.name} pulse">
  <div class="row2">
   <button class="sel" onclick="api({action:'select',key:'${k}'})">select</button>
   <button class="zero" onclick="setUs('${k}',${s.zero_us})">go ZERO</button>
   <button onclick="nudge('${k}',-50)">-50</button><button onclick="nudge('${k}',-10)">-10</button>
   <button onclick="nudge('${k}',-5)">-5</button><button onclick="nudge('${k}',5)">+5</button>
   <button onclick="nudge('${k}',10)">+10</button><button onclick="nudge('${k}',50)">+50</button>
   <button class="mark" onclick="api({action:'mark',key:'${k}',kind:'zero'})">ZERO</button>
   <button class="mark" onclick="api({action:'mark',key:'${k}',kind:'min'})">MIN</button>
   <button class="mark" onclick="api({action:'mark',key:'${k}',kind:'max'})">MAX</button>
   <button onclick="api({action:'release',key:'${k}'})">release</button>
  </div><div class="cal" id="cal${k}"></div>`;
  L.appendChild(d);
  const sl=d.querySelector('input');
  sl.addEventListener('pointerdown',()=>dragging=k);
  sl.addEventListener('pointerup',()=>{dragging=null;flush(k);});
  sl.addEventListener('input',()=>{show(k,+sl.value,s.zero_us);pending[k]=+sl.value;
   if(!timers[k])timers[k]=setTimeout(()=>flush(k),120);});
 }
 built=true;
}
function show(k,us,z){document.getElementById('us'+k).textContent=us+' us';
 document.getElementById('dg'+k).textContent=deg(us,z)+'° vs zero';}
function flush(k){clearTimeout(timers[k]);timers[k]=null;
 if(pending[k]!=null){const v=pending[k];pending[k]=null;api({action:'set',key:k,us:v});}}
function update(){
 if(S.fault)document.getElementById('msg').textContent=S.fault;
 const a=document.getElementById('arm');a.textContent=S.armed?'DISARM':'ARM';a.className=S.armed?'on':'';
 for(const s of S.servos){const k=s.key,d=document.getElementById('sv'+k);if(!d)continue;
  d.className='servo'+(k===S.active?' active':(s.us&&!S.active?' grp':''));
  document.getElementById('cal'+k).textContent=s.cal?
   `captured: zero ${s.cal.zero??'-'} · min ${s.cal.min??'-'} · max ${s.cal.max??'-'}`:'';
  if(dragging!==k&&pending[k]==null&&s.us){document.getElementById('sl'+k).value=s.us;show(k,s.us,s.zero_us);}
  if(!s.us&&dragging!==k){document.getElementById('us'+k).textContent='off';
   document.getElementById('dg'+k).textContent='';}}}
async function api(body){const r=await fetch('/api',{method:'POST',body:JSON.stringify(body)});
 const j=await r.json();S=j.state;
 if(body.action!=='state'&&j.msg)document.getElementById('msg').textContent=j.msg;
 if(!built)build();update();updateSeq();}
function setUs(k,v){const sl=document.getElementById('sl'+k);sl.value=v;
 const s=S.servos.find(x=>x.key===k);show(k,v,s.zero_us);api({action:'set',key:k,us:v});}
function nudge(k,d){setUs(k,+document.getElementById('sl'+k).value+d);}
let seqBuilt=false;
function play(step){api({action:'play',name:document.getElementById('seq').value,step_mode:step});}
function updateSeq(){
 if(!seqBuilt&&S.sequences){const sel=document.getElementById('seq');
  for(const n of S.sequences){const o=document.createElement('option');o.textContent=n;sel.appendChild(o);}seqBuilt=true;}
 const P=S.player,st=document.getElementById('sstat');
 st.textContent=P?(P.running?`${P.name}: step ${P.step}/${P.of} (${P.pose})${P.waiting?' · waiting for Next':''}`:`${P.name}: finished`):'';
 document.getElementById('snext').disabled=!(P&&P.waiting);}
function toggleArm(){api({action:S.armed?'disarm':'arm'})}
api({action:'state'});setInterval(()=>api({action:'state'}),1000);
</script></body></html>"""


def make_handler(fleet, cal):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _state(self):
            servos = []
            for key, r in fleet.rows.items():
                c = cal.data.get(str(r['ch'])) or {}
                # go ZERO prefers today's captured zero over the map's
                servos.append({**r, 'zero_us': c.get('zero', r['zero_us']),
                               'us': round(fleet.last_us.get(key, 0)) or None,
                               'cal': c or None})
            return {'armed': fleet.armed, 'active': fleet.active,
                    'fault': fleet.fault, 'player': fleet.player,
                    'sequences': sorted(SEQUENCES), 'servos': servos}

        def _send(self, code, body, ctype='application/json'):
            data = body.encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            self._send(200, PAGE, 'text/html')

        def do_POST(self):
            req = json.loads(self.rfile.read(
                int(self.headers.get('Content-Length', 0)) or 0) or '{}')
            act, key = req.get('action'), req.get('key')
            if key is not None and key not in fleet.rows:
                self._send(200, json.dumps({'state': self._state(),
                                            'msg': f'refused: unknown output {key}'}))
                return
            msg = ''
            if act == 'arm':
                with fleet.lock:
                    fleet.init_boards()          # a reset board comes back here
                    fleet.fault = None
                    fleet.armed = True
                msg = 'ARMED (boards re-configured). Select ONE joint and press "go ZERO" before moving it.'
            elif act == 'disarm':
                fleet.armed = False
                msg = 'DISARMED (outputs keep their last pulse; use release/ALL OFF to cut).'
            elif act == 'all_off':
                fleet.all_off()
                msg = 'ALL OFF: every output of every board released, disarmed.'
            elif act == 'select':
                fleet.player_stop()
                fleet.active = key
                msg = f'{fleet.rows[key]["name"]} (board 0x{fleet.rows[key]["addr"]:02x} ch{fleet.rows[key]["ch"]}) is now the active servo.'
            elif act == 'set':
                msg = f'{key}: {fleet.command(key, req.get("us", fleet.rows[key]["zero_us"]))}'
            elif act == 'group':
                fleet.player_stop()
                if getattr(fleet, 'raw', False):
                    msg = 'refused: no master slider in raw test mode.'
                else:
                    t = group_targets(list(fleet.rows.values()), cal.data,
                                      float(req.get('value', 0.0)),
                                      float(req.get('cap', 0.25)),
                                      set(req.get('arms', [])))
                    msg = fleet.group(t) if t else 'refused: no arm selected.'
            elif act == 'play':
                msg = fleet.play(req.get('name', ''), bool(req.get('step_mode')))
            elif act == 'next':
                msg = fleet.player_next()
            elif act == 'stop':
                msg = fleet.player_stop()
            elif act == 'release':
                fleet.release(key)
                msg = f'{key} released (signal cut).'
            elif act == 'mark':
                us = fleet.last_us.get(key)
                if getattr(fleet, 'raw', False):
                    msg = 'refused: raw test mode never writes calibration.'
                elif us is None:
                    msg = 'refused: servo has no commanded pulse yet.'
                else:
                    cal.mark(fleet.rows[key], req.get('kind'), us)
                    path = cal.save()
                    msg = f'{key} {req.get("kind")} = {us:.0f} us saved to {path}'
            self._send(200, json.dumps({'state': self._state(), 'msg': msg}))
    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bus', type=int, default=None, help='I2C bus number')
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--raw', action='store_true',
                    help='every channel of every board, for a loose spare '
                         'servo; calibration capture disabled')
    args = ap.parse_args()

    rows = joint_rows()
    addrs = sorted({r['addr'] for r in rows})
    bus_num, present = present_boards(addrs, args.bus)
    missing = [a for a in addrs if a not in present]
    if missing:
        print('WARNING: boards not answering, their joints are left out: '
              + ', '.join(hex(a) for a in missing))
    rows = raw_rows(present) if args.raw else [
        r for r in rows if r['addr'] in present]
    fleet = Fleet(bus_num, rows)
    fleet.raw = args.raw
    cal = Cal()
    print(f'Boards {", ".join(hex(a) for a in present)} on /dev/i2c-{bus_num}: '
          f'50 Hz set, ALL OFF, DISARMED. {len(rows)} '
          + ('raw outputs (spare servo test).' if args.raw else 'joints from SERVO_MAP.'))
    print(f'Open http://<this-host>:{args.port}  (calibration -> {CAL_FILE})')
    server = ThreadingHTTPServer(('0.0.0.0', args.port), make_handler(fleet, cal))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        fleet.all_off()
        print('\nALL OFF sent to every board. Bye.')


if __name__ == '__main__':
    main()
