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

    def release(self, key):
        with self.lock:
            r = self.rows[key]
            self.bus.write_i2c_block_data(
                r['addr'], LED0 + 4 * r['ch'], [0, 0, 0, 0x10])
            self.last_us.pop(key, None)
            self.target_us.pop(key, None)

    def all_off(self):
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
        entry = self.data.setdefault(str(row['ch']), {'name': row['name']})
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
:root{--bg:#0f1113;--panel:#171a1d;--line:#262a2e;--ink:#e8eaec;--muted:#8b949e;
--go:#2f855a;--stop:#c53030;--accent:#3b82d8;--warn:#d69e2e}
body{font-family:system-ui,sans-serif;margin:0;background:var(--bg);color:var(--ink)}
header{display:flex;gap:.6rem;align-items:center;padding:.6rem 1rem;background:var(--panel);
position:sticky;top:0;z-index:2;border-bottom:1px solid var(--line)}
h1{font-size:1rem;margin:0;flex:1}
button{border:0;border-radius:8px;padding:.5rem .8rem;font-weight:700;cursor:pointer;color:#fff}
button:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
#arm{background:var(--go)}#arm.on{background:var(--stop)}
#alloff{background:var(--stop);font-size:1rem;padding:.6rem 1.1rem}
#msg{padding:.45rem 1rem;color:var(--warn);min-height:1.2rem;font-size:.85rem}
h2{font-size:.8rem;letter-spacing:.08em;text-transform:uppercase;color:var(--muted);
margin:1rem 1rem .3rem}
.servo{padding:.65rem 1rem;border-bottom:1px solid var(--line);display:grid;
grid-template-columns:minmax(12rem,auto) 1fr 9rem;gap:.35rem .8rem;align-items:center;opacity:.5}
.servo.active{opacity:1;background:#13201a;box-shadow:inset 3px 0 0 var(--go)}
.nm b{font-family:ui-monospace,monospace;color:var(--accent)}
.nm small{display:block;color:var(--muted);font-size:.75rem}
.rd{font-family:ui-monospace,monospace;text-align:right;font-variant-numeric:tabular-nums}
.rd .us{font-size:1.05rem}.rd .deg{display:block;color:var(--muted);font-size:.8rem}
input[type=range]{width:100%}
.row2{grid-column:1/4;display:flex;gap:.35rem;flex-wrap:wrap}
.row2 button{background:#2b3036;padding:.32rem .55rem;font-weight:600}
.row2 .sel{background:var(--go)}.row2 .zero{background:#285e8e}.row2 .mark{background:#1c4587}
.cal{grid-column:1/4;font-family:ui-monospace,monospace;font-size:.75rem;color:#9ad1a5}
@media (max-width:640px){.servo{grid-template-columns:1fr 7rem}.servo input{grid-column:1/3}}
</style></head><body>
<header><h1>SOMA servo workbench</h1>
<button id="arm" onclick="toggleArm()">ARM</button>
<button id="alloff" onclick="api({action:'all_off'})">ALL OFF</button></header>
<div id="msg">DISARMED. Arm, select ONE joint, press "go ZERO" first, then move it.</div>
<div id="list"></div>
<script>
let S={armed:false,active:null,servos:[]};
let built=false,dragging=null,pending={},timers={};
const deg=(us,z)=>((us-z)*0.09).toFixed(1);
function build(){
 const L=document.getElementById('list');L.innerHTML='';let board=null;
 for(const s of S.servos){
  if(s.addr!==board){board=s.addr;const h=document.createElement('h2');
   h.textContent='board 0x'+s.addr.toString(16)+(board===0x40?'  (right arm)':'  (left arm)');L.appendChild(h);}
  const k=s.key,d=document.createElement('div');d.id='sv'+k;d.className='servo';
  d.innerHTML=`<div class="nm"><b>ch${s.ch}</b> ${s.name.replace('_joint','')}
   <small>map zero ${s.zero_us} us · band ${s.band[0]}-${s.band[1]} us</small></div>
  <input type="range" id="sl${k}" min="500" max="2500" step="5" value="${s.zero_us}"
   aria-label="${s.name} pulse">
  <div class="rd"><span class="us" id="us${k}">off</span><span class="deg" id="dg${k}"></span></div>
  <div class="row2">
   <button class="sel" onclick="api({action:'select',key:'${k}'})">select</button>
   <button class="zero" onclick="setUs('${k}',${s.zero_us})">go ZERO</button>
   <button onclick="nudge('${k}',-50)">-50</button><button onclick="nudge('${k}',-10)">-10</button>
   <button onclick="nudge('${k}',-5)">-5</button><button onclick="nudge('${k}',5)">+5</button>
   <button onclick="nudge('${k}',10)">+10</button><button onclick="nudge('${k}',50)">+50</button>
   <button class="mark" onclick="api({action:'mark',key:'${k}',kind:'zero'})">set ZERO</button>
   <button class="mark" onclick="api({action:'mark',key:'${k}',kind:'min'})">mark MIN</button>
   <button class="mark" onclick="api({action:'mark',key:'${k}',kind:'max'})">mark MAX</button>
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
  d.className='servo'+(k===S.active?' active':'');
  document.getElementById('cal'+k).textContent=s.cal?
   `captured: zero ${s.cal.zero??'-'} · min ${s.cal.min??'-'} · max ${s.cal.max??'-'}`:'';
  if(dragging!==k&&pending[k]==null&&s.us){document.getElementById('sl'+k).value=s.us;show(k,s.us,s.zero_us);}
  if(!s.us&&dragging!==k){document.getElementById('us'+k).textContent='off';
   document.getElementById('dg'+k).textContent='';}}}
async function api(body){const r=await fetch('/api',{method:'POST',body:JSON.stringify(body)});
 const j=await r.json();S=j.state;
 if(body.action!=='state'&&j.msg)document.getElementById('msg').textContent=j.msg;
 if(!built)build();update();}
function setUs(k,v){const sl=document.getElementById('sl'+k);sl.value=v;
 const s=S.servos.find(x=>x.key===k);show(k,v,s.zero_us);api({action:'set',key:k,us:v});}
function nudge(k,d){setUs(k,+document.getElementById('sl'+k).value+d);}
function toggleArm(){api({action:S.armed?'disarm':'arm'})}
api({action:'state'});setInterval(()=>api({action:'state'}),3000);
</script></body></html>"""


def make_handler(fleet, cal):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _state(self):
            servos = []
            for key, r in fleet.rows.items():
                servos.append({**r, 'us': round(fleet.last_us.get(key, 0)) or None,
                               'cal': cal.data.get(str(r['ch']))})
            return {'armed': fleet.armed, 'active': fleet.active,
                    'fault': fleet.fault, 'servos': servos}

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
                fleet.active = key
                msg = f'{fleet.rows[key]["name"]} (board 0x{fleet.rows[key]["addr"]:02x} ch{fleet.rows[key]["ch"]}) is now the active servo.'
            elif act == 'set':
                msg = f'{key}: {fleet.command(key, req.get("us", fleet.rows[key]["zero_us"]))}'
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
