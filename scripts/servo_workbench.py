#!/usr/bin/env python3
"""SOMA servo workbench: browser sliders to jog, zero and range-map each servo.

Runs on any Linux host with the PCA9685 on an I2C bus (Raspberry Pi or
Jetson Orin Nano, 40 pin header). Serves a single page web UI; move one
servo at a time, capture its mechanical zero and its physical min/max, and
save everything to a JSON file that later feeds the offsets in servo_map.py.

Safety model (the project rules, encoded):
  - Starts DISARMED with every output in FULL_OFF. Arming is an explicit
    button, and nothing moves until a servo is selected on purpose.
  - ONE servo active at a time. Selecting another does not release the
    previous one (arm servos need holding torque), but only the active one
    accepts commands.
  - Pulses are clamped to 500-2500 us. The browser only sets TARGETS; a
    50 Hz server-side ramp walks the wire toward them at 400 us/s, so no
    slider gesture can snap a servo. Exception: the very first pulse on a
    channel snaps the servo from its unknown physical pose, once — keep the
    arm resting when you arm.
  - Big ALL OFF button: every channel to FULL_OFF, disarmed.
  - The L16 torso (ch 3) is included WITH ITS OWN PHYSICS: band clamped to
    1000-2000 us, ramp matched to its real 20 mm/s, and auto release 2 s
    after settling, so a command can never be held against a stop (it
    wedged once on 2026-07-22; see docs/bench.md for the slow stop-probing
    procedure: approach stops with the +/-10 nudges, never park on them).

Bring-up order (docs/wiring.md): wire with the servo rail OFF, run this
tool and check the bus scan, fold the arms into a compact resting pose,
only then energise the 6 V rail, and only then arm.

Usage on the Jetson/Pi:
    sudo apt install -y python3-smbus2 || pip3 install smbus2 --break-system-packages
    python3 scripts/servo_workbench.py            # autodetects the I2C bus
    python3 scripts/servo_workbench.py --bus 7    # or force one
Then open http://<host-ip>:8080 from any browser on the LAN.
"""
import argparse
import glob
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ADDR = 0x40
MODE1, PRESCALE, LED0, ALL_OFF_H = 0x00, 0xFE, 0x06, 0xFD
MIN_US, MAX_US, CENTER_US = 500, 2500, 1500
RAMP_US_PER_S = 400.0     # smooth server-side ramp toward the slider target
TICK_S = 0.02             # 50 Hz ramp loop
CAL_FILE = 'servo_calibration.json'

# Measured wiring 2026-07-22 (docs/wiring.md).
L16_CH = 3
SERVOS = [
    (15, 'right gripper'), (14, 'right wrist roll'), (13, 'right wrist pitch'),
    (12, 'right elbow'), (11, 'right shoulder'), (10, 'right yaw'),
    (9, 'left gripper'), (8, 'left wrist roll'), (7, 'left wrist pitch'),
    (6, 'left elbow'), (5, 'left shoulder'), (4, 'left yaw'),
    (3, 'torso L16 (INVERTED: 2000us = retracted)'),
]

# The L16 gets its own physics (it wedged once, 2026-07-22, docs/bench.md):
#   - band clamped to its real 1.0-2.0 ms interface, never the servo band
#   - ramp matched to its 20 mm/s (140 mm over 1000 us -> ~140 us/s), so
#     the signal can never run ahead of the rod and lean on a stop
#   - AUTO-RELEASE: 2 s after the signal settles, the channel is cut. The
#     lead screw is self locking (46 N unpowered), nothing sags, and a
#     command can never be held against a mechanical stop.
CH_LIMITS = {L16_CH: (1000.0, 2000.0)}
CH_RAMP = {L16_CH: 140.0}
CH_RELEASE_S = {L16_CH: 2.0}


class Board:
    """Thin PCA9685 driver with the safety rules baked in."""

    def __init__(self, bus_num):
        from smbus2 import SMBus
        self.bus = SMBus(bus_num)
        self.bus_num = bus_num
        self.armed = False
        self.active = None                 # channel allowed to move
        self.last_us = {}                  # channel -> pulse currently on the wire
        self.target_us = {}                # channel -> where the slider wants it
        self.lock = threading.Lock()
        self.bus.write_byte_data(ADDR, MODE1, 0x10)
        self.bus.write_byte_data(ADDR, PRESCALE, 121)   # exactly 50.0 Hz
        self.bus.write_byte_data(ADDR, MODE1, 0x20)
        time.sleep(0.01)
        self.all_off()
        threading.Thread(target=self._ramp_loop, daemon=True).start()

    def _ramp_loop(self):
        """50 Hz: walk each commanded channel smoothly toward its target.

        The browser can spam or reorder requests all it wants; the wire only
        ever sees this ramp. Same philosophy as rate_limit() in the driver.
        """
        settled = {}
        while True:
            time.sleep(TICK_S)
            with self.lock:
                if not self.armed:
                    continue
                for ch in list(self.target_us):
                    target = self.target_us[ch]
                    cur = self.last_us.get(ch)
                    if cur is None:
                        continue
                    if cur == target:
                        # settled: self locking channels get their signal cut
                        settled[ch] = settled.get(ch, 0.0) + TICK_S
                        if settled[ch] >= CH_RELEASE_S.get(ch, float('inf')):
                            self.bus.write_i2c_block_data(
                                ADDR, LED0 + 4 * ch, [0, 0, 0, 0x10])
                            self.last_us.pop(ch, None)
                            self.target_us.pop(ch, None)
                            settled.pop(ch, None)
                        continue
                    settled[ch] = 0.0
                    step = CH_RAMP.get(ch, RAMP_US_PER_S) * TICK_S
                    if abs(target - cur) <= step:
                        new = target
                    else:
                        new = cur + (step if target > cur else -step)
                    self._write_us(ch, new)
                    self.last_us[ch] = new

    def _write_us(self, ch, us):
        counts = round(us / 20000.0 * 4096.0)
        self.bus.write_i2c_block_data(
            ADDR, LED0 + 4 * ch, [0, 0, counts & 0xFF, counts >> 8])

    def command(self, ch, us):
        with self.lock:
            if not self.armed:
                return 'refused: DISARMED'
            if ch != self.active:
                return 'refused: not the active servo'
            lo, hi = CH_LIMITS.get(ch, (MIN_US, MAX_US))
            us = max(lo, min(hi, float(us)))
            if ch not in self.last_us:
                # First pulse on this channel: the servo snaps to it from
                # wherever it physically is. One unavoidable jump; from here
                # on, everything is ramped. Keep the arm resting.
                self._write_us(ch, us)
                self.last_us[ch] = us
            self.target_us[ch] = us
            return f'target {us:.0f}'

    def release(self, ch):
        with self.lock:
            self.bus.write_i2c_block_data(ADDR, LED0 + 4 * ch, [0, 0, 0, 0x10])
            self.last_us.pop(ch, None)
            self.target_us.pop(ch, None)

    def all_off(self):
        with self.lock:
            self.bus.write_byte_data(ADDR, ALL_OFF_H, 0x10)
            self.last_us.clear()
            self.target_us.clear()
            self.armed = False
            self.active = None


def find_bus(forced=None):
    """Scan /dev/i2c-* for a PCA9685 answering at 0x40 with our prescale reg."""
    from smbus2 import SMBus
    candidates = ([forced] if forced is not None else
                  sorted(int(p.rsplit('-', 1)[1]) for p in glob.glob('/dev/i2c-*')))
    for n in candidates:
        try:
            with SMBus(n) as b:
                b.read_byte_data(ADDR, MODE1)
            return n
        except OSError:
            continue
    raise SystemExit('No PCA9685 at 0x40 on any I2C bus. Check wiring and '
                     'that your user can read /dev/i2c-* (or use sudo).')


class Cal:
    def __init__(self):
        self.data = {}
        if os.path.exists(CAL_FILE):
            with open(CAL_FILE) as f:
                self.data = json.load(f)

    def mark(self, ch, name, kind, us):
        entry = self.data.setdefault(str(ch), {'name': name})
        entry[kind] = round(us)
        entry['date'] = time.strftime('%Y-%m-%d')

    def save(self):
        with open(CAL_FILE, 'w') as f:
            json.dump(self.data, f, indent=2, sort_keys=True)
        return os.path.abspath(CAL_FILE)


PAGE = """<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SOMA servo workbench</title><style>
body{font-family:system-ui;margin:0;background:#111;color:#eee}
header{display:flex;gap:.6rem;align-items:center;padding:.6rem 1rem;background:#1b1b1b;position:sticky;top:0}
h1{font-size:1rem;margin:0;flex:1}
button{border:0;border-radius:8px;padding:.55rem .9rem;font-weight:700;cursor:pointer}
#arm{background:#2c6e49;color:#fff}#arm.on{background:#c92a2a}
#alloff{background:#c92a2a;color:#fff;font-size:1rem}
.servo{padding:.7rem 1rem;border-bottom:1px solid #2a2a2a;display:grid;
grid-template-columns:auto 1fr auto;gap:.5rem;align-items:center;opacity:.45}
.servo.active{opacity:1;background:#16211b}
.servo .nm{min-width:11rem}.servo .us{font-variant-numeric:tabular-nums;min-width:4.5rem;text-align:right}
input[type=range]{width:100%}
.row2{grid-column:1/4;display:flex;gap:.4rem;flex-wrap:wrap}
.row2 button{background:#333;color:#eee;padding:.35rem .6rem;font-weight:600}
.row2 .mark{background:#1c4587}
.badge{font-size:.72rem;color:#9ad1a5}
#msg{padding:.4rem 1rem;color:#ffd43b;min-height:1.2rem;font-size:.85rem}
</style></head><body>
<header><h1>SOMA servo workbench</h1>
<button id="arm" onclick="toggleArm()">ARM</button>
<button id="alloff" onclick="api({action:'all_off'})">ALL OFF</button></header>
<div id="msg">DISARMED. Select a servo, then arm. One servo moves at a time.</div>
<div id="list"></div>
<script>
let S={armed:false,active:null,servos:[]};
let built=false, dragging=null, pending={}, timers={};

function build(){
 const L=document.getElementById('list');L.innerHTML='';
 for(const s of S.servos){
  const d=document.createElement('div');d.id='sv'+s.ch;d.className='servo';
  d.innerHTML=`<div class="nm"><b>ch${s.ch}</b> ${s.name} <span class="badge" id="cal${s.ch}"></span></div>
  <input type="range" id="sl${s.ch}" min="${s.min}" max="${s.max}" step="5"
   value="${(s.min+s.max)/2}">
  <div class="us" id="us${s.ch}">off</div>
  <div class="row2">
   <button onclick="api({action:'select',ch:${s.ch}})">select</button>
   <button onclick="nudge(${s.ch},-50)">-50</button><button onclick="nudge(${s.ch},-10)">-10</button>
   <button onclick="nudge(${s.ch},10)">+10</button><button onclick="nudge(${s.ch},50)">+50</button>
   <button class="mark" onclick="api({action:'mark',ch:${s.ch},kind:'zero'})">set ZERO</button>
   <button class="mark" onclick="api({action:'mark',ch:${s.ch},kind:'min'})">mark MIN</button>
   <button class="mark" onclick="api({action:'mark',ch:${s.ch},kind:'max'})">mark MAX</button>
   <button onclick="api({action:'release',ch:${s.ch}})">release</button>
  </div>`;
  L.appendChild(d);
  const sl=d.querySelector('input');
  sl.addEventListener('pointerdown',()=>dragging=s.ch);
  sl.addEventListener('pointerup',()=>{dragging=null;flush(s.ch);});
  sl.addEventListener('input',()=>{
   document.getElementById('us'+s.ch).textContent=sl.value+'us';
   pending[s.ch]=+sl.value;
   if(!timers[s.ch])timers[s.ch]=setTimeout(()=>flush(s.ch),120);});
 }
 built=true;
}
function flush(ch){
 clearTimeout(timers[ch]);timers[ch]=null;
 if(pending[ch]!=null){const v=pending[ch];pending[ch]=null;
  api({action:'set',ch:ch,us:v});}
}
function update(){
 document.getElementById('arm').textContent=S.armed?'DISARM':'ARM';
 document.getElementById('arm').className=S.armed?'on':'';
 for(const s of S.servos){
  const d=document.getElementById('sv'+s.ch);if(!d)continue;
  d.className='servo'+(s.ch===S.active?' active':'');
  const c=s.cal?`zero:${s.cal.zero??'-'} min:${s.cal.min??'-'} max:${s.cal.max??'-'}`:'';
  document.getElementById('cal'+s.ch).textContent=c;
  // never touch the slider the user is holding, nor overwrite a pending send
  if(dragging!==s.ch&&pending[s.ch]==null&&s.us){
   document.getElementById('sl'+s.ch).value=s.us;
   document.getElementById('us'+s.ch).textContent=s.us+'us';}
  if(!s.us&&dragging!==s.ch)document.getElementById('us'+s.ch).textContent='off';
 }
}
async function api(body){
 const r=await fetch('/api',{method:'POST',body:JSON.stringify(body)});
 const j=await r.json();S=j.state;
 if(body.action!=='state'&&j.msg)document.getElementById('msg').textContent=j.msg;
 if(!built)build();update();}
function nudge(ch,d){
 const sl=document.getElementById('sl'+ch);
 sl.value=+sl.value+d;
 document.getElementById('us'+ch).textContent=sl.value+'us';
 api({action:'set',ch:ch,us:+sl.value});}
function toggleArm(){api({action:S.armed?'disarm':'arm'})}
api({action:'state'});setInterval(()=>api({action:'state'}),4000);
</script></body></html>"""


def make_handler(board, cal):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _state(self):
            servos = []
            for ch, name in SERVOS:
                lo, hi = CH_LIMITS.get(ch, (MIN_US, MAX_US))
                servos.append({'ch': ch, 'name': name,
                               'us': round(board.last_us.get(ch, 0)) or None,
                               'min': lo, 'max': hi,
                               'cal': cal.data.get(str(ch))})
            return {'armed': board.armed, 'active': board.active, 'servos': servos}

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
            act, ch = req.get('action'), req.get('ch')
            msg = ''
            if act == 'arm':
                board.armed = True
                msg = 'ARMED. The selected servo will follow its slider.'
            elif act == 'disarm':
                board.armed = False
                msg = 'DISARMED (outputs keep their last pulse; use release/ALL OFF to cut).'
            elif act == 'all_off':
                board.all_off()
                msg = 'ALL OFF: every channel released, disarmed.'
            elif act == 'select':
                board.active = ch
                msg = f'ch{ch} is now the active servo.'
            elif act == 'set':
                msg = f'ch{ch}: {board.command(ch, req.get("us", CENTER_US))}'
            elif act == 'nudge':
                base = board.target_us.get(ch, board.last_us.get(ch, CENTER_US))
                msg = f'ch{ch}: {board.command(ch, base + req.get("delta", 0))}'
            elif act == 'release':
                board.release(ch)
                msg = f'ch{ch} released (signal cut).'
            elif act == 'mark':
                us = board.last_us.get(ch)
                if us is None:
                    msg = 'refused: servo has no commanded pulse yet.'
                else:
                    name = dict(SERVOS)[ch]
                    cal.mark(ch, name, req.get('kind'), us)
                    path = cal.save()
                    msg = f'ch{ch} {req.get("kind")} = {us:.0f}us saved to {path}'
            self._send(200, json.dumps({'state': self._state(), 'msg': msg}))
    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bus', type=int, default=None, help='I2C bus number')
    ap.add_argument('--port', type=int, default=8080)
    args = ap.parse_args()

    bus_num = find_bus(args.bus)
    board = Board(bus_num)
    cal = Cal()
    print(f'PCA9685 found on /dev/i2c-{bus_num}, 50 Hz set, ALL OFF, DISARMED.')
    print(f'Open http://<this-host>:{args.port}  (calibration -> {CAL_FILE})')
    server = ThreadingHTTPServer(('0.0.0.0', args.port), make_handler(board, cal))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        board.all_off()
        print('\nALL OFF sent. Bye.')


if __name__ == '__main__':
    main()
