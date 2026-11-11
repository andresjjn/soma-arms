#!/usr/bin/env python3
"""Phase 0 of SOMA's voice (docs/voice_plan.md): the eye board's microphones
and speaker over USB, measured. No network, no API.

    audio_check.py rec 3            record 3 s, save a WAV, print levels per channel
    audio_check.py rec 3 --listen   ... and play it on the Mac
    audio_check.py tone 1000 2 -12  a 1 kHz sine, 2 s, -12 dBFS, out the board's speaker
    audio_check.py play voice.wav   any 16 bit WAV, resampled to the board's rate
    audio_check.py loop 3           record 3 s, then play the louder channel back on the board
    audio_check.py vol 70           speaker volume, 0..100
    audio_check.py selftest         does the speaker sound? the board's mics listen to it

Recordings go to firmware/eyes/rec/ (ignored by git: they are room audio).
Levels are dBFS: 0 is a full-scale 16 bit sample. The noise floor is the
quietest 100 ms window, so record with a pause in it.
"""
import argparse
import glob
import math
import subprocess
import sys
import time
import wave
from pathlib import Path

import numpy as np
import serial

RATE = 16000                     # eyes::AUDIO_RATE
BLOCK = 4096                     # eyes::AUDIO_BLOCK
MAX_PLAY_S = 10.0
REC_DIR = Path(__file__).resolve().parents[1] / 'rec'


def dbfs(x: np.ndarray) -> float:
    rms = math.sqrt(float(np.mean(np.square(x.astype(np.float64))))) if x.size else 0.0
    return 20 * math.log10(rms / 32768) if rms > 0 else -120.0


def levels(stereo: np.ndarray, rate: int) -> list[dict]:
    """Per channel: RMS, peak, noise floor (quietest 100 ms) and clipped samples."""
    out = []
    win = rate // 10
    for ch in range(stereo.shape[1]):
        x = stereo[:, ch]
        windows = [dbfs(x[i:i + win]) for i in range(0, len(x) - win + 1, win)] or [dbfs(x)]
        peak = int(np.max(np.abs(x.astype(np.int32)))) if x.size else 0
        out.append({'rms': dbfs(x), 'peak': 20 * math.log10(peak / 32768) if peak else -120.0,
                    'floor': min(windows), 'clipped': int(np.sum(np.abs(x.astype(np.int32)) >= 32767))})
    return out


def to_board_rate(samples: np.ndarray, rate: int) -> np.ndarray:
    """Mono int16 at the board's rate (linear interpolation: fine for speech checks)."""
    if rate == RATE:
        return samples.astype(np.int16)
    t_out = np.arange(int(len(samples) * RATE / rate)) / RATE
    t_in = np.arange(len(samples)) / rate
    return np.interp(t_out, t_in, samples.astype(np.float64)).round().clip(-32768, 32767).astype(np.int16)


class Board:
    def __init__(self, port: str):
        self.s = serial.Serial(port, 115200, timeout=6)
        time.sleep(0.2)
        self.s.reset_input_buffer()

    def line(self) -> str:
        raw = self.s.readline()
        if not raw:
            raise SystemExit('the board did not answer (timeout)')
        return raw.decode(errors='replace').strip()

    def command(self, text: str) -> str:
        self.s.write((text + '\n').encode())
        return self.line()

    def rec(self, seconds: float) -> np.ndarray:
        self.s.timeout = seconds + 6
        self.s.write(f'rec {seconds:g}\n'.encode())
        head = self.line()
        if not head.startswith('pcm '):
            raise SystemExit(f'rec: {head}')
        n, rate, ch = (int(v) for v in head.split()[1:4])
        data = self.s.read(n)
        tail = self.line()
        if len(data) != n or tail != 'ok':
            raise SystemExit(f'rec: got {len(data)} of {n} bytes, then {tail!r}')
        if rate != RATE:
            raise SystemExit(f'the board records at {rate} Hz, this tool expects {RATE}')
        return np.frombuffer(data, dtype='<i2').reshape(-1, ch)

    def selftest(self) -> np.ndarray:
        self.s.timeout = 8
        self.s.write(b'selftest\n')
        head = self.line()
        if not head.startswith('pcm '):
            raise SystemExit(f'selftest: {head}')
        n, _, ch = (int(v) for v in head.split()[1:4])
        data = self.s.read(n)
        if len(data) != n or self.line() != 'ok':
            raise SystemExit('selftest: short transfer')
        return np.frombuffer(data, dtype='<i2').reshape(-1, ch)

    def play(self, mono: np.ndarray) -> None:
        payload = mono.astype('<i2').tobytes()
        if len(payload) > int(RATE * MAX_PLAY_S) * 2:
            raise SystemExit(f'at most {MAX_PLAY_S:g} s per play')
        self.s.timeout = 6
        self.s.write(f'play {len(payload)}\n'.encode())
        for i in range(0, len(payload), BLOCK):
            ack = self.s.read(1)
            if ack != b'k':
                rest = ack + self.s.readline()
                raise SystemExit(f'play: {rest.decode(errors="replace").strip() or "no ack (timeout)"}')
            self.s.write(payload[i:i + BLOCK])
        self.s.timeout = len(mono) / RATE + 6
        tail = self.line()
        if tail != 'ok':
            raise SystemExit(f'play: {tail}')


def tone_db(x: np.ndarray, freq: float = 1000.0) -> float:
    """Level of one frequency in dBFS (a single DFT bin, Hann window)."""
    if x.size == 0:
        return -120.0
    w = np.hanning(len(x))
    t = np.arange(len(x)) / RATE
    amp = 2 * abs(np.sum(x * w * np.exp(-2j * np.pi * freq * t))) / np.sum(w)
    return 20 * math.log10(amp / 32768) if amp > 0 else -120.0


def save(stereo: np.ndarray) -> Path:
    REC_DIR.mkdir(exist_ok=True)
    path = REC_DIR / time.strftime('%Y-%m-%d_%H%M%S.wav')
    with wave.open(str(path), 'wb') as w:
        w.setnchannels(stereo.shape[1])
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(stereo.astype('<i2').tobytes())
    return path


def report(stereo: np.ndarray) -> int:
    lv = levels(stereo, RATE)
    for ch, d in enumerate(lv):
        print(f'  channel {ch}: rms {d["rms"]:6.1f} dBFS, peak {d["peak"]:6.1f}, '
              f'floor {d["floor"]:6.1f}, clipped {d["clipped"]}')
    return int(np.argmax([d['rms'] for d in lv]))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port')
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('rec'); r.add_argument('seconds', type=float); r.add_argument('--listen', action='store_true')
    t = sub.add_parser('tone'); t.add_argument('freq', type=float, nargs='?', default=1000.0)
    t.add_argument('seconds', type=float, nargs='?', default=2.0); t.add_argument('level', type=float, nargs='?', default=-12.0)
    p = sub.add_parser('play'); p.add_argument('wav')
    lo = sub.add_parser('loop'); lo.add_argument('seconds', type=float)
    v = sub.add_parser('vol'); v.add_argument('percent', type=int)
    sub.add_parser('selftest')
    args = ap.parse_args(argv)

    port = args.port or next(iter(sorted(glob.glob('/dev/cu.usbmodem*'))), None)
    if not port:
        raise SystemExit('no board found on /dev/cu.usbmodem*')
    board = Board(port)

    if args.cmd == 'selftest':
        x = board.selftest()[:, 0].astype(np.float64)
        half = RATE // 2
        quiet, loud = tone_db(x[half // 5:half]), tone_db(x[half + RATE // 5:])
        print(f'1 kHz at the microphone: quiet {quiet:6.1f} dBFS, tone playing {loud:6.1f} dBFS, '
              f'difference {loud - quiet:+.1f} dB')
        print('the speaker sounds' if loud - quiet > 20 else
              'no tone at the microphone: no speaker, or it does not sound')
    elif args.cmd == 'vol':
        print(board.command(f'vol {args.percent}'))
    elif args.cmd in ('rec', 'loop'):
        print(f'recording {args.seconds:g} s: talk now, leave a pause', flush=True)
        stereo = board.rec(args.seconds)
        path = save(stereo)
        print(f'saved {path.relative_to(REC_DIR.parents[2])}')
        loud = report(stereo)
        if args.cmd == 'rec' and args.listen:
            subprocess.run(['afplay', str(path)], check=False)
        if args.cmd == 'loop':
            print(f'playing channel {loud} back on the board')
            board.play(stereo[:, loud])
    elif args.cmd == 'tone':
        t_s = np.arange(int(args.seconds * RATE)) / RATE
        amp = 32767 * 10 ** (args.level / 20)
        ramp = np.minimum(1.0, np.minimum(t_s, t_s[::-1]) / 0.02)        # 20 ms fades, no clicks
        board.play((amp * ramp * np.sin(2 * np.pi * args.freq * t_s)).round().astype(np.int16))
        print('ok')
    elif args.cmd == 'play':
        with wave.open(args.wav, 'rb') as w:
            if w.getsampwidth() != 2:
                raise SystemExit('16 bit WAV only')
            x = np.frombuffer(w.readframes(w.getnframes()), dtype='<i2').reshape(-1, w.getnchannels())
            rate = w.getframerate()
        board.play(to_board_rate(x.mean(axis=1), rate))
        print('ok')
    return 0


if __name__ == '__main__':
    sys.exit(main())
