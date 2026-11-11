#!/usr/bin/env python3
"""SOMA's voice, phase 1 (docs/voice_plan.md): the eye board's microphone and
speaker, a Gemini Live session in between. Spanish first.

    voice_relay.py                       talk; Ctrl-C ends
    voice_relay.py --model gemini-3.8-live
    voice_relay.py --vol 85
    voice_relay.py --voice Kore          one of Gemini's prebuilt voices

The board streams 20 ms microphone frames and plays what this sends
(firmware/eyes/eyes/frames.h). The key comes from the macOS Keychain
(service soma-voice-gemini), never from a file. Every turn is logged with
its latency and tokens to firmware/eyes/rec/voice_log.csv (ignored by git:
it holds transcripts of the room).

Phase 1 is half duplex: while SOMA speaks, the microphone sends silence, so
the speaker does not talk to itself (barge-in is phase 2). The relay
exposes no tool: the voice never reaches the arms.
"""
import argparse
import asyncio
import signal
import csv
import glob
import math
import os
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

RATE = 16000                    # the board's I2S rate, both directions
GEMINI_OUT_RATE = 24000
MAGIC = 0xA5
MAX_PAYLOAD = 2048              # eyes::MAX_PAYLOAD
WINDOW = 4                      # speaker frames in flight before an ack
KEEPALIVE_S = 1.0               # the board leaves the stream after 3 s of quiet
VOICE_DBFS = -40.0              # speech frames: voice RMS about -30, quiet room median -59
VOICE_RUN = 3                   # ... at least this many in a row (60 ms): single clicks reach -29
TAIL_S = 0.35                   # after the speaker empties, still half duplex this long
DEFAULT_MODEL = 'gemini-3.1-flash-live-preview'
# Without a voice the model picks one per session, so SOMA changed between
# a man's and a woman's voice (2026-11-10). One fixed voice is part of who it is.
DEFAULT_VOICE = 'Capella'      # chosen by Andres, 2026-11-10
LANGUAGE = 'es-MX'             # Mexican Spanish: the speech config and the prompt both say it
LOG = Path(__file__).resolve().parents[1] / 'rec' / 'voice_log.csv'
SYSTEM = ('Eres SOMA, un robot con dos brazos de aluminio colgados de una caja central, '
          'unos ojos redondos en dos pantallas y una cámara. Hablas español de México, con acento '
          'mexicano, con frases cortas '
          'y naturales, como en una conversación. No puedes mover los brazos por voz todavía: '
          'si te lo piden, dilo con honestidad.')


# ---- pure parts, tested in soma_driver/test/test_voice_relay.py ----

def frame(kind: str, payload: bytes = b'') -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('payload too long')
    return bytes([MAGIC, ord(kind), len(payload) & 0xFF, len(payload) >> 8]) + payload


class BoardFrames:
    """Board to host: 'M' microphone, 'K' acks. Text between frames (the
    board's replies and logs) is collected as lines, never mistaken for audio."""

    TYPES = {ord('M'), ord('K')}

    def __init__(self):
        self.buf = bytearray()
        self.text = bytearray()

    def feed(self, data: bytes) -> tuple[list[tuple[str, bytes]], list[str]]:
        self.buf += data
        frames, lines = [], []
        while self.buf:
            if self.buf[0] != MAGIC:
                cut = self.buf.find(MAGIC)
                cut = len(self.buf) if cut < 0 else cut
                self.text += self.buf[:cut]
                del self.buf[:cut]
                continue
            if len(self.buf) < 4:
                break
            kind, size = self.buf[1], self.buf[2] | self.buf[3] << 8
            if kind not in self.TYPES or size > MAX_PAYLOAD:
                del self.buf[:1]                     # not a frame: hunt for the next magic
                continue
            if len(self.buf) < 4 + size:
                break
            frames.append((chr(kind), bytes(self.buf[4:4 + size])))
            del self.buf[:4 + size]
        while b'\n' in self.text:
            line, _, rest = self.text.partition(b'\n')
            self.text = bytearray(rest)
            # Joining a stream mid frame leaves audio bytes here: real board
            # text is printable ASCII, anything else is dropped.
            text = line.strip()
            if text and all(32 <= c < 127 for c in text):
                lines.append(text.decode())
        return frames, lines


class Down24to16:
    """24 kHz to 16 kHz, streaming: up by 2, a windowed-sinc low pass at
    7.2 kHz, down by 3. Chunk boundaries leave no seam."""

    TAPS = 121

    def __init__(self):
        n = np.arange(self.TAPS) - (self.TAPS - 1) / 2
        fc = 7200 / 48000
        h = 2 * fc * np.sinc(2 * fc * n) * np.blackman(self.TAPS)
        self.h = h / h.sum() * 2                     # gain 2 restores the zero-stuffed level
        self.hist = np.zeros(self.TAPS - 1)
        self.consumed = 0                           # 48 kHz samples seen so far
        self.next_out = 0                           # next 48 kHz index to emit (a multiple of 3)

    def __call__(self, pcm24: np.ndarray) -> np.ndarray:
        up = np.zeros(2 * len(pcm24))
        up[::2] = pcm24
        buf = np.concatenate([self.hist, up])
        base = self.consumed - (self.TAPS - 1)      # 48 kHz index of buf[0]
        y = np.convolve(buf, self.h, mode='valid')  # y[i] is the output at index base + TAPS - 1 + i
        first = self.next_out - (base + self.TAPS - 1)
        out = y[first::3] if first < len(y) else np.zeros(0)
        self.next_out += 3 * len(out)
        self.consumed += len(up)
        self.hist = buf[-(self.TAPS - 1):]
        return out


class VoiceTracker:
    """When the person last spoke: the end of the last run of VOICE_RUN or
    more frames above VOICE_DBFS. A lone click is not a word (2026-11-10:
    single frames up to -29 dBFS in a quiet room made the latency read 0.1 s)."""

    def __init__(self):
        self.run = 0
        self.last = 0.0

    def frame(self, level_dbfs: float, now: float) -> None:
        self.run = self.run + 1 if level_dbfs > VOICE_DBFS else 0
        if self.run >= VOICE_RUN:
            self.last = now


def drain(ser, quiet_s: float = 0.15, limit_s: float = 1.5) -> int:
    """Read until the board has been quiet for quiet_s: a stream that ended
    with the port closed leaves frames in the buffers, delivered on the next
    open (2026-11-10: 7 KB of them)."""
    old, ser.timeout = ser.timeout, quiet_s
    n, end = 0, time.monotonic() + limit_s
    while time.monotonic() < end:
        got = ser.read(65536)
        if not got:
            break
        n += len(got)
    ser.timeout = old
    return n


def to_int16(x: np.ndarray) -> bytes:
    return np.clip(np.round(x), -32768, 32767).astype('<i2').tobytes()


def dbfs(pcm: bytes) -> float:
    x = np.frombuffer(pcm, dtype='<i2').astype(np.float64)
    rms = math.sqrt(float(np.mean(x * x))) if x.size else 0.0
    return 20 * math.log10(rms / 32768) if rms > 0 else -120.0


class SpeakerQueue:
    """What the board still has to play, estimated between acks: the last
    ack's count, drained at the I2S rate. Speaking = something left or the
    tail after it, when the microphone must send silence."""

    def __init__(self):
        self.queued = 0
        self.capacity = 1 << 19
        self.at = 0.0
        self.inflight = 0

    def ack(self, queued: int, capacity: int, now: float) -> None:
        self.queued, self.capacity, self.at = queued, capacity, now
        self.inflight = max(0, self.inflight - 1)

    def left(self, now: float) -> float:
        return max(0.0, self.queued - (now - self.at) * RATE * 2)

    def speaking(self, now: float) -> bool:
        return self.inflight > 0 or self.left(now) > 0 or now - self.empty_at(now) < TAIL_S

    def empty_at(self, now: float) -> float:
        return self.at + self.queued / (RATE * 2) if self.queued else self.at

    def may_send(self, size: int, now: float) -> bool:
        return self.inflight < WINDOW and self.left(now) + (self.inflight + 1) * MAX_PAYLOAD + size < self.capacity


# ---- the session ----

def keychain_key() -> str:
    r = subprocess.run(['security', 'find-generic-password', '-a', os.environ.get('USER', ''),
                        '-s', 'soma-voice-gemini', '-w'], capture_output=True, text=True)
    if r.returncode or not r.stdout.strip():
        raise SystemExit('no key in the Keychain: security add-generic-password -a "$USER" -s soma-voice-gemini -w')
    return r.stdout.strip()


class Relay:
    def __init__(self, port: str, model: str, voice: str = DEFAULT_VOICE):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.model = model
        self.voice_name = voice
        self.frames = BoardFrames()
        self.speaker = SpeakerQueue()
        self.resample = Down24to16()
        self.pending = bytearray()                  # 16 kHz speaker audio not yet sent
        self.write_lock = threading.Lock()
        self.voice = VoiceTracker()
        self.turn = {}
        self.tokens = 0

    def send(self, kind: str, payload: bytes = b'') -> None:
        with self.write_lock:
            self.ser.write(frame(kind, payload))

    def command(self, line: str) -> None:
        self.send('C', line.encode())

    def reader(self, loop, mic: asyncio.Queue, stop: threading.Event) -> None:
        while not stop.is_set():
            data = self.ser.read(4096)
            if not data:
                continue
            frames, lines = self.frames.feed(data)
            now = time.monotonic()
            for kind, payload in frames:
                if kind == 'M':
                    loop.call_soon_threadsafe(mic.put_nowait, (now, payload))
                elif kind == 'K' and len(payload) == 8:
                    self.speaker.ack(*struct.unpack('<II', payload), now)
            for line in lines:
                if line != 'ok':
                    print(f'  [board] {line}')

    async def run(self, vol: int | None) -> None:
        from google import genai
        from google.genai import types

        with self.write_lock:
            drain(self.ser)
            self.ser.write(b'\nstream\n')
        if vol is not None:
            self.command(f'vol {vol}')
        self.command('pupil 1.25')                  # listening shows: wide pupils
        client = genai.Client(api_key=keychain_key())
        config = types.LiveConnectConfig(
            response_modalities=['AUDIO'], system_instruction=SYSTEM,
            input_audio_transcription={}, output_audio_transcription={},
            speech_config=types.SpeechConfig(language_code=LANGUAGE, voice_config=types.VoiceConfig(
                prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name=self.voice_name))))
        loop = asyncio.get_running_loop()
        mic: asyncio.Queue = asyncio.Queue()
        stop = threading.Event()
        threading.Thread(target=self.reader, args=(loop, mic, stop), daemon=True).start()
        print(f'{self.model}, voice {self.voice_name}: connecting...', flush=True)
        try:
            async with client.aio.live.connect(model=self.model, config=config) as session:
                print('listening. Talk to SOMA (Ctrl-C ends).', flush=True)
                await asyncio.gather(self.uplink(session, mic, types), self.downlink(session),
                                     self.speaker_feed(), self.keepalive())
        finally:
            stop.set()
            self.command('pupil 1')
            self.command('stream off')
            time.sleep(0.2)
            self.ser.close()
            print(f'session tokens: {self.tokens}')

    async def uplink(self, session, mic: asyncio.Queue, types) -> None:
        while True:
            now, pcm = await mic.get()
            if self.speaker.speaking(now):
                pcm = bytes(len(pcm))               # half duplex: SOMA does not hear itself
            else:
                self.voice.frame(dbfs(pcm), now)
            await session.send_realtime_input(audio=types.Blob(data=pcm, mime_type=f'audio/pcm;rate={RATE}'))

    async def downlink(self, session) -> None:
        while True:
            async for msg in session.receive():
                if msg.usage_metadata and msg.usage_metadata.total_token_count:
                    self.turn['tokens'] = msg.usage_metadata.total_token_count
                sc = msg.server_content
                if not sc:
                    continue
                if sc.input_transcription and sc.input_transcription.text:
                    self.turn['heard'] = self.turn.get('heard', '') + sc.input_transcription.text
                if sc.output_transcription and sc.output_transcription.text:
                    self.turn['said'] = self.turn.get('said', '') + sc.output_transcription.text
                for part in (sc.model_turn.parts if sc.model_turn else []):
                    if part.inline_data and part.inline_data.data:
                        if 'first' not in self.turn:
                            self.turn['first'] = time.monotonic()
                        pcm24 = np.frombuffer(part.inline_data.data, dtype='<i2').astype(np.float64)
                        self.pending += to_int16(self.resample(pcm24))
                if sc.interrupted:
                    self.pending.clear()
                    self.send('X')
                if sc.turn_complete:
                    self.log_turn()

    async def speaker_feed(self) -> None:
        while True:
            now = time.monotonic()
            if self.pending and self.speaker.may_send(MAX_PAYLOAD, now):
                chunk = bytes(self.pending[:MAX_PAYLOAD])
                del self.pending[:MAX_PAYLOAD]
                self.speaker.inflight += 1
                self.send('P', chunk)
                continue
            await asyncio.sleep(0.005)

    async def keepalive(self) -> None:
        while True:
            await asyncio.sleep(KEEPALIVE_S)
            self.send('N')

    def log_turn(self) -> None:
        t = self.turn
        self.turn = {}
        latency = t['first'] - self.voice.last if 'first' in t and self.voice.last else float('nan')
        self.tokens += t.get('tokens', 0)
        print(f'  you:  {t.get("heard", "").strip()}')
        print(f'  SOMA: {t.get("said", "").strip()}')
        print(f'  latency {latency:.2f} s (your last word to Gemini\'s first audio), '
              f'tokens {t.get("tokens", 0)}, session {self.tokens}', flush=True)
        LOG.parent.mkdir(exist_ok=True)
        new = not LOG.exists()
        with LOG.open('a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(['time', 'model', 'latency_s', 'tokens', 'heard', 'said'])
            w.writerow([time.strftime('%Y-%m-%d %H:%M:%S'), self.model, f'{latency:.2f}', t.get('tokens', 0),
                        t.get('heard', '').strip(), t.get('said', '').strip()])


def _terminate(*_):
    raise KeyboardInterrupt


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port')
    ap.add_argument('--model', default=DEFAULT_MODEL)
    ap.add_argument('--vol', type=int, help='speaker volume 0..100 for this session')
    ap.add_argument('--voice', default=DEFAULT_VOICE, help=f'prebuilt voice (default {DEFAULT_VOICE})')
    args = ap.parse_args(argv)
    port = args.port or next(iter(sorted(glob.glob('/dev/cu.usbmodem*'))), None)
    if not port:
        raise SystemExit('no board found on /dev/cu.usbmodem*')
    relay = Relay(port, args.model, args.voice)
    # SIGTERM ends like Ctrl-C, so the board hears `stream off` (a relay in
    # the background ignores SIGINT).
    signal.signal(signal.SIGTERM, _terminate)
    try:
        asyncio.run(relay.run(args.vol))
    except KeyboardInterrupt:
        print('\nbye')
    return 0


if __name__ == '__main__':
    sys.exit(main())
