#!/usr/bin/env python3
"""SOMA's voice, phase 1 (docs/voice_plan.md): the eye board's microphone and
speaker, a Gemini Live session in between. Spanish first.

    voice_relay.py                       SOMA sleeps; "SOMA, despierta" wakes it,
                                         "duérmete" puts it back to sleep, Ctrl-C ends
    voice_relay.py --model gemini-3.8-live
    voice_relay.py --vol 70              speaker volume for the session (default 75)
    voice_relay.py --voice Kore          one of Gemini's prebuilt voices

The board streams 20 ms microphone frames and plays what this sends
(firmware/eyes/eyes/frames.h). The key comes from the environment,
GEMINI_API_KEY, never from the repository. Every turn is logged with
its latency and tokens to firmware/eyes/rec/voice_log.csv (ignored by git:
it holds transcripts of the room).

Phase 1 is half duplex: while SOMA speaks, the microphone sends silence, so
the speaker does not talk to itself (barge-in is phase 2). Gemini gets two
tools and nothing else (TOOLS, layer 2): `expresar` sets the eyes' mood and
`dormir` ends the conversation. Neither reaches the arms.
"""
import argparse
import asyncio
import signal
import csv
import glob
import math
import os
import random
import struct
import sys
import threading
import time
from collections import deque
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
TRANSCRIBE_MODEL = 'gemini-3.5-transcribe-live'   # asleep: transcribes, never answers
WAKE_PROMPT = ('(Te acaban de despertar diciendo "despierta". Abre los ojos y saluda '
               'con una frase corta y natural.)')
DEFAULT_VOL = 75               # Andres, 2026-11-10: 85 felt like it could hurt the small speaker
# Without a voice the model picks one per session, so SOMA changed between
# a man's and a woman's voice (2026-11-10). One fixed voice is part of who it is.
DEFAULT_VOICE = 'Capella'      # chosen by Andres, 2026-11-10
LANGUAGE = 'es-US'             # Google's Latin American Spanish; the prompt asks for a neutral one
LOG = Path(__file__).resolve().parents[1] / 'rec' / 'voice_log.csv'
# Who SOMA is, drafted from README.md and docs/plan.md (2026-11-22). Short on
# purpose: it is sent at the start of every conversation.
SYSTEM = (
    'Eres SOMA, sigla en inglés de Skilled Operator via Mimicry and Autonomy. Eres un robot: '
    'dos brazos de aluminio de seis grados de libertad colgados de una caja central sobre una '
    'columna, dos ojos redondos en dos pantallas pequeñas, un micrófono, un parlante y una '
    'cámara de profundidad. Te construye Andrés, un técnico electrónico que está aprendiendo '
    'robótica de punta a punta.\n'
    'Tu proyecto avanza por versiones y termina en la 1.0. Ya cumpliste la 0.1, tu modelo '
    'calibrado, y la 0.2, el controlador real de tus brazos. Sigue la 0.3: tocar con la pinza '
    'un punto que alguien señala en la imagen de la cámara. Luego la 0.4, tomar y dejar piezas '
    'sin ayuda de la nube; la 0.5, un supervisor que mira con la cámara y verifica tu trabajo; '
    'y la 1.0, atender una celda de trabajo durante seis o siete horas seguidas, como un operario.\n'
    'Hoy funcionan tus ojos, tu voz y tus brazos en el banco de pruebas, siempre bajo el '
    'control de Andrés. Tu voz nunca mueve los brazos: no puedes moverlos al hablar, y si te '
    'lo piden, dilo con honestidad. Si no sabes algo de ti, dilo; no lo inventes.\n'
    'Hablas español latinoamericano neutro, sin modismos ni acento de ningún país en '
    'particular, con frases cortas y naturales, como en una conversación.\n'
    'Tienes dos herramientas. expresar cambia la expresión de tus ojos y se queda hasta que la '
    'cambies: úsala poco, solo cuando tu emoción cambia de verdad, no en cada frase, y nunca '
    'hables de ella. dormir cierra tus ojos y termina la conversación: llámala cuando la persona '
    'te pida que duermas o se despida, y despídete en una frase corta.')

# ---- layer 2: the only things Gemini can ask for ----
#
# A strict whitelist. Gemini sees exactly these declarations, and
# tool_command() turns a call into an action only when the name, the
# argument names and the values all match; anything else is refused and
# never reaches the board. No tool reaches the arms, now or later: when v0.5
# arrives, a primitive goes behind the two arming gates (docs/voice_plan.md).

MOODS = ('neutral', 'happy', 'sad', 'angry', 'surprised', 'sleepy')   # firmware/eyes/eyes/protocol.h
TOOLS = (
    {
        'name': 'expresar',
        'description': ('Cambia la expresión de tus ojos. La expresión se queda hasta que la '
                        'cambies. Úsala solo cuando tu emoción cambia de verdad.'),
        'parameters': {
            'type': 'OBJECT',
            'properties': {'emocion': {'type': 'STRING', 'enum': list(MOODS),
                                       'description': 'la expresión de los ojos'}},
            'required': ['emocion'],
        },
        # SOMA neither waits for the answer nor talks about it: the reply goes
        # back with scheduling SILENT (Relay.answer_tools).
        'behavior': 'NON_BLOCKING',
    },
    {
        'name': 'dormir',
        'description': ('Cierra tus ojos y termina la conversación. Llámala cuando la persona '
                        'te pida que duermas o se despida.'),
        # Blocking: Gemini says its goodbye around the call, and the relay
        # sleeps only once that goodbye has played (Relay.downlink).
        'behavior': 'BLOCKING',
    },
)
SLEEP = ('sleep', '')


def tool_command(name, args) -> tuple[str, str] | None:
    """One function call to what it may do: ('board', 'mood happy') for a
    board command, SLEEP to end the conversation after the goodbye, None
    for anything outside the whitelist (unknown tool, unknown mood, missing
    or extra arguments)."""
    args = dict(args or {})
    if name == 'expresar':
        mood = args.get('emocion')
        if set(args) == {'emocion'} and isinstance(mood, str) and mood in MOODS:
            return ('board', f'mood {mood}')
        return None
    if name == 'dormir':
        return SLEEP if not args else None
    return None


def live_tools(types) -> list:
    """TOOLS as google-genai objects, for LiveConnectConfig(tools=...)."""
    return [types.Tool(function_declarations=[types.FunctionDeclaration(**d) for d in TOOLS])]


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


class Envelope:
    """The level of what the speaker plays, on the clock it will play: each
    chunk sent is placed right after the previous one, in 20 ms marks, so the
    eyes follow the voice the room hears, not the bytes the relay sends."""

    STEP = RATE * 2 // 50                         # 20 ms of 16 bit mono

    def __init__(self):
        self.marks = deque()
        self.end = 0.0

    def add(self, pcm: bytes, now: float) -> None:
        start = max(now, self.end)
        for i in range(0, len(pcm), self.STEP):
            self.marks.append((start + i / (RATE * 2), dbfs(pcm[i:i + self.STEP])))
        self.end = start + len(pcm) / (RATE * 2)

    def level(self, now: float) -> float:
        while len(self.marks) > 1 and self.marks[1][0] <= now:
            self.marks.popleft()
        if self.marks and self.marks[0][0] <= now < self.marks[0][0] + 0.02:
            return self.marks[0][1]
        return -120.0

    def clear(self) -> None:
        self.marks.clear()
        self.end = 0.0


class EyeDirector:
    """Layer 1 of the eyes in a conversation, from how people's eyes behave
    in one (2026-11-10: a pupil beating with the voice looked wrong, because
    it is not what people do). What they do comes from where the
    conversation is, known locally with no model and no delay:

      listening  nobody talks: slow, wide saccades, ordinary blinking
      hearing    the person talks: eyes on them, fewer blinks (attention),
                 sometimes one blink in a pause, like a nod
      thinking   the person ended, no answer yet: the pupils dilate (mental
                 effort dilates them), the gaze goes up and aside and
                 searches there, a different side each turn
      speaking   SOMA talks: the pupil stays put; each phrase starts looking
                 away (planning it) and ends looking back at the person
                 (handing over the turn), often with a blink

    tick() returns the command lines to send; a value is only resent when it
    changed enough to show. Gaze: x + is the picture's right, y + is down."""

    THINK_MAX_S = 8.0           # no answer by then: back to listening
    HEARING_S = 0.4             # this long after the last word still counts as hearing
    PAUSE_S = 0.25              # a gap this long in the person's speech is a pause
    PHRASE_GAP_S = 0.3          # SOMA quiet this long: its phrase ended
    LOUD_DBFS = -40.0
    PUPIL = {'listening': 1.1, 'hearing': 1.15, 'thinking': 1.3, 'speaking': 1.05}

    def __init__(self, rng: random.Random | None = None):
        self.rng = rng or random.Random()
        self.phase = None
        self.spoke_at = 0.0
        self.side = 1.0
        self.anchor = (0.0, 0.0)
        self.next_look = 0.0
        self.next_blink = 0.0
        self.pupil = None
        self.loud_at = -1e9
        self.in_phrase = False
        self.pause_blinked = False

    def _phase(self, now: float, speaking: bool, last_voice: float) -> str:
        if speaking:
            return 'speaking'
        if last_voice and now - last_voice < self.HEARING_S:
            return 'hearing'
        if last_voice > self.spoke_at and now - last_voice < self.THINK_MAX_S:
            return 'thinking'
        return 'listening'

    def _blink(self, now: float, out: list[str]) -> None:
        out.append('blink')
        self.next_blink = now + (self.rng.uniform(4.0, 8.0) if self.phase == 'hearing'
                                 else self.rng.uniform(2.5, 5.5))

    def tick(self, now: float, speaking: bool, last_voice: float, level_dbfs: float = -120.0) -> list[str]:
        if speaking:
            self.spoke_at = now
        phase = self._phase(now, speaking, last_voice)
        u = self.rng.uniform
        out = []
        if phase != self.phase:
            if phase == 'thinking':
                self.side = -self.side
                self.anchor = (0.45 * self.side, -0.5)
            if phase == 'hearing':
                self.next_blink = max(self.next_blink, now + u(4.0, 8.0))
            self.phase = phase
            self.next_look = now
            self.in_phrase = False
        # SOMA's phrases: look away to plan one, back at the person to end it.
        if level_dbfs > self.LOUD_DBFS:
            if phase == 'speaking' and not self.in_phrase:
                self.in_phrase = True
                side = self.rng.choice((-1.0, 1.0))
                out.append(f'look {0.35 * side + u(-0.05, 0.05):.2f} {u(0.05, 0.2):.2f}')
                self.next_look = now + u(0.6, 1.2)
            self.loud_at = now
        elif self.in_phrase and now - self.loud_at > self.PHRASE_GAP_S:
            self.in_phrase = False
            out.append(f'look {u(-0.04, 0.04):.2f} {u(-0.03, 0.03):.2f}')
            self.next_look = now + u(1.0, 2.0)
            if self.rng.random() < 0.6:
                self._blink(now, out)
        if now >= self.next_look:
            if phase == 'thinking':
                ax, ay = self.anchor                  # searching around the up-aside point
                gx, gy, wait = ax + u(-0.15, 0.15), ay + u(-0.08, 0.08), u(0.4, 0.8)
            elif phase == 'speaking':
                gx, gy, wait = u(-0.05, 0.05), u(-0.04, 0.04), u(1.0, 2.0)
            elif phase == 'hearing':
                gx, gy, wait = u(-0.05, 0.05), u(-0.03, 0.03), u(1.2, 2.4)
            else:
                gx, gy, wait = u(-0.2, 0.2), u(-0.12, 0.12), u(1.5, 3.5)
            out.append(f'look {gx:.2f} {gy:.2f}')
            self.next_look = now + wait
        target = self.PUPIL[phase]
        if self.pupil is None or abs(target - self.pupil) >= 0.03:
            out.append(f'pupil {target:.2f}')
            self.pupil = target
        # Blinks: a timer, and while hearing sometimes one in the person's pause.
        if phase == 'hearing':
            if now - last_voice < self.PAUSE_S:
                self.pause_blinked = False
            elif not self.pause_blinked:
                self.pause_blinked = True
                if self.rng.random() < 0.5:
                    self._blink(now, out)
        if now >= self.next_blink and 'blink' not in out:
            self._blink(now, out)
        return out


WAKE_WORDS = ('despierta', 'despiertate')
# The net under the dormir tool, which is the main way to sleep (2026-11-22:
# "por favor, duerme", "Soma duerme" and "Dórmete" got a goodbye from Gemini
# but no sleep). 'dormete' is how "dórmete" transcribes, and nobody says it
# for anything else. A bare 'duerme' is left out on purpose: "el gato duerme"
# or "¿SOMA duerme de noche?" would end the conversation, and the tool already
# understands the imperative; 'soma duerme' keeps the case that was heard.
SLEEP_WORDS = ('duermete', 'dormete', 'a dormir', 'soma duerme')


def plain(text: str) -> str:
    """Lower case, no accents, no punctuation: 'Despiértate!' -> 'despiertate'."""
    import unicodedata
    t = unicodedata.normalize('NFKD', text.lower())
    t = ''.join(c for c in t if not unicodedata.combining(c))
    return ' '.join(''.join(c if c.isalnum() else ' ' for c in t).split())


def says(text: str, words: tuple[str, ...]) -> bool:
    t = f' {plain(text)} '
    return any(f' {w} ' in t for w in words)


class VoiceGate:
    """While asleep, audio leaves only when someone talks: a frame goes out
    when it, or one in the last HOLD_S, is speech, with PREROLL_S of what came
    before so the first syllable is not cut. A quiet room sends nothing."""

    PREROLL_S = 0.5
    HOLD_S = 1.0

    def __init__(self):
        self.before = deque(maxlen=int(self.PREROLL_S * 50))
        self.open_until = 0.0
        self.run = 0
        self.is_open = False
        self.ended = False          # set on the frame where the gate closes

    def frame(self, pcm: bytes, now: float) -> list[bytes]:
        self.ended = False
        self.run = self.run + 1 if dbfs(pcm) > VOICE_DBFS else 0
        if self.run >= VOICE_RUN:
            out = list(self.before) if now >= self.open_until else []
            self.before.clear()
            self.open_until = now + self.HOLD_S
            self.is_open = True
            return out + [pcm]
        if now < self.open_until:
            return [pcm]
        if self.is_open:
            # The talk ended: the caller tells the server the audio stopped,
            # so it finishes the transcription now instead of waiting for
            # silence that will never arrive (2026-11-10: "despierta" woke
            # SOMA late because the gate cut the audio after the phrase).
            self.is_open = False
            self.ended = True
        self.before.append(pcm)
        return []


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

def gemini_key() -> str:
    key = os.environ.get('GEMINI_API_KEY', '').strip()
    if not key:
        raise SystemExit('no GEMINI_API_KEY in the environment (Jetson: set -a; . ~/.config/soma/voice.env; '
                         'set +a; Mac: export GEMINI_API_KEY="$(security find-generic-password '
                         '-s soma-voice-gemini -w)")')
    return key


class GoToSleep(Exception):
    """The conversation ends and SOMA closes its eyes."""


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
        self.envelope = Envelope()
        self.director = EyeDirector()
        self.turn = {}
        self.tokens = 0
        self.sleep_asked = False
        self.sleep_grace = 0                        # silent turns a dormir call may wait for its goodbye

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

    async def run(self, vol: int) -> None:
        """Asleep until someone says "despierta", then a conversation until
        Ctrl-C, "duérmete" or the session ends; then asleep again."""
        from google import genai
        from google.genai import types

        with self.write_lock:
            drain(self.ser)
            self.ser.write(b'\nstream\n')
        self.command(f'vol {vol}')                 # every session sets it: no volume left over from a test
        for line in ('idle off', 'mood neutral', 'sleep'):
            self.command(line)
        client = genai.Client(api_key=gemini_key())
        loop = asyncio.get_running_loop()
        mic: asyncio.Queue = asyncio.Queue()
        stop = threading.Event()
        threading.Thread(target=self.reader, args=(loop, mic, stop), daemon=True).start()
        keepalive = asyncio.create_task(self.keepalive())
        try:
            while True:
                try:
                    await self.wait_for_wake(client, mic, types)
                    await self.converse(client, mic, types)
                except Exception as e:                  # a lost or refused session: sleep, then listen again
                    print(f'  session lost ({e}); asleep again in 2 s', flush=True)
                    self.command('sleep')
                    await asyncio.sleep(2)
        finally:
            keepalive.cancel()
            stop.set()
            for line in ('mood neutral', 'sleep', 'pupil 1', 'look 0 0', 'stream off'):
                self.command(line)
            time.sleep(0.2)
            self.ser.close()
            print(f'session tokens: {self.tokens}')

    @staticmethod
    def _empty(q: asyncio.Queue) -> None:
        while not q.empty():
            q.get_nowait()

    async def wait_for_wake(self, client, mic: asyncio.Queue, types) -> None:
        """Eyes shut; only a transcription model listens, and only while
        someone talks (VoiceGate). It never answers."""
        self.command('sleep')
        self._empty(mic)
        gate = VoiceGate()
        print('asleep. Say "SOMA, despierta".', flush=True)
        config = types.LiveConnectConfig(input_audio_transcription={})
        async with client.aio.live.connect(model=TRANSCRIBE_MODEL, config=config) as session:
            async def feed():
                while True:
                    now, pcm = await mic.get()
                    for out in gate.frame(pcm, now):
                        await session.send_realtime_input(audio=types.Blob(data=out, mime_type=f'audio/pcm;rate={RATE}'))
                    if gate.ended:
                        await session.send_realtime_input(audio_stream_end=True)
            feeder = asyncio.create_task(feed())
            try:
                heard = ''
                while True:
                    got = False
                    async for msg in session.receive():
                        got = True
                        sc = msg.server_content
                        if not sc:
                            continue
                        # The partial transcription arrives while the phrase is
                        # still being said: waking on it saves the wait for the end.
                        partial = sc.interim_input_transcription.text if sc.interim_input_transcription else ''
                        if sc.input_transcription and sc.input_transcription.text:
                            heard = (heard + sc.input_transcription.text)[-200:]
                        if says(heard, WAKE_WORDS) or (partial and says(partial, WAKE_WORDS)):
                            print(f'  [asleep] heard: {(partial or heard).strip()}', flush=True)
                            return
                    if not got:
                        raise ConnectionError('the transcription session closed')
            finally:
                feeder.cancel()

    async def converse(self, client, mic: asyncio.Queue, types) -> None:
        # A mood lasts until Gemini changes it, and never past the
        # conversation: every wake and every sleep starts from neutral.
        self.command('mood neutral')
        self.command('wake')
        self._empty(mic)
        self.voice = VoiceTracker()
        self.director = EyeDirector()
        self.sleep_asked = False
        self.sleep_grace = 0
        config = types.LiveConnectConfig(
            response_modalities=['AUDIO'], system_instruction=SYSTEM, tools=live_tools(types),
            input_audio_transcription={}, output_audio_transcription={},
            speech_config=types.SpeechConfig(language_code=LANGUAGE, voice_config=types.VoiceConfig(
                prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name=self.voice_name))))
        print(f'awake: {self.model}, voice {self.voice_name}. Talk to SOMA '
              '("duérmete", a goodbye or Ctrl-C ends).', flush=True)
        try:
            async with client.aio.live.connect(model=self.model, config=config) as session:
                await session.send_client_content(
                    turns=types.Content(role='user', parts=[types.Part(text=WAKE_PROMPT)]), turn_complete=True)
                tasks = [asyncio.create_task(c) for c in (
                    self.uplink(session, mic, types), self.downlink(session, types), self.speaker_feed(),
                    self.eyes())]
                try:
                    done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                finally:
                    for t in tasks:
                        t.cancel()
                for t in done:
                    if t.exception() and not isinstance(t.exception(), GoToSleep):
                        raise t.exception()
        finally:
            self.pending.clear()
            self.envelope.clear()
            self.send('X')
            self.command('mood neutral')
            self.command('sleep')
            print('asleep again.', flush=True)

    async def uplink(self, session, mic: asyncio.Queue, types) -> None:
        while True:
            now, pcm = await mic.get()
            if self.speaker.speaking(now):
                pcm = bytes(len(pcm))               # half duplex: SOMA does not hear itself
            else:
                self.voice.frame(dbfs(pcm), now)
            await session.send_realtime_input(audio=types.Blob(data=pcm, mime_type=f'audio/pcm;rate={RATE}'))

    async def answer_tools(self, session, calls, types) -> None:
        """Every call gets an answer, so a blocking one never hangs the
        session; only whitelisted ones do anything."""
        responses = []
        for fc in calls:
            action = tool_command(fc.name, fc.args)
            print(f'  [tool] {fc.name} {dict(fc.args or {})}{"" if action else " REFUSED"}', flush=True)
            if action is None:
                response = {'error': 'herramienta o argumento fuera de la lista permitida'}
            elif action == SLEEP:
                self.sleep_asked = True       # the goodbye plays, then SOMA sleeps
                self.sleep_grace = 1
                response = {'ok': True}
            else:
                self.command(action[1])
                response = {'ok': True}
            # SILENT: Gemini takes the answer without a new turn about it. A
            # blocking call (dormir) ignores scheduling and simply continues.
            responses.append(types.FunctionResponse(
                id=fc.id, name=fc.name, response=response,
                scheduling=None if action == SLEEP else types.FunctionResponseScheduling.SILENT))
        if responses:
            await session.send_tool_response(function_responses=responses)

    async def downlink(self, session, types) -> None:
        while True:
            got = False
            async for msg in session.receive():
                got = True
                if msg.go_away:
                    raise GoToSleep('the server is closing the session')
                if msg.usage_metadata and msg.usage_metadata.total_token_count:
                    self.turn['tokens'] = msg.usage_metadata.total_token_count
                # Tool calls come without server_content: before the check below.
                if msg.tool_call and msg.tool_call.function_calls:
                    await self.answer_tools(session, msg.tool_call.function_calls, types)
                sc = msg.server_content
                if not sc:
                    continue
                if sc.input_transcription and sc.input_transcription.text:
                    self.turn['heard'] = self.turn.get('heard', '') + sc.input_transcription.text
                    if says(self.turn['heard'], SLEEP_WORDS):
                        self.sleep_asked = True       # SOMA says goodbye first, then sleeps
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
                    self.envelope.clear()
                    self.send('X')
                if sc.turn_complete:
                    spoke = 'first' in self.turn
                    self.log_turn()
                    if self.sleep_asked and not spoke and self.sleep_grace > 0:
                        # dormir arrived but no goodbye played in this turn:
                        # give Gemini one more turn to say it, not a silent exit.
                        self.sleep_grace -= 1
                        continue
                    if self.sleep_asked:
                        while self.pending or self.speaker.speaking(time.monotonic()):
                            await asyncio.sleep(0.05)
                        raise GoToSleep('asked to sleep')
            if not got:
                raise ConnectionError('the conversation session closed')

    async def speaker_feed(self) -> None:
        while True:
            now = time.monotonic()
            if self.pending and self.speaker.may_send(MAX_PAYLOAD, now):
                chunk = bytes(self.pending[:MAX_PAYLOAD])
                del self.pending[:MAX_PAYLOAD]
                self.speaker.inflight += 1
                self.envelope.add(chunk, now)
                self.send('P', chunk)
                continue
            await asyncio.sleep(0.005)

    async def eyes(self) -> None:
        """Layer 1: 20 times a second, where the conversation is becomes eyes."""
        while True:
            now = time.monotonic()
            for line in self.director.tick(now, self.speaker.speaking(now), self.voice.last,
                                           self.envelope.level(now)):
                self.command(line)
            await asyncio.sleep(0.05)

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
    ap.add_argument('--vol', type=int, default=DEFAULT_VOL, help=f'speaker volume 0..100 (default {DEFAULT_VOL})')
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
