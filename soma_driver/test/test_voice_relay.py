"""The voice relay's pure parts (docs/voice_plan.md, phase 1): the board's
frames, the 24 to 16 kHz resampler and the half duplex estimate. No serial
port, no network, no Gemini."""
import struct
import sys
from pathlib import Path

import pytest

np = pytest.importorskip('numpy')
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'firmware' / 'eyes' / 'tools'))
import voice_relay as vr  # noqa: E402


def test_frames_are_found_between_text_and_split_anywhere():
    mic = vr.frame('M', b'\x01\x02\x03\x04')
    ack = bytes([vr.MAGIC, ord('K'), 8, 0]) + struct.pack('<II', 4096, 1 << 19)
    stream = b'ok\r\n' + mic + b'stream ended\n' + ack
    parser = vr.BoardFrames()
    frames, lines = [], []
    for i in range(len(stream)):                     # one byte at a time
        f, l = parser.feed(stream[i:i + 1])
        frames += f
        lines += l
    assert frames == [('M', b'\x01\x02\x03\x04'), ('K', ack[4:])]
    assert lines == ['ok', 'stream ended']


def test_a_host_frame_type_from_the_board_is_not_audio():
    frames, _ = vr.BoardFrames().feed(vr.frame('P', b'\x00\x00') + vr.frame('M', b'\x05\x00'))
    assert frames == [('M', b'\x05\x00')]


def test_frames_refuse_oversized_payloads():
    with pytest.raises(ValueError):
        vr.frame('P', bytes(vr.MAX_PAYLOAD + 1))


def _tone(freq, rate, seconds=0.5):
    t = np.arange(int(rate * seconds)) / rate
    return 10000 * np.sin(2 * np.pi * freq * t)


def _level(x, freq, rate):
    x = x[len(x) // 4:]                              # past the filter's start
    t = np.arange(len(x)) / rate
    w = np.hanning(len(x))
    return 2 * abs(np.sum(x * w * np.exp(-2j * np.pi * freq * t))) / np.sum(w)


def test_speech_band_passes_and_aliases_are_cut():
    one_k = vr.Down24to16()(_tone(1000, 24000))
    assert len(one_k) == 8000                        # 0.5 s at 16 kHz
    assert _level(one_k, 1000, 16000) == pytest.approx(10000, rel=0.02)
    # 10 kHz would fold to 6 kHz at 16 kHz: it must be at least 40 dB down
    folded = vr.Down24to16()(_tone(10000, 24000))
    assert _level(folded, 6000, 16000) < 100


def test_chunks_leave_no_seam():
    x = _tone(440, 24000) + _tone(3100, 24000)
    whole = vr.Down24to16()(x)
    r = vr.Down24to16()
    pieces = np.concatenate([r(x[i:i + 777]) for i in range(0, len(x), 777)])
    assert len(pieces) == len(whole)
    assert np.max(np.abs(pieces - whole)) < 1e-6


def test_soma_does_not_hear_itself_while_the_speaker_drains():
    q = vr.SpeakerQueue()
    assert not q.speaking(10.0)
    q.inflight = 1
    assert q.speaking(10.0)                          # sent, not yet acked
    q.ack(32000, 1 << 19, 10.0)                      # one second queued on the board
    assert q.inflight == 0
    assert q.speaking(10.5)
    assert q.speaking(11.0 + vr.TAIL_S / 2)           # the tail after the queue empties
    assert not q.speaking(11.0 + vr.TAIL_S * 2)


def test_the_window_bounds_what_is_in_flight():
    q = vr.SpeakerQueue()
    q.ack(0, 1 << 19, 0.0)
    for _ in range(vr.WINDOW):
        assert q.may_send(vr.MAX_PAYLOAD, 0.0)
        q.inflight += 1
    assert not q.may_send(vr.MAX_PAYLOAD, 0.0)
    q.ack((1 << 19) - 1000, 1 << 19, 0.0)            # the board's queue is nearly full
    assert not q.may_send(vr.MAX_PAYLOAD, 0.0)


def test_the_relay_offers_no_way_to_the_arms():
    source = (Path(vr.__file__)).read_text()
    for word in ('/soma/arm', 'allow_real', 'soma/command', 'rclpy', 'tools=', 'FunctionDeclaration'):
        assert word not in source


def test_a_click_is_not_a_word_but_a_syllable_is():
    v = vr.VoiceTracker()
    for i, level in enumerate([-60, -29, -60, -60]):      # one loud frame alone
        v.frame(level, i * 0.02)
    assert v.last == 0.0
    for i, level in enumerate([-35, -32, -31, -33, -60]):  # 80 ms of voice
        v.frame(level, 1.0 + i * 0.02)
    assert v.last == pytest.approx(1.06)


def test_audio_caught_mid_frame_is_not_printed_as_board_text():
    _, lines = vr.BoardFrames().feed(b'\x12\xfe\x00\x81 \x7f\x03\n' + b'stream ended\n')
    assert lines == ['stream ended']
