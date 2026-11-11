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


# ---- layer 1 of the eyes: the conversation's phase becomes eyes ----

def _director():
    import random
    return vr.EyeDirector(random.Random(7))


def test_the_eyes_hear_then_think_then_speak_then_listen():
    d = _director()
    d.tick(0.0, False, 0.0)
    assert d.phase == 'listening'
    d.tick(1.0, False, 0.9)                          # a word 0.1 s ago
    assert d.phase == 'hearing'
    out = d.tick(1.6, False, 0.9)                    # the person stopped
    assert d.phase == 'thinking'
    look = [c for c in out if c.startswith('look')][0]
    assert float(look.split()[2]) < -0.4             # a look up
    d.tick(2.5, True, 0.9)
    assert d.phase == 'speaking'
    d.tick(5.0, False, 0.9)                          # SOMA finished: the old word is answered
    assert d.phase == 'listening'


def test_thinking_looks_to_a_different_side_each_turn():
    d = _director()
    sides = []
    for turn in range(3):
        t = 10.0 * turn
        d.tick(t, False, t - 0.1)                    # hearing
        out = d.tick(t + 1.0, False, t - 0.1)        # thinking
        sides.append(float([c for c in out if c.startswith('look')][0].split()[1]) > 0)
        d.tick(t + 2.0, True, t - 0.1)               # speaking
    assert sides[0] != sides[1] and sides[1] != sides[2]


def test_an_unanswered_turn_does_not_think_forever():
    d = _director()
    d.tick(0.0, False, 0.0)
    d.tick(1.0, False, 0.9)
    d.tick(2.0, False, 0.9)
    assert d.phase == 'thinking'
    d.tick(0.9 + vr.EyeDirector.THINK_MAX_S + 0.1, False, 0.9)
    assert d.phase == 'listening'


def _looks(out):
    return [tuple(float(v) for v in c.split()[1:]) for c in out if c.startswith('look')]


def _pupils(out):
    return [float(c.split()[1]) for c in out if c.startswith('pupil')]


def test_speaking_keeps_the_pupil_still():
    # 2026-11-10: a pupil beating with the voice looked wrong; people's don't.
    d = _director()
    pupils = []
    for i, level in enumerate([-60, -15, -12, -15, -60, -60, -15, -60]):
        pupils += _pupils(d.tick(i * 0.05, True, 0.0, level))
    assert len(set(pupils)) == 1


def test_thinking_dilates_the_pupils_and_the_gaze_searches_up_there():
    d = _director()
    d.tick(0.0, False, 0.0)
    d.tick(1.0, False, 0.95)                           # hearing
    out = d.tick(1.5, False, 0.95)                     # thinking
    assert _pupils(out) and _pupils(out)[0] > vr.EyeDirector.PUPIL['hearing']
    looks = _looks(out)
    for i in range(1, 60):                            # 3 s of thinking
        looks += _looks(d.tick(1.5 + i * 0.05, False, 0.95))
    assert len(looks) >= 4                            # it searches, it does not stare
    assert all(y < -0.3 for _, y in looks)            # and it stays up there


def test_a_phrase_starts_looking_away_and_ends_looking_back():
    d = _director()
    d.tick(0.0, True, 0.0, -60)
    start = _looks(d.tick(0.05, True, 0.0, -20))      # SOMA's phrase begins
    assert start and abs(start[0][0]) > 0.25          # away, to plan it
    out = []
    for i in range(2, 12):                            # loud for half a second
        d.tick(i * 0.05, True, 0.0, -20)
    for i in range(12, 22):                           # then a pause
        out += d.tick(i * 0.05, True, 0.0, -60)
    back = _looks(out)
    assert back and abs(back[0][0]) < 0.06 and abs(back[0][1]) < 0.06   # back at the person


def test_hearing_blinks_less_than_listening():
    d = _director()
    listening = sum('blink' in d.tick(i * 0.05, False, 0.0) for i in range(600))
    d2 = _director()
    hearing = sum('blink' in d2.tick(100 + i * 0.05, False, 100 + i * 0.05 - 0.05) for i in range(600))
    assert hearing < listening


def test_wake_and_sleep_words_survive_accents_and_punctuation():
    assert vr.says('Oye SOMA, ¡despiértate!', vr.WAKE_WORDS)
    assert vr.says('soma despierta', vr.WAKE_WORDS)
    assert not vr.says('estaba despiertamente cansado', vr.WAKE_WORDS)
    assert vr.says('Bueno, ya, duérmete.', vr.SLEEP_WORDS)
    assert vr.says('vete a dormir', vr.SLEEP_WORDS)
    assert not vr.says('no me puedo dormir', vr.SLEEP_WORDS)


def test_asleep_a_quiet_room_sends_nothing_and_a_voice_keeps_its_first_syllable():
    g = vr.VoiceGate()
    quiet = bytes(640)
    voice = vr.to_int16(np.full(320, 3000.0))         # about -21 dBFS
    sent = []
    for i in range(100):                              # 2 s of quiet room
        sent += g.frame(quiet, i * 0.02)
    assert sent == []
    for i in range(100, 103):                         # a word starts
        sent += g.frame(voice, i * 0.02)
    # the half second before (its last two frames are the word's start, which
    # were not yet confirmed as speech), then the frame that confirmed it
    assert len(sent) == vr.VoiceGate.PREROLL_S * 50 + 1
    assert sent[-3:] == [voice] * 3
    after = []
    for i in range(103, 103 + 100):                   # quiet again: held a second, then closed
        after += g.frame(quiet, i * 0.02)
    assert len(after) == pytest.approx(vr.VoiceGate.HOLD_S * 50, abs=2)


def test_a_steady_eye_sends_nothing_new():
    d = _director()
    d.tick(0.0, False, 0.0)
    quiet = d.tick(0.05, False, 0.0)
    assert not [c for c in quiet if c.startswith('pupil')]


def test_the_eyes_blink_by_themselves():
    d = _director()
    blinks = sum('blink' in d.tick(i * 0.05, False, 0.0) for i in range(400))   # 20 s
    assert 3 <= blinks <= 9


def test_the_envelope_plays_chunks_back_to_back_on_the_speaker_clock():
    e = vr.Envelope()
    loud = vr.to_int16(np.full(3200, 8000.0))                 # 0.2 s
    quiet = bytes(3200 * 2)
    e.add(loud, 10.0)
    e.add(quiet, 10.0)                                        # queued right after, not at 10.0
    assert e.level(10.1) > -20
    assert e.level(10.3) == -120.0                            # the quiet chunk is playing
    assert e.level(10.5) == -120.0                            # nothing left
    e.clear()
    assert e.level(10.05) == -120.0


def test_the_gate_says_once_when_the_talk_ended():
    # 2026-11-10: without this the transcription waited for silence that the
    # closed gate never sent, and "despierta" woke SOMA seconds late.
    g = vr.VoiceGate()
    voice = vr.to_int16(np.full(320, 3000.0))
    quiet = bytes(640)
    for i in range(5):
        g.frame(voice, i * 0.02)
    ends = []
    for i in range(5, 5 + 100):                        # two seconds of quiet
        g.frame(quiet, i * 0.02)
        ends.append(g.ended)
    assert sum(ends) == 1
    assert ends.index(True) == pytest.approx(vr.VoiceGate.HOLD_S * 50, abs=2)
