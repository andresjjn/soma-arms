# SOMA's voice: plan

Off the ladder, like the eyes (docs/plan.md section 8): time-boxed, and it
never delays v0.3. Decided 2026-11-10.

## What it is

SOMA listens through the eye board's microphones and answers through its
speaker, in Spanish, with the Gemini Live API doing the conversation.

```
mic (ES7210) -> ESP32-S3 -> USB serial -> relay (Python) -> Gemini Live API
speaker (ES8311) <- ESP32-S3 <- USB serial <- relay <- response audio
```

## Decisions

- **The brain lives on the host, not on the board.** The relay (Python,
  `google-genai`) owns the Live session, the system prompt, the tools and
  the model ID. The Mac stands in until the Jetson is back; then the same
  script runs on the Jetson, next to the driver and the v0.5 supervisor.
  One orchestrator, one context: a voice session inside the ESP32 would be
  a second brain that shares nothing with the one that sees and moves, and
  every change to its behavior would need a reflash.
- **USB serial is the transport.** The cable already powers the board and
  carries the eye protocol; 16 bit audio at 16 to 24 kHz is 32 to 48 KB/s,
  well inside it, with the block-and-acknowledge scheme `img` proved. No
  WiFi credentials and no API key on the board.
- **The key stays on the host**, in the Keychain or the environment. A
  dedicated key, restricted to the Generative Language API, with a spend
  limit on its project.
- **The model ID is configuration.** The native audio Live models are
  previews or carry retirement dates (the Vertex GA one is listed to retire
  on 2026-12-13): check the current ID when wiring phase 1.
- **Voice never moves an arm.** The relay exposes no motion tool in any of
  these phases. When v0.5 arrives, a primitive asked for by voice goes
  behind the two arming gates like any other call from the reasoner.
- **Listening is always visible.** Whenever audio leaves the board, the
  eyes show it.
- **Spanish first, neutral Latin American, one voice.** Without a fixed
  voice the model picked a different one per session, sometimes a man's,
  sometimes a woman's. SOMA speaks with the prebuilt voice Capella and
  `es-US` (Google's Latin American Spanish), and the system prompt asks
  for a neutral Spanish with no regional idioms (2026-11-10; Mexican
  Spanish first, then neutral at Andres's request).

## Board facts (from Waveshare's 03_Mic_test and the board's xiaozhi config)

| Signal | GPIO |
|---|---|
| I2S MCLK / BCLK / WS | 12 / 13 / 14 |
| I2S DIN (from ES7210, mics) | 15 |
| I2S DOUT (to ES8311, speaker) | 16 |
| Speaker amplifier enable | 9 |

Both codecs share BCLK and WS, so input and output run at one sample rate.

## Phase 0 results (2026-11-10)

- Both codecs start at boot (`stats` says `audio ready`). Channel 0 is the
  microphone; channel 1 stays at -78 dBFS with or without voice, so phase 1
  sends channel 0 only, mono.
- Voice at about 50 cm with the ES7210 at 30 dB gain: RMS -30 dBFS, peaks
  -13 to -22 dBFS, room floor -62 to -64 dBFS (about 34 dB of margin), no
  clipped samples. The gain stays.
- `selftest`: a 1 kHz tone at -12 dBFS out the speaker reads -16.4 dBFS at
  the board's own microphone against -72.8 quiet, +56.4 dB: the speaker
  sounds, measured without an ear.
- `loop`: Andres's voice played back was clearly understood, quiet at
  volume 75 (the recording peaks 14 to 22 dB under full scale; Gemini's
  output comes much nearer full scale; `vol` changes it live).
- One unexplained reboot during a `loop` (the boot banner arrived instead of
  the header); not reproduced in four loops and tones up to -1 dBFS.
  `stats` now reports the reset reason (`boot N`, esp_reset_reason) so the
  next one names itself. Two corrupted headers coincided with two processes
  on the serial port at once.
- While audio runs the eyes stop (phase 0 is blocking); phase 1 moves audio
  to its own task.

## Phase 1 results (2026-11-10)

- Three Live models answered a synthesized Spanish question with this key:
  first audio 1.54 s after the question (gemini-3.8-live, 809 tokens),
  1.63 s (gemini-3.1-flash-live-preview, 382 tokens, the only one that
  heard "SOMA"), 2.63 s (gemini-2.5-flash-native-audio-latest, 539).
  All answer 24 kHz PCM. Default: 3.1 flash live; `--model` changes it.
- Transport: `stream` switches the line protocol to frames (eyes/frames.h):
  microphone up at 50.5 frames a second, no wrong sizes; speaker audio down
  in 2 KB frames, a window of 4, acked with the queue level; a 16 s ring in
  PSRAM absorbs Gemini's bursts; 0 frames dropped. The relay resamples 24 to
  16 kHz (windowed sinc, tested: 1 kHz passes within 2 %, a 10 kHz alias is
  over 40 dB down, chunks leave no seam).
- End to end, the board's own microphone hearing a question played by the
  Mac's speaker: two turns, 1.63 s and 1.98 s from the last word to
  Gemini's first audio, 470 and 795 tokens; SOMA said it cannot move its
  arms by voice yet, as its prompt asks.
- Found and fixed on the way: the mic task and the acks shared one frame
  buffer across the two cores (frames came out mixed; now a lock); a lone
  click up to -29 dBFS counted as speech and read the latency as 0.1 s (a
  word is now 3 frames in a row over -40 dBFS); a stream that ends with the
  port closed leaves frames in the buffers for the next open (the tools
  drain before talking); the 3 s timeout itself works.
- Half duplex: while SOMA speaks the microphone sends silence.

## Phase 2, layer 1: the eyes follow the conversation (2026-11-10)

The relay knows where the conversation is without asking any model, and an
`EyeDirector` turns that into eye commands 20 times a second: listening
(slow wide saccades), hearing (fixed on the person, wide pupils), thinking
(the person ended, no answer yet: a look up, to the other side each turn),
speaking (micro-saccades, the pupil beating with the voice on the speaker's
clock, blinks at phrase ends). The firmware now eases the pupil like the
gaze, so many small steps read as one movement. Layer 2, an `expresar`
mood tool for Gemini (non-blocking, silent), comes next. The session
volume defaults to 75 (85 felt like too much for the small speaker).

## Asleep and awake (2026-11-10)

SOMA's eyes are shut unless it is in a conversation: the board boots with
them closed and closes them whenever a stream ends, even by the 3 s
timeout, so a crashed relay never leaves them open. The relay starts
asleep: a transcription-only Live model (gemini-3.5-transcribe-live, which
never answers) listens for "despierta", and audio leaves the board only
while someone talks (a 0.5 s preroll keeps the first syllable; a quiet
room sends nothing). "Despierta" opens the eyes over about a second and
starts the conversation with a short greeting; "duérmete", Ctrl-C or a
lost session closes them, and the relay listens for "despierta" again.
Tested end to end: wake, two turns, a goodbye, asleep again; latencies
1.2 to 1.5 s.

Layer 1 was reworked after Andres saw it: a pupil beating with the voice
looked wrong (people's pupils do not do that; they dilate with mental
effort). Now the pupil dilates while thinking and stays still while
speaking; each of SOMA's phrases starts looking away and ends looking back
at the person; the gaze searches up and aside while thinking; the eyes
blink less while hearing, and sometimes once in the person's pause.

Waking late, fixed the same day: the gate stopped sending audio a second
after the phrase, so the transcription model kept waiting for the silence
that ends an utterance and "despierta" only registered when more sound
reopened the gate. Measured against the API with a synthesized "SOMA,
despierta": before, no wake within 6 s; with `audio_stream_end` when the
gate closes, 1.2 s after the phrase; waking on the partial transcription
as well, 0.6 to 0.7 s.

Half duplex, measured: the speaker's sound at the microphone falls from
-32 dBFS to the -63 floor about 0.2 s after the relay's estimate of the
end of playback, inside the 0.35 s tail.

Video of the first take: `docs/media/video/voice_wake_2026-11-10.mp4`.

## Phases, each with its gate

| Phase | Work | Gate |
|---|---|---|
| 0 (passed) | Codecs and I2S in the eye firmware; `rec S` streams S s of microphone to the Mac, `play N` plays PCM the Mac sends in acknowledged blocks, `selftest` checks the speaker with the board's own microphone; noise floor, voice level at 50 cm, clipping | a recording played back is clearly understood |
| 1 (works; a human turn pending) | Audio in its own task so the eyes keep about 50 fps; `tools/voice_relay.py`: serial audio to and from a Live session; latency from end of speech to first sound measured; cost of one minute measured | one spoken question, one spoken answer, numbers logged |
| 2 | Touch an eye to talk, barge-in, the eyes follow the conversation (listening, thinking, speaking) through `look`, `pupil`, `mood` | a short video |
| 3 | The relay on the Jetson, the serial device passed into the container | phase 2 working from the Jetson |
| 4 | "SOMA, wake up" on the board with ESP-SR, so audio leaves only after the phrase; needs a model partition, so a full reflash | wakes on the phrase, not on ordinary talk in the room |
