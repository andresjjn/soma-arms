// SOMA's voice, phase 0 (docs/voice_plan.md): the board's microphones and
// speaker over the USB serial port, no network. Hardware only.
#pragma once
#include <cstddef>
#include <cstdint>

namespace audio {

// Codecs and I2S at `rate`; false (and audio stays off) if a codec does not
// answer. The eyes keep working either way.
bool begin(uint32_t rate);
bool ready();

// `rec S`: S seconds of the microphones streamed to the host as
// "pcm N RATE 2", N raw bytes of 16 bit stereo frames, "ok".
void rec(float seconds);

// `play N`: N bytes of 16 bit mono at the I2S rate arrive in acknowledged
// blocks, then go out the speaker.
void play(uint32_t bytes);

void volume(int percent);

// `selftest`: 0.5 s quiet, then 1.5 s of a 1 kHz tone at -12 dBFS out the
// speaker while the microphones record, streamed like `rec`. The host
// compares the 1 kHz energy of the two parts: the speaker sounds or not.
void selftest();

// Phase 1, the conversation (frames.h): the microphone streams 20 ms frames
// and the speaker plays a queue the host fills, each in its own task on
// core 0 while the eyes keep drawing on core 1.
bool stream_begin();
void stream_end();
bool streaming();
void stream_frame(uint8_t type, const uint8_t* payload, size_t len);   // P, X, N
void stream_poll();                         // leaves the stream if the host went quiet

}  // namespace audio
