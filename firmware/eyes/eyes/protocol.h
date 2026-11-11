// SOMA's eyes: the line protocol over USB serial. Pure C++, no hardware.
//
//   ping               -> pong soma-eyes 0.1
//   look X Y           -> ok        gaze, each -1..1 (X + toward the picture's right)
//   blink              -> ok
//   mood NAME          -> ok        neutral | happy | sad | angry | surprised | sleepy
//   pupil S            -> ok        0.5..1.5
//   bright N           -> ok        backlight, 0..100
//   idle on|off        -> ok        wandering gaze and blinks of their own
//   set KEY VALUE      -> ok        one design field live (style.h): iris_r 58,
//                                   colors.iris_in #ff8800, happy.low 80, ...
//   style              -> {...}     the whole design as JSON, for style.json
//   save | reset       -> ok        keep the live design across reboots | back to default
//   stats              -> fps ...   frames drawn and skipped since the last stats
//   rec S              -> pcm N RATE 2, N raw bytes, ok
//                                   S seconds (0.1..10) of the microphones, 16 bit
//                                   stereo frames as the codec delivers them
//   play N             -> k ... ok  N bytes (even, at most 10 s) of 16 bit mono at
//                                   AUDIO_RATE follow in AUDIO_BLOCK blocks, each
//                                   acknowledged with k, then the speaker plays them
//   vol N              -> ok        speaker volume, 0..100
//   stream             -> ok, then frames both ways (frames.h) until `stream off`
//                                   or 3 s without a frame from the host
//   selftest           -> pcm ...   like rec: 0.5 s quiet, then 1.5 s with a 1 kHz
//                                   tone out the speaker, so the microphones tell
//                                   whether the speaker sounds
//
// Anything else answers `err <why>`. The eyes never arm or move anything:
// they are off SOMA's safety path by construction.
#pragma once
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "style.h"

namespace eyes {

enum class Cmd : uint8_t { None, Ping, Look, Blink, SetMood, Pupil, Bright, Idle, Set, Style, Save, Reset, Stats,
                           Rec, Play, Vol, SelfTest, Stream };

// Audio (docs/voice_plan.md, phase 0): one I2S rate for both codecs.
constexpr uint32_t AUDIO_RATE = 16000;
constexpr float AUDIO_MAX_S = 10.0f;
constexpr uint32_t AUDIO_MAX_PLAY = uint32_t(AUDIO_RATE * AUDIO_MAX_S) * 2;   // mono bytes
constexpr uint32_t AUDIO_BLOCK = 4096;

struct Parsed {
  Cmd cmd = Cmd::None;
  float a = 0, b = 0;
  Mood mood = Mood::Neutral;
  bool on = false;
  const char* key = nullptr;   // Set: point into the parsed line
  const char* value = nullptr;
  const char* error = nullptr;
  uint32_t bytes = 0;          // Play
};

inline bool parse_float(const char* s, float lo, float hi, float* out) {
  if (!s || !*s) return false;
  char* end = nullptr;
  float v = std::strtof(s, &end);
  if (end == s || *end != '\0' || !(v >= lo && v <= hi)) return false;
  *out = v;
  return true;
}

inline bool mood_from(const char* s, Mood* m) {
  for (int i = 0; s && i < MOODS; i++)
    if (std::strcmp(s, mood_name(i)) == 0) { *m = Mood(i); return true; }
  return false;
}

// Parse one line (no newline). `buf` is modified (split into words).
inline Parsed parse(char* buf) {
  Parsed p;
  char* words[4] = {nullptr, nullptr, nullptr, nullptr};
  int n = 0;
  for (char* t = std::strtok(buf, " \t\r"); t && n < 4; t = std::strtok(nullptr, " \t\r")) words[n++] = t;
  if (n == 0) { p.error = "empty"; return p; }
  const char* w = words[0];
  if (!std::strcmp(w, "ping") && n == 1) { p.cmd = Cmd::Ping; return p; }
  if (!std::strcmp(w, "blink") && n == 1) { p.cmd = Cmd::Blink; return p; }
  if (!std::strcmp(w, "style") && n == 1) { p.cmd = Cmd::Style; return p; }
  if (!std::strcmp(w, "save") && n == 1) { p.cmd = Cmd::Save; return p; }
  if (!std::strcmp(w, "reset") && n == 1) { p.cmd = Cmd::Reset; return p; }
  if (!std::strcmp(w, "stats") && n == 1) { p.cmd = Cmd::Stats; return p; }
  if (!std::strcmp(w, "set")) {
    // the value is checked when it is applied (set_param), against its field
    if (n == 3) { p.cmd = Cmd::Set; p.key = words[1]; p.value = words[2]; return p; }
    p.error = "set KEY VALUE"; return p;
  }
  if (!std::strcmp(w, "look")) {
    if (n == 3 && parse_float(words[1], -1, 1, &p.a) && parse_float(words[2], -1, 1, &p.b)) { p.cmd = Cmd::Look; return p; }
    p.error = "look X Y, each -1..1"; return p;
  }
  if (!std::strcmp(w, "mood")) {
    if (n == 2 && mood_from(words[1], &p.mood)) { p.cmd = Cmd::SetMood; return p; }
    p.error = "mood neutral|happy|sad|angry|surprised|sleepy"; return p;
  }
  if (!std::strcmp(w, "pupil")) {
    if (n == 2 && parse_float(words[1], 0.5f, 1.5f, &p.a)) { p.cmd = Cmd::Pupil; return p; }
    p.error = "pupil S, 0.5..1.5"; return p;
  }
  if (!std::strcmp(w, "bright")) {
    if (n == 2 && parse_float(words[1], 0, 100, &p.a)) { p.cmd = Cmd::Bright; return p; }
    p.error = "bright N, 0..100"; return p;
  }
  if (!std::strcmp(w, "selftest") && n == 1) { p.cmd = Cmd::SelfTest; return p; }
  if (!std::strcmp(w, "stream") && n == 1) { p.cmd = Cmd::Stream; return p; }
  if (!std::strcmp(w, "rec")) {
    if (n == 2 && parse_float(words[1], 0.1f, AUDIO_MAX_S, &p.a)) { p.cmd = Cmd::Rec; return p; }
    p.error = "rec S, 0.1..10 seconds"; return p;
  }
  if (!std::strcmp(w, "play")) {
    char* end = nullptr;
    unsigned long v = n == 2 ? std::strtoul(words[1], &end, 10) : 0;
    if (n == 2 && end != words[1] && *end == '\0' && words[1][0] != '-' && v >= 2 && v <= AUDIO_MAX_PLAY && v % 2 == 0) {
      p.cmd = Cmd::Play; p.bytes = uint32_t(v); return p;
    }
    p.error = "play N, even bytes, at most 10 s of 16 bit mono"; return p;
  }
  if (!std::strcmp(w, "vol")) {
    if (n == 2 && parse_float(words[1], 0, 100, &p.a)) { p.cmd = Cmd::Vol; return p; }
    p.error = "vol N, 0..100"; return p;
  }
  if (!std::strcmp(w, "idle")) {
    if (n == 2 && (!std::strcmp(words[1], "on") || !std::strcmp(words[1], "off"))) {
      p.cmd = Cmd::Idle; p.on = !std::strcmp(words[1], "on"); return p;
    }
    p.error = "idle on|off"; return p;
  }
  p.error = "unknown command";
  return p;
}

}  // namespace eyes
