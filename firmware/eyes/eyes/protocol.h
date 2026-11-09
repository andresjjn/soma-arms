// SOMA's eyes: the line protocol over USB serial. Pure C++, no hardware.
//
//   ping               -> pong soma-eyes 0.1
//   look X Y           -> ok        gaze, each -1..1 (X + toward the picture's right)
//   blink              -> ok
//   mood NAME          -> ok        neutral | happy | sad | angry | surprised | sleepy
//   pupil S            -> ok        0.5..1.5
//   bright N           -> ok        backlight, 0..100
//   idle on|off        -> ok        wandering gaze and blinks of their own
//
// Anything else answers `err <why>`. The eyes never arm or move anything:
// they are off SOMA's safety path by construction.
#pragma once
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "eye_model.h"

namespace eyes {

enum class Cmd : uint8_t { None, Ping, Look, Blink, SetMood, Pupil, Bright, Idle };

struct Parsed {
  Cmd cmd = Cmd::None;
  float a = 0, b = 0;
  Mood mood = Mood::Neutral;
  bool on = false;
  const char* error = nullptr;
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
  static const struct { const char* name; Mood mood; } table[] = {
      {"neutral", Mood::Neutral}, {"happy", Mood::Happy}, {"sad", Mood::Sad},
      {"angry", Mood::Angry}, {"surprised", Mood::Surprised}, {"sleepy", Mood::Sleepy}};
  for (auto& e : table)
    if (s && std::strcmp(s, e.name) == 0) { *m = e.mood; return true; }
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
