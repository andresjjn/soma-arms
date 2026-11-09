// SOMA's eyes: the design as data. Pure C++, no hardware.
//
// Every visual number lives in a Style. The default comes from
// firmware/eyes/style.json through tools/gen_style.py (style_default.h);
// on the board, `set KEY VALUE` changes one field live and `style` prints
// the whole design back as JSON, ready to paste into style.json.
#pragma once
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace eyes {

struct Rgb { uint8_t r, g, b; };

enum class Mood : uint8_t { Neutral, Happy, Sad, Angry, Surprised, Sleepy };
constexpr int MOODS = 6;
inline const char* mood_name(int i) {
  static const char* names[MOODS] = {"neutral", "happy", "sad", "angry", "surprised", "sleepy"};
  return names[i];
}

struct MoodLids { float up, tilt, low, curve; };

struct Style {
  float iris_r, pupil_r, gaze_reach, glint_dx, glint_dy, glint_r, lid_shadow;
  Rgb sclera, sclera_edge, iris_in, iris_out, pupil, lid, glint;
  MoodLids moods[MOODS];
  float blink_close, blink_open, saccade_min, saccade_max, blink_every_min, blink_every_max, look_holds;
};

// The settable fields, by the same names style.json uses.
struct Field { const char* name; size_t offset; char kind; };   // kind: 'f' float, 'c' color
#define EYES_F(n) {#n, offsetof(Style, n), 'f'}
#define EYES_C(n) {"colors." #n, offsetof(Style, n), 'c'}
#define EYES_T(n) {"timing_ms." #n, offsetof(Style, n), 'f'}
inline const Field* fields(size_t* count) {
  static const Field table[] = {
      EYES_F(iris_r), EYES_F(pupil_r), EYES_F(gaze_reach), EYES_F(glint_dx), EYES_F(glint_dy),
      EYES_F(glint_r), EYES_F(lid_shadow),
      EYES_C(sclera), EYES_C(sclera_edge), EYES_C(iris_in), EYES_C(iris_out), EYES_C(pupil),
      EYES_C(lid), EYES_C(glint),
      EYES_T(blink_close), EYES_T(blink_open), EYES_T(saccade_min), EYES_T(saccade_max),
      EYES_T(blink_every_min), EYES_T(blink_every_max), EYES_T(look_holds)};
  *count = sizeof(table) / sizeof(table[0]);
  return table;
}
#undef EYES_F
#undef EYES_C
#undef EYES_T

inline bool parse_color(const char* s, Rgb* out) {
  if (!s || s[0] != '#' || std::strlen(s) != 7) return false;
  char* end = nullptr;
  unsigned long v = std::strtoul(s + 1, &end, 16);
  if (*end != '\0') return false;
  *out = {uint8_t(v >> 16), uint8_t(v >> 8), uint8_t(v)};
  return true;
}

inline bool parse_number(const char* s, float* out) {
  if (!s || !*s) return false;
  char* end = nullptr;
  float v = std::strtof(s, &end);
  if (end == s || *end != '\0' || v != v || v < -1000 || v > 100000) return false;
  *out = v;
  return true;
}

// `set KEY VALUE`: a top-level field, colors.NAME, timing_ms.NAME, or MOOD.FIELD
// (e.g. happy.low). False if the key or the value is not understood.
inline bool set_param(Style& s, const char* key, const char* value) {
  size_t n;
  const Field* f = fields(&n);
  for (size_t i = 0; i < n; i++) {
    if (std::strcmp(key, f[i].name)) continue;
    char* base = reinterpret_cast<char*>(&s) + f[i].offset;
    if (f[i].kind == 'c') return parse_color(value, reinterpret_cast<Rgb*>(base));
    return parse_number(value, reinterpret_cast<float*>(base));
  }
  const char* dot = std::strchr(key, '.');
  if (!dot) return false;
  for (int m = 0; m < MOODS; m++) {
    if (std::strncmp(key, mood_name(m), dot - key) || std::strlen(mood_name(m)) != size_t(dot - key)) continue;
    const char* field = dot + 1;
    float* target = !std::strcmp(field, "up") ? &s.moods[m].up
                  : !std::strcmp(field, "tilt") ? &s.moods[m].tilt
                  : !std::strcmp(field, "low") ? &s.moods[m].low
                  : !std::strcmp(field, "curve") ? &s.moods[m].curve : nullptr;
    return target && parse_number(value, target);
  }
  return false;
}

// The whole style as compact JSON, in style.json's shape. Returns the length.
inline int dump_json(const Style& s, char* out, size_t cap) {
  int w = 0;
  auto put = [&](const char* fmt, auto... args) {
    int k = std::snprintf(out + w, w < int(cap) ? cap - w : 0, fmt, args...);
    if (k > 0) w += k;
  };
  put("{\"iris_r\":%g,\"pupil_r\":%g,\"gaze_reach\":%g,\"glint_dx\":%g,\"glint_dy\":%g,"
      "\"glint_r\":%g,\"lid_shadow\":%g,\"colors\":{",
      s.iris_r, s.pupil_r, s.gaze_reach, s.glint_dx, s.glint_dy, s.glint_r, s.lid_shadow);
  const Rgb* cs[] = {&s.sclera, &s.sclera_edge, &s.iris_in, &s.iris_out, &s.pupil, &s.lid, &s.glint};
  const char* cn[] = {"sclera", "sclera_edge", "iris_in", "iris_out", "pupil", "lid", "glint"};
  for (int i = 0; i < 7; i++) put("%s\"%s\":\"#%02x%02x%02x\"", i ? "," : "", cn[i], cs[i]->r, cs[i]->g, cs[i]->b);
  put("%s", "},\"moods\":{");
  for (int m = 0; m < MOODS; m++)
    put("%s\"%s\":{\"up\":%g,\"tilt\":%g,\"low\":%g,\"curve\":%g}", m ? "," : "", mood_name(m),
        s.moods[m].up, s.moods[m].tilt, s.moods[m].low, s.moods[m].curve);
  put("},\"timing_ms\":{\"blink_close\":%g,\"blink_open\":%g,\"saccade_min\":%g,\"saccade_max\":%g,"
      "\"blink_every_min\":%g,\"blink_every_max\":%g,\"look_holds\":%g}}",
      s.blink_close, s.blink_open, s.saccade_min, s.saccade_max, s.blink_every_min, s.blink_every_max,
      s.look_holds);
  return w;
}

}  // namespace eyes
