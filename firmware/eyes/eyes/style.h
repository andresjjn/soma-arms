// SOMA's eyes: the design as data. Pure C++, no hardware.
//
// Every visual number lives in a Style. The default comes from
// firmware/eyes/style.json through tools/gen_style.py (style_default.h);
// on the board, `set KEY VALUE` changes one field live and `style` prints
// the whole design back as JSON, ready to paste into style.json.
//
// The design is spec v2 (2026-11-22, Andres): a flat sclera with a rim, an
// iris in five discrete rings, and metal shutter lids with a tip, a rim and
// two slanted joints, closing onto a common seam. eye_model.h has the
// geometry; the names below follow its layers.
#pragma once
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

namespace eyes {

struct Rgb { uint8_t r, g, b; };

enum class Mood : uint8_t { Neutral, Happy, Sad, Angry, Surprised, Sleepy };
constexpr int MOODS = 6;
inline const char* mood_name(int i) {
  static const char* names[MOODS] = {"neutral", "happy", "sad", "angry", "surprised", "sleepy"};
  return names[i];
}

// One mood's lids and pupil, in px from the eye's center, y down, across
// u = -1 (nose) .. +1 (outer corner):
//   upper lid edge  y_up  = up + tilt * u + arch * u^2
//   lower lid edge  y_low = low - curve * u^2
// and the pupil scale this mood multiplies the commanded one by.
struct MoodLids { float up, tilt, arch, low, curve, pupil; };

struct Style {
  // Sizes, px.
  float iris_r, pupil_r, gaze_reach, glint_dx, glint_dy, glint_r, sclera_r, lid_shadow;
  float ring_limbal, ring_outer, ring_mid, ring_inner_pad;     // iris ring radii, and the inner ring past the pupil
  float lid_tip, lid_rim;                                      // widths across the lid edge
  float joint_x, joint_slope, joint_w, bevel_x, bevel_w;       // the lid plates' joints
  float seam_y, seam_curve;                                    // where shut lids meet: y = seam_y - seam_curve * u^2
  Rgb sclera, sclera_shadow, sclera_rim, iris_inner, iris_mid1, iris_mid2, iris_outer, iris_limbal, iris_shadow,
      pupil, glint, glint_shadow, lid, lid_tip_color, lid_rim_color, lid_seam, lid_seam_hi;
  MoodLids moods[MOODS];
  // ms
  float blink_close, blink_hold, blink_open, sleep_close, sleep_open, mood_transition, saccade_min, saccade_max,
      blink_every_min, blink_every_max, look_holds;
};

// The settable fields, by the same names style.json uses. kind: 'f' a top
// level number, 'c' a color under "colors", 't' a time under "timing_ms".
struct Field { const char* name; size_t offset; char kind; };
#define EYES_F(n) {#n, offsetof(Style, n), 'f'}
#define EYES_C(n, f) {n, offsetof(Style, f), 'c'}
#define EYES_T(n) {#n, offsetof(Style, n), 't'}
inline const Field* fields(size_t* count) {
  static const Field table[] = {
      EYES_F(iris_r), EYES_F(pupil_r), EYES_F(gaze_reach), EYES_F(glint_dx), EYES_F(glint_dy), EYES_F(glint_r),
      EYES_F(sclera_r), EYES_F(lid_shadow), EYES_F(ring_limbal), EYES_F(ring_outer), EYES_F(ring_mid),
      EYES_F(ring_inner_pad), EYES_F(lid_tip), EYES_F(lid_rim), EYES_F(joint_x), EYES_F(joint_slope),
      EYES_F(joint_w), EYES_F(bevel_x), EYES_F(bevel_w), EYES_F(seam_y), EYES_F(seam_curve),
      EYES_C("sclera", sclera), EYES_C("sclera_shadow", sclera_shadow), EYES_C("sclera_rim", sclera_rim),
      EYES_C("iris_inner", iris_inner), EYES_C("iris_mid1", iris_mid1), EYES_C("iris_mid2", iris_mid2),
      EYES_C("iris_outer", iris_outer), EYES_C("iris_limbal", iris_limbal), EYES_C("iris_shadow", iris_shadow),
      EYES_C("pupil", pupil), EYES_C("glint", glint), EYES_C("glint_shadow", glint_shadow), EYES_C("lid", lid),
      EYES_C("lid_tip", lid_tip_color), EYES_C("lid_rim", lid_rim_color), EYES_C("lid_seam", lid_seam),
      EYES_C("lid_seam_hi", lid_seam_hi),
      EYES_T(blink_close), EYES_T(blink_hold), EYES_T(blink_open), EYES_T(sleep_close), EYES_T(sleep_open),
      EYES_T(mood_transition), EYES_T(saccade_min), EYES_T(saccade_max), EYES_T(blink_every_min),
      EYES_T(blink_every_max), EYES_T(look_holds)};
  *count = sizeof(table) / sizeof(table[0]);
  return table;
}
#undef EYES_F
#undef EYES_C
#undef EYES_T

constexpr const char* MOOD_FIELDS[6] = {"up", "tilt", "arch", "low", "curve", "pupil"};
inline float* mood_field(MoodLids& m, int i) {
  float* f[6] = {&m.up, &m.tilt, &m.arch, &m.low, &m.curve, &m.pupil};
  return f[i];
}

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

// Whether a design is safe to draw. Sizes stay on the panel (radius 120
// px), lids within two panels of the center, pupil scales sane, and no
// negative timing (it would pause the eye for weeks). set refuses a value
// that breaks this, and the board drops a saved design that fails it, so a
// bad `save` cannot outlive a reboot.
inline bool valid(const Style& s) {
  auto in = [](float v, float lo, float hi) { return v >= lo && v <= hi; };
  const float r = 120.0f;
  for (float v : {s.iris_r, s.pupil_r, s.gaze_reach, s.glint_r, s.sclera_r, s.lid_shadow, s.ring_limbal,
                  s.ring_outer, s.ring_mid, s.ring_inner_pad, s.joint_x, s.bevel_x})
    if (!in(v, 0, r)) return false;
  for (float v : {s.glint_dx, s.glint_dy, s.seam_y, s.seam_curve})
    if (!in(v, -r, r)) return false;
  for (float v : {s.lid_tip, s.lid_rim, s.joint_w, s.bevel_w})
    if (!in(v, 0, 20)) return false;
  if (!in(s.joint_slope, -2, 2)) return false;
  for (const MoodLids& m : s.moods) {
    for (float v : {m.up, m.tilt, m.arch, m.low, m.curve})
      if (!in(v, -2 * r, 2 * r)) return false;
    if (!in(m.pupil, 0.25f, 2.0f)) return false;
  }
  for (float t : {s.blink_close, s.blink_hold, s.blink_open, s.sleep_close, s.sleep_open, s.mood_transition,
                  s.saccade_min, s.saccade_max, s.blink_every_min, s.blink_every_max, s.look_holds})
    if (!in(t, 0, 60000)) return false;
  return true;
}

inline bool set_param_unchecked(Style& s, const char* key, const char* value);

// `set KEY VALUE`: a top-level field, colors.NAME, timing_ms.NAME, or MOOD.FIELD
// (e.g. happy.low, sad.pupil). False, and s unchanged, if the key or the
// value is not understood or the result would not be valid().
inline bool set_param(Style& s, const char* key, const char* value) {
  Style next = s;
  if (!set_param_unchecked(next, key, value) || !valid(next)) return false;
  s = next;
  return true;
}

inline bool set_param_unchecked(Style& s, const char* key, const char* value) {
  const char* dot = std::strchr(key, '.');
  size_t n;
  const Field* f = fields(&n);
  for (size_t i = 0; i < n; i++) {
    const char* prefix = f[i].kind == 'c' ? "colors." : f[i].kind == 't' ? "timing_ms." : "";
    size_t pl = std::strlen(prefix);
    if (std::strncmp(key, prefix, pl) || std::strcmp(key + pl, f[i].name)) continue;
    char* base = reinterpret_cast<char*>(&s) + f[i].offset;
    if (f[i].kind == 'c') return parse_color(value, reinterpret_cast<Rgb*>(base));
    return parse_number(value, reinterpret_cast<float*>(base));
  }
  if (!dot) return false;
  for (int m = 0; m < MOODS; m++) {
    if (std::strncmp(key, mood_name(m), dot - key) || std::strlen(mood_name(m)) != size_t(dot - key)) continue;
    for (int i = 0; i < 6; i++)
      if (!std::strcmp(dot + 1, MOOD_FIELDS[i])) return parse_number(value, mood_field(s.moods[m], i));
    return false;
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
  size_t n;
  const Field* f = fields(&n);
  const char* base = reinterpret_cast<const char*>(&s);
  put("%s", "{");
  for (size_t i = 0; i < n; i++)
    if (f[i].kind == 'f') put("\"%s\":%g,", f[i].name, double(*reinterpret_cast<const float*>(base + f[i].offset)));
  put("%s", "\"colors\":{");
  bool first = true;
  for (size_t i = 0; i < n; i++)
    if (f[i].kind == 'c') {
      const Rgb* c = reinterpret_cast<const Rgb*>(base + f[i].offset);
      put("%s\"%s\":\"#%02x%02x%02x\"", first ? "" : ",", f[i].name, c->r, c->g, c->b);
      first = false;
    }
  put("%s", "},\"moods\":{");
  for (int m = 0; m < MOODS; m++) {
    const MoodLids& l = s.moods[m];
    put("%s\"%s\":{\"up\":%g,\"tilt\":%g,\"arch\":%g,\"low\":%g,\"curve\":%g,\"pupil\":%g}", m ? "," : "",
        mood_name(m), double(l.up), double(l.tilt), double(l.arch), double(l.low), double(l.curve), double(l.pupil));
  }
  put("%s", "},\"timing_ms\":{");
  first = true;
  for (size_t i = 0; i < n; i++)
    if (f[i].kind == 't') {
      put("%s\"%s\":%g", first ? "" : ",", f[i].name, double(*reinterpret_cast<const float*>(base + f[i].offset)));
      first = false;
    }
  put("%s", "}}");
  return w;
}

}  // namespace eyes
