// SOMA's eyes: the eye as math, one pixel at a time. Pure C++, no hardware,
// so it builds on the board, in firmware/eyes/test and in the host preview
// (firmware/eyes/tools/preview.py) from the very same code.
//
// One eye is a 240x240 round GC9A01 panel. How it looks is data (style.h,
// from style.json); what it does right now is an EyeState (gaze, pupil,
// blink, mood). This file only turns the two into pixels.
#pragma once
#include <cmath>
#include <cstdint>
#include "style.h"

namespace eyes {

constexpr int SIZE = 240;
constexpr float R = SIZE / 2.0f;     // panel radius, px

struct EyeState {
  float gx = 0, gy = 0;   // gaze, -1..1 (x + toward the picture's right, y + down)
  float pupil = 1.0f;     // pupil scale, 0.5..1.5
  float blink = 0.0f;     // 0 open .. 1 shut
  Mood mood = Mood::Neutral;
};

// RGB565 with the bytes swapped: the panel takes the high byte first and
// the ESP32 is little endian.
inline uint16_t rgb565_swapped(Rgb c) {
  uint16_t v = uint16_t(((c.r & 0xF8) << 8) | ((c.g & 0xFC) << 3) | (c.b >> 3));
  return uint16_t((v >> 8) | (v << 8));
}

inline Rgb mix(Rgb a, Rgb b, float t) {
  t = t < 0 ? 0 : (t > 1 ? 1 : t);
  return {uint8_t(a.r + (b.r - a.r) * t), uint8_t(a.g + (b.g - a.g) * t),
          uint8_t(a.b + (b.b - a.b) * t)};
}

constexpr Rgb BLACK{0, 0, 0};

// How far each lid comes in, in px from the panel edge, at column dx
// (-R..R). `mirror` is +1 for one eye and -1 for the other, so slanted
// lids slant toward the nose on both.
struct Lids { float upper, lower; };

inline Lids lids(const Style& st, const EyeState& s, float dx, float mirror) {
  const MoodLids& m = st.moods[int(s.mood)];
  float u = dx / R * mirror;                     // -1 .. 1 across the eye
  float up = m.up + m.tilt * u;
  float low = m.low - m.curve * u * u;
  // A blink brings both lids to meet a little below the middle.
  up += (R + 10.0f - up) * s.blink;
  low += (R - 10.0f - low) * s.blink;
  return {up, low};
}

// The color of one pixel (x, y in 0..SIZE-1) of an eye.
inline Rgb pixel(const Style& st, const EyeState& s, int x, int y, float mirror) {
  float dx = x + 0.5f - R, dy = y + 0.5f - R;
  float r2 = dx * dx + dy * dy;
  if (r2 > R * R) return BLACK;                  // outside the round glass
  Lids l = lids(st, s, dx, mirror);
  if (dy < -R + l.upper || dy > R - l.lower) return st.lid;
  float ix = dx - s.gx * st.gaze_reach, iy = dy - s.gy * st.gaze_reach;
  float d = std::sqrt(ix * ix + iy * iy);
  float pr = st.pupil_r * s.pupil;
  Rgb c;
  if (d < pr) {
    c = st.pupil;
  } else if (d < st.iris_r) {
    c = mix(st.iris_in, st.iris_out, (d - pr) / (st.iris_r - pr));
  } else {
    // the sclera darkens toward the rim: a sphere, not a disc
    c = mix(st.sclera, st.sclera_edge, std::sqrt(r2) / R - 0.55f);
  }
  float gx = ix - st.glint_dx, gy = iy - st.glint_dy;   // one light, same side on both eyes
  if (gx * gx + gy * gy < st.glint_r * st.glint_r) c = st.glint;
  if (st.lid_shadow > 0) {                       // a soft shadow under the upper lid
    float edge = dy - (-R + l.upper);
    if (edge < st.lid_shadow) c = mix(c, st.lid, 0.5f * (1.0f - edge / st.lid_shadow));
  }
  return c;
}

}  // namespace eyes
