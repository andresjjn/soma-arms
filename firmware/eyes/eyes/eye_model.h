// SOMA's eyes: the eye as math, one pixel at a time. Pure C++, no hardware,
// so it builds on the board, in firmware/eyes/test and in the host preview
// (firmware/eyes/tools/preview.py) from the very same code.
//
// One eye is a 240x240 round GC9A01 panel. How it looks is data (style.h,
// from style.json); what it does right now is an EyeState (gaze, pupil,
// closure, mood and the blend into it). This file only turns the two into
// pixels. The design is spec v2 (2026-11-22, Andres, references in
// docs/eye_references); fast_render.h draws the same eye on the board.
//
// Coordinates: px from the eye's center (pixel centers, so x and y run
// -119.5 .. 119.5), y down. Across the eye u = x / 120 with u = -1 at the
// nose: `mirror` is +1 for the eye drawn on the picture's left (nose to
// its right) and -1 for the other, so tilted lids tilt the same way on
// both faces of the face. Gaze and the glint are NOT mirrored: both eyes
// look the same way and one light shines on both.
#pragma once
#include <cmath>
#include <cstdint>
#include "style.h"

namespace eyes {

constexpr int SIZE = 240;
constexpr float R = SIZE / 2.0f;     // panel radius, px

struct EyeState {
  float gx = 0, gy = 0;   // gaze, -1..1 (x + toward the picture's right, y + down)
  float pupil = 1.0f;     // commanded pupil scale, 0.5..1.5; the mood scales it
  float blink = 0.0f;     // closure c, 0 open .. 1 shut: a blink or sleep, whichever is more
  Mood mood = Mood::Neutral;
  // A mood change blends the lids and pupil from `from` (the shape on
  // screen when it began) into the mood's own over mood_transition ms;
  // blend 1 is the mood itself, whatever `from` holds.
  float blend = 1.0f;
  MoodLids from{};
};

// RGB565 with the bytes swapped: the panel takes the high byte first and
// the ESP32 is little endian.
inline uint16_t rgb565_swapped(Rgb c) {
  uint16_t v = uint16_t(((c.r & 0xF8) << 8) | ((c.g & 0xFC) << 3) | (c.b >> 3));
  return uint16_t((v >> 8) | (v << 8));
}

constexpr Rgb BLACK{0, 0, 0};

inline float clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

// The lids and pupil on screen: the mood's own, or a blend into it.
inline MoodLids shape(const Style& st, const EyeState& s) {
  const MoodLids& to = st.moods[int(s.mood)];
  if (s.blend >= 1.0f) return to;
  float t = clamp01(s.blend);
  auto lerp = [t](float a, float b) { return a + (b - a) * t; };
  return {lerp(s.from.up, to.up), lerp(s.from.tilt, to.tilt), lerp(s.from.arch, to.arch),
          lerp(s.from.low, to.low), lerp(s.from.curve, to.curve), lerp(s.from.pupil, to.pupil)};
}

// The pupil's radius, px: the mood's scale times the commanded one, clamped.
inline float pupil_radius(const Style& st, const MoodLids& m, float commanded) {
  float k = m.pupil * commanded;
  return st.pupil_r * (k < 0.5f ? 0.5f : (k > 1.5f ? 1.5f : k));
}

// How far across the eye a column is, px toward the outer corner: u * 120,
// so u = -1 is the nose.
inline float across(float dx, float mirror) { return -dx * mirror; }

// Where the lids' edges are in a column, y px from the center: the upper
// lid covers y <= up, the lower one y >= low. Closing (c) draws both onto
// the seam y_seam(u) = seam_y - seam_curve * u^2, the same curve for both,
// so shut lids meet on one line with their joints lined up.
//
// Evaluated scaled by 120^2, from k = u * 120 (across()): with whole
// numbers in the design every term is then an exact integer, even in
// float, and only the last division rounds. An edge that lands exactly on
// a pixel row (sad's upper lid at u = -2/3 is y = -56.0) stays exactly on
// it, as in the reference images; plain float u flipped 24 px there.
// T is double here and float on the board (fast_render.h: the ESP32-S3
// has no double FPU); the two agree but for rare edges within float
// rounding of a row, mid blend or mid blink.
template <class T> struct LidsOf { T up, low; };
using Lids = LidsOf<double>;

template <class T = double>
inline LidsOf<T> lids(const Style& st, const MoodLids& m, float blink, float across_px) {
  const T W = T(R), W2 = T(R) * T(R), k = T(across_px), k2 = k * k;
  T c = clamp01(blink);
  T seam = T(st.seam_y) * W2 - T(st.seam_curve) * k2;
  T up = (1 - c) * (T(m.up) * W2 + T(m.tilt) * W * k + T(m.arch) * k2) + c * seam;
  T low = (1 - c) * (T(m.low) * W2 - T(m.curve) * k2) + c * seam;
  up /= W2;
  low /= W2;
  return {up, low < up ? up : low};
}

// A lid at depth d px from its edge, |x| px from the eye's center: the
// tip, the metal rim, then the body with its two slanted joints (a dark
// line and a lighter bevel beside it), symmetric about the center.
inline Rgb lid_color(const Style& st, double ax, double d) {
  if (d < st.lid_tip) return st.lid_tip_color;
  if (d < st.lid_tip + st.lid_rim) return st.lid_rim_color;
  if (std::fabs(ax - (st.joint_x + double(st.joint_slope) * d)) <= st.joint_w) return st.lid_seam;
  if (std::fabs(ax - (st.bevel_x + double(st.joint_slope) * d)) <= st.bevel_w) return st.lid_seam_hi;
  return st.lid;
}

// The iris and pupil at squared distance i2 from the iris center.
inline Rgb iris_color(const Style& st, float i2, float pr, bool shade) {
  if (i2 <= pr * pr) return st.pupil;
  if (shade) return st.iris_shadow;
  if (i2 >= st.ring_limbal * st.ring_limbal) return st.iris_limbal;
  if (i2 >= st.ring_outer * st.ring_outer) return st.iris_outer;
  if (i2 >= st.ring_mid * st.ring_mid) return st.iris_mid2;
  float inner = pr + st.ring_inner_pad;
  if (i2 <= inner * inner) return st.iris_inner;
  return st.iris_mid1;
}

// The timing curves (spec v2), pure so the host tests can check them;
// eyes.ino drives them with the clock.
//
// A blink's closure t ms after it began: closes in blink_close ms along
// t^2, holds shut blink_hold ms, opens in blink_open ms along the cubic
// ease-out 1 - (1 - t)^3 of the opening (so closure (1 - t)^3). -1 once
// it is over.
inline float blink_closure(const Style& st, float t) {
  if (t < 0) return 0;
  if (t < st.blink_close) { float k = t / st.blink_close; return k * k; }
  t -= st.blink_close;
  if (t < st.blink_hold) return 1;
  t -= st.blink_hold;
  if (t < st.blink_open) { float k = 1 - t / st.blink_open; return k * k * k; }
  return -1;
}

// Sleep moves the closure at a steady rate: shut in sleep_close ms, open
// in sleep_open ms. One step of dt ms from `closed` toward `target`.
inline float sleep_step(const Style& st, float closed, float target, float dt) {
  float span = target > closed ? st.sleep_close : st.sleep_open;
  float step = span > 0 ? dt / span : 1.0f;
  if (target > closed) return closed + step >= target ? target : closed + step;
  return closed - step <= target ? target : closed - step;
}

// A mood change t ms after it began: the blend into the new mood, linear
// over mood_transition ms.
inline float mood_blend(const Style& st, float t) {
  if (st.mood_transition <= 0 || t >= st.mood_transition) return 1;
  return t <= 0 ? 0 : t / st.mood_transition;
}

// The color of one pixel (x, y in 0..SIZE-1) of an eye.
inline Rgb pixel(const Style& st, const EyeState& s, int x, int y, float mirror) {
  float dx = x - R, dy = y - R;                 // the spec's integer grid: x, y in -120..119
  float r2 = dx * dx + dy * dy;
  if (r2 > R * R) return BLACK;                  // the bezel: outside the round glass
  MoodLids m = shape(st, s);
  Lids l = lids(st, m, s.blink, across(dx, mirror));
  double ax = std::fabs(dx);
  if (dy >= l.low) return lid_color(st, ax, dy - l.low);   // the lower lid is in front
  if (dy <= l.up) return lid_color(st, ax, l.up - dy);
  bool shade = dy - l.up <= st.lid_shadow;       // the band under the upper lid, on sclera, iris and glint
  float ix = dx - s.gx * st.gaze_reach, iy = dy - s.gy * st.gaze_reach;
  float gx = ix - st.glint_dx, gy = iy - st.glint_dy;   // one light, same side on both eyes
  if (gx * gx + gy * gy <= st.glint_r * st.glint_r) return shade ? st.glint_shadow : st.glint;
  float i2 = ix * ix + iy * iy;
  if (i2 <= st.iris_r * st.iris_r) return iris_color(st, i2, pupil_radius(st, m, s.pupil), shade);
  if (r2 < st.sclera_r * st.sclera_r) return shade ? st.sclera_shadow : st.sclera;
  return st.sclera_rim;
}

}  // namespace eyes
