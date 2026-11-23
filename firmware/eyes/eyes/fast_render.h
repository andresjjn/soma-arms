// SOMA's eyes: the fast path. Pure C++, no hardware.
//
// eye_model.h's pixel() is the reference: clear, and too slow on the board
// (it works out the lids for every pixel, in double, 115,200 pixels a frame
// for two eyes). This draws the same eye with the work moved out of the
// pixel loop:
//   - colors are RGB565 once per design (prepare_style);
//   - the lids depend on the column only: per frame and column, with
//     pixel()'s own expressions, every boundary down the column becomes an
//     integer row (lid edge, tip, metal rim, joint, bevel, the shadow
//     band), each checked against the same test pixel() makes, so no
//     boundary rounds the other way (prepare_frame). In float, not
//     pixel()'s double: the ESP32-S3 has a single precision FPU only, and
//     double here would cost several ms a frame (eye_model.h lids() keeps
//     the two in step);
//   - per pixel that leaves integer compares for the lids and two squared
//     distances for the glint and the iris, with no square root.
// firmware/eyes/test checks it against pixel() on many states.
#pragma once
#include <cmath>
#include <cstdint>
#include "eye_model.h"

namespace eyes {

struct FastStyle {
  Style st;
  uint16_t sclera, sclera_shadow, sclera_rim, iris_inner, iris_mid1, iris_mid2, iris_outer, iris_limbal,
      iris_shadow, pupil, glint, glint_shadow, lid, tip, rim, seam, seam_hi;
};

inline void prepare_style(FastStyle& f, const Style& st) {
  f.st = st;
  f.sclera = rgb565_swapped(st.sclera);
  f.sclera_shadow = rgb565_swapped(st.sclera_shadow);
  f.sclera_rim = rgb565_swapped(st.sclera_rim);
  f.iris_inner = rgb565_swapped(st.iris_inner);
  f.iris_mid1 = rgb565_swapped(st.iris_mid1);
  f.iris_mid2 = rgb565_swapped(st.iris_mid2);
  f.iris_outer = rgb565_swapped(st.iris_outer);
  f.iris_limbal = rgb565_swapped(st.iris_limbal);
  f.iris_shadow = rgb565_swapped(st.iris_shadow);
  f.pupil = rgb565_swapped(st.pupil);
  f.glint = rgb565_swapped(st.glint);
  f.glint_shadow = rgb565_swapped(st.glint_shadow);
  f.lid = rgb565_swapped(st.lid);
  f.tip = rgb565_swapped(st.lid_tip_color);
  f.rim = rgb565_swapped(st.lid_rim_color);
  f.seam = rgb565_swapped(st.lid_seam);
  f.seam_hi = rgb565_swapped(st.lid_seam_hi);
}

// One lid in one column, as rows dy (y - 120) measured from the lid's edge
// inward: depth k = |dy - edge row| in whole rows. A lid row at depth k is
// tip below tip_end, rim below rim_end, a joint for k in [j0, j1], a bevel
// for k in [b0, b1] (empty when the first exceeds the second), else body.
struct LidRows { int edge, tip_end, rim_end, j0, j1, b0, b1; };

struct FastFrame {
  LidRows up[SIZE], low[SIZE];   // per column: the upper lid (rows going up), the lower (going down)
  int shade_end[SIZE];           // per column: rows dy <= this are in the shadow band
  float cx, cy;                  // iris center, px from the eye center
  float pr2, inner2, ir2;        // pupil, inner ring and iris radii, squared
  float limbal2, outer2, mid2;   // ring radii, squared
  float gcx, gcy, gr2;           // glint center and radius squared
};

// The rows of one lid in a column. depth(k) is the lid's depth d in px at
// k rows in from its first row: pixel()'s own expression, in float.
template <class Depth>
inline LidRows lid_rows(const Style& st, float ax, int edge, Depth depth) {
  LidRows r{edge, 0, 0, 0, -1, 0, -1};
  const int far = 2 * SIZE;                      // deeper than any lid on the panel
  auto first = [&](auto pred, int k) {           // first k >= 0 where pred holds, from a guess
    if (k < 0) k = 0;
    if (k > far) k = far;
    while (k > 0 && pred(k - 1)) k--;
    while (k < far && !pred(k)) k++;
    return k;
  };
  // d grows by exactly 1 per row: guesses from the edge's fractional depth.
  float d0 = depth(0);
  r.tip_end = first([&](int k) { return !(depth(k) < st.lid_tip); }, int(std::ceil(st.lid_tip - d0)));
  r.rim_end = first([&](int k) { return !(depth(k) < st.lid_tip + st.lid_rim); },
                    int(std::ceil(st.lid_tip + st.lid_rim - d0)));
  // A joint and a bevel are a band of d: |ax - (x0 + slope * d)| <= w.
  auto band = [&](float x0, float w, int* a, int* b) {
    auto in = [&](int k) { return std::fabs(ax - (x0 + st.joint_slope * depth(k))) <= w; };
    if (st.joint_slope == 0) {                   // straight down: the whole column or nothing
      if (in(0)) { *a = 0; *b = far; }
      return;
    }
    float mid = (ax - x0) / st.joint_slope - d0, half = w / std::fabs(st.joint_slope);
    if (!(mid + half >= -1 && mid - half <= far + 1)) return;   // off the lid (or not a number)
    int lo = int(std::floor(mid - half)), hi = int(std::ceil(mid + half));
    if (hi > far) hi = far;
    int k = lo < 0 ? 0 : lo;
    while (k <= hi && !in(k)) k++;               // within a row or two of the guess
    if (k > hi) return;
    while (k > 0 && in(k - 1)) k--;
    int e = k;
    while (e < far && in(e + 1)) e++;
    *a = k; *b = e;
  };
  band(st.joint_x, st.joint_w, &r.j0, &r.j1);
  band(st.bevel_x, st.bevel_w, &r.b0, &r.b1);
  return r;
}

inline void prepare_frame(FastStyle& f, FastFrame& fr, const EyeState& s, float mirror) {
  const Style& st = f.st;
  MoodLids m = shape(st, s);
  for (int x = 0; x < SIZE; x++) {
    float dx = x - R, ax = std::fabs(dx);
    LidsOf<float> l = lids<float>(st, m, s.blink, across(dx, mirror));
    // pixel(): lower lid where dy >= low; upper lid where dy <= up.
    int lo = int(std::ceil(l.low));
    while (lo - 1 >= l.low) lo--;
    while (lo < l.low) lo++;
    int hi = int(std::floor(l.up));
    while (hi + 1 <= l.up) hi++;
    while (hi > l.up) hi--;
    fr.low[x] = lid_rows(st, ax, lo, [&](int k) { return float(lo + k) - l.low; });
    fr.up[x] = lid_rows(st, ax, hi, [&](int k) { return l.up - float(hi - k); });
    int sh = int(std::floor(l.up + st.lid_shadow));   // pixel(): dy - up <= lid_shadow
    while (sh + 1 - l.up <= st.lid_shadow) sh++;
    while (sh - l.up > st.lid_shadow) sh--;
    fr.shade_end[x] = sh;
  }
  fr.cx = s.gx * st.gaze_reach;
  fr.cy = s.gy * st.gaze_reach;
  float pr = pupil_radius(st, m, s.pupil), inner = pr + st.ring_inner_pad;
  fr.pr2 = pr * pr;
  fr.inner2 = inner * inner;
  fr.ir2 = st.iris_r * st.iris_r;
  fr.limbal2 = st.ring_limbal * st.ring_limbal;
  fr.outer2 = st.ring_outer * st.ring_outer;
  fr.mid2 = st.ring_mid * st.ring_mid;
  fr.gcx = fr.cx + st.glint_dx;
  fr.gcy = fr.cy + st.glint_dy;
  fr.gr2 = st.glint_r * st.glint_r;
}

inline uint16_t lid565(const FastStyle& f, const LidRows& r, int k) {
  if (k < r.tip_end) return f.tip;
  if (k < r.rim_end) return f.rim;
  if (k >= r.j0 && k <= r.j1) return f.seam;
  if (k >= r.b0 && k <= r.b1) return f.seam_hi;
  return f.lid;
}

// Rows y0 .. y0 + rows - 1, columns x0 .. x1 - 1, into out (row major,
// x1 - x0 wide), panel-ready RGB565. The window lets a gaze-only change
// redraw just the box around the iris.
inline void render_rows(const FastStyle& f, const FastFrame& fr, int y0, int rows, uint16_t* out,
                        int x0 = 0, int x1 = SIZE) {
  const int R2 = int(R * R);
  const float sclera2 = f.st.sclera_r * f.st.sclera_r;
  for (int y = y0; y < y0 + rows; y++) {
    int dy = y - int(R);
    float iy = dy - fr.cy, gy = dy - fr.gcy;
    float iy2 = iy * iy, gy2 = gy * gy;
    uint16_t* row = out + (y - y0) * (x1 - x0) - x0;
    for (int x = x0; x < x1; x++) {
      int dx = x - int(R), r2 = dx * dx + dy * dy;
      if (r2 > R2) { row[x] = 0; continue; }
      const LidRows& lo = fr.low[x];
      if (dy >= lo.edge) { row[x] = lid565(f, lo, dy - lo.edge); continue; }
      const LidRows& up = fr.up[x];
      if (dy <= up.edge) { row[x] = lid565(f, up, up.edge - dy); continue; }
      bool shade = dy <= fr.shade_end[x];
      float gx = dx - fr.gcx;
      if (gx * gx + gy2 <= fr.gr2) { row[x] = shade ? f.glint_shadow : f.glint; continue; }
      float ix = dx - fr.cx, i2 = ix * ix + iy2;
      if (i2 <= fr.ir2) {
        row[x] = i2 <= fr.pr2 ? f.pupil : shade ? f.iris_shadow : i2 >= fr.limbal2 ? f.iris_limbal
               : i2 >= fr.outer2 ? f.iris_outer : i2 >= fr.mid2 ? f.iris_mid2
               : i2 <= fr.inner2 ? f.iris_inner : f.iris_mid1;
        continue;
      }
      row[x] = float(r2) < sclera2 ? (shade ? f.sclera_shadow : f.sclera) : f.sclera_rim;
    }
  }
}

// The box a gaze-only change touches: the iris and its glint at both the
// old and the new gaze, a margin for rounding, clipped to the panel. The
// glint is counted on its own: a glint_dx past the iris edge used to
// leave stale pixels outside the box (review 2026-11-10). The box may be
// empty (the whole iris off the panel): callers check empty() first.
struct Box {
  int x0, y0, x1, y1;
  bool empty() const { return x1 <= x0 || y1 <= y0; }
};
inline Box gaze_box(const Style& st, const EyeState& a, const EyeState& b) {
  float r = std::fmax(st.iris_r, std::hypot(st.glint_dx, st.glint_dy) + st.glint_r) + 3.0f;
  float ax = R + a.gx * st.gaze_reach, ay = R + a.gy * st.gaze_reach;
  float bx = R + b.gx * st.gaze_reach, by = R + b.gy * st.gaze_reach;
  auto clip = [](float v) { int i = int(v); return i < 0 ? 0 : i > SIZE ? SIZE : i; };
  return {clip(std::floor(std::fmin(ax, bx) - r)), clip(std::floor(std::fmin(ay, by) - r)),
          clip(std::ceil(std::fmax(ax, bx) + r)), clip(std::ceil(std::fmax(ay, by) + r))};
}

}  // namespace eyes
