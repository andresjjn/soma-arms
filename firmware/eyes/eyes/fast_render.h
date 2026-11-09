// SOMA's eyes: the fast path. Pure C++, no hardware.
//
// eye_model.h's pixel() is the reference: clear, and slow on the board
// (two square roots and several float color mixes per pixel, 115,200
// pixels a frame for two eyes). This draws the same eye with the work
// moved out of the pixel loop:
//   - colors of the iris gradient and the sclera shading are tables,
//     rebuilt only when the design changes (prepare_style);
//   - the lids depend on the column only: 240 evaluations a frame, not
//     57,600 (prepare_frame);
//   - no square root per pixel: distances stay squared, and the few that
//     need a radius look it up.
// firmware/eyes/test checks it against pixel() on many states.
#pragma once
#include <cmath>
#include <cstdint>
#include "eye_model.h"

namespace eyes {

constexpr int R2 = int(R * R);                   // 14400

struct FastStyle {
  Style st;
  uint8_t root[R2 + 1];                          // floor(sqrt(i)), for i up to R^2
  Rgb sclera_by_r[SIZE / 2 + 1];                 // sclera color at radius r
  Rgb iris_by_d[SIZE / 2 + 1];                   // iris color at distance d from the iris center
  uint16_t sclera565[SIZE / 2 + 1], iris565[SIZE / 2 + 1];
  uint16_t lid565, pupil565, glint565;
};

inline void prepare_style(FastStyle& f, const Style& st) {
  f.st = st;
  for (int i = 0; i <= R2; i++) f.root[i] = uint8_t(std::sqrt(float(i)));
  for (int r = 0; r <= SIZE / 2; r++) {
    // radius of the pixel center that floor-roots to r, as pixel() sees it
    f.sclera_by_r[r] = mix(st.sclera, st.sclera_edge, (r + 0.5f) / R - 0.55f);
    f.sclera565[r] = rgb565_swapped(f.sclera_by_r[r]);
  }
  f.lid565 = rgb565_swapped(st.lid);
  f.pupil565 = rgb565_swapped(st.pupil);
  f.glint565 = rgb565_swapped(st.glint);
}

struct FastFrame {
  float upper[SIZE];       // the upper lid's reach per column, px (pixel()'s l.upper)
  int lid_top[SIZE];       // first open row of each column
  int lid_bottom[SIZE];    // first lid row at the bottom of each column
  float cx, cy;            // iris center, px from the top left
  float pr2, ir2, pr;      // pupil and iris radii, squared
  float gcx, gcy, gr2;     // glint center and radius squared
};

inline void prepare_frame(FastStyle& f, FastFrame& fr, const EyeState& s, float mirror) {
  const Style& st = f.st;
  for (int x = 0; x < SIZE; x++) {
    Lids l = lids(st, s, x + 0.5f - R, mirror);
    fr.upper[x] = l.upper;
    // pixel(): lid where dy < -R + upper, i.e. y + 0.5 < upper
    fr.lid_top[x] = int(std::ceil(l.upper - 0.5f));
    fr.lid_bottom[x] = int(std::floor(SIZE - l.lower - 0.5f)) + 1;
  }
  fr.cx = R + s.gx * st.gaze_reach;
  fr.cy = R + s.gy * st.gaze_reach;
  fr.pr = st.pupil_r * s.pupil;
  fr.pr2 = fr.pr * fr.pr;
  fr.ir2 = st.iris_r * st.iris_r;
  for (int d = 0; d <= SIZE / 2; d++) {          // the iris gradient for this pupil size
    float t = (d + 0.5f - fr.pr) / (st.iris_r - fr.pr);
    f.iris_by_d[d] = mix(st.iris_in, st.iris_out, t);
    f.iris565[d] = rgb565_swapped(f.iris_by_d[d]);
  }
  fr.gcx = fr.cx + st.glint_dx;
  fr.gcy = fr.cy + st.glint_dy;
  fr.gr2 = st.glint_r * st.glint_r;
}

// Rows y0 .. y0 + rows - 1, columns x0 .. x1 - 1, into out (row major,
// x1 - x0 wide), panel-ready RGB565. The window lets a gaze-only change
// redraw just the box around the iris.
inline void render_rows(const FastStyle& f, const FastFrame& fr, int y0, int rows, uint16_t* out,
                        int x0 = 0, int x1 = SIZE) {
  const Style& st = f.st;
  for (int y = y0; y < y0 + rows; y++) {
    float py = y + 0.5f, dy = py - R, iy = py - fr.cy, gy = py - fr.gcy;
    float dy2 = dy * dy, iy2 = iy * iy, gy2 = gy * gy;
    uint16_t* row = out + (y - y0) * (x1 - x0) - x0;
    for (int x = x0; x < x1; x++) {
      float px = x + 0.5f, dx = px - R;
      float r2 = dx * dx + dy2;
      if (r2 > R * R) { row[x] = 0; continue; }
      if (y < fr.lid_top[x] || y >= fr.lid_bottom[x]) { row[x] = f.lid565; continue; }
      float ix = px - fr.cx, d2 = ix * ix + iy2;
      float gx = px - fr.gcx;
      bool glint = gx * gx + gy2 < fr.gr2;
      float lid_edge = py - fr.upper[x];         // pixel(): dy - (-R + upper)
      bool shadow = st.lid_shadow > 0 && lid_edge < st.lid_shadow;
      if (!shadow) {
        if (glint) row[x] = f.glint565;
        else if (d2 < fr.pr2) row[x] = f.pupil565;
        else if (d2 < fr.ir2) row[x] = f.iris565[f.root[int(d2)]];
        else row[x] = f.sclera565[f.root[int(r2)]];
        continue;
      }
      // the few rows under the upper lid: mix toward the lid color
      Rgb c = glint ? st.glint : d2 < fr.pr2 ? st.pupil
            : d2 < fr.ir2 ? f.iris_by_d[f.root[int(d2)]] : f.sclera_by_r[f.root[int(r2)]];
      row[x] = rgb565_swapped(mix(c, st.lid, 0.5f * (1.0f - lid_edge / st.lid_shadow)));
    }
  }
}

// The box a gaze-only change touches: the iris (and its glint, which sits
// inside it) at both the old and the new gaze, a margin for rounding.
struct Box { int x0, y0, x1, y1; };
inline Box gaze_box(const Style& st, const EyeState& a, const EyeState& b) {
  float r = st.iris_r + 3.0f;
  float ax = R + a.gx * st.gaze_reach, ay = R + a.gy * st.gaze_reach;
  float bx = R + b.gx * st.gaze_reach, by = R + b.gy * st.gaze_reach;
  auto lo = [](float v) { int i = int(std::floor(v)); return i < 0 ? 0 : i; };
  auto hi = [](float v) { int i = int(std::ceil(v)); return i > SIZE ? SIZE : i; };
  return {lo(std::fmin(ax, bx) - r), lo(std::fmin(ay, by) - r), hi(std::fmax(ax, bx) + r), hi(std::fmax(ay, by) + r)};
}

}  // namespace eyes
