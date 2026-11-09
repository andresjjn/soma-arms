// The fast renderer must draw the same eye as the reference model.
// For many states (moods, gazes, blinks, pupils, both mirrors), every
// pixel of render_rows() is compared with pixel(): no channel may differ
// by more than a few levels of RGB565 rounding, except on a sliver of
// boundary pixels (a 1 px edge rounded the other way).
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../eyes/fast_render.h"
#include "../eyes/style_default.h"

using namespace eyes;

static Rgb from565(uint16_t swapped) {
  uint16_t v = uint16_t((swapped >> 8) | (swapped << 8));
  return {uint8_t((v >> 8) & 0xF8), uint8_t((v >> 3) & 0xFC), uint8_t((v << 3) & 0xF8)};
}

int main() {
  static FastStyle f;
  prepare_style(f, default_style());
  FastFrame fr;
  std::vector<uint16_t> buf(SIZE * SIZE);
  int states = 0, worst_bad = 0;
  for (int m = 0; m < MOODS; m++)
    for (float gx : {-1.0f, -0.3f, 0.0f, 0.7f})
      for (float gy : {-0.8f, 0.0f, 0.5f})
        for (float blink : {0.0f, 0.4f})
          for (float pupil : {0.6f, 1.0f, 1.4f})
            for (float mirror : {1.0f, -1.0f}) {
              EyeState s; s.mood = Mood(m); s.gx = gx; s.gy = gy; s.blink = blink; s.pupil = pupil;
              prepare_frame(f, fr, s, mirror);
              render_rows(f, fr, 0, SIZE, buf.data());
              int bad = 0;
              for (int y = 0; y < SIZE; y++)
                for (int x = 0; x < SIZE; x++) {
                  Rgb want = pixel(f.st, s, x, y, mirror), got = from565(buf[y * SIZE + x]);
                  Rgb want565 = from565(rgb565_swapped(want));
                  if (std::abs(want565.r - got.r) > 12 || std::abs(want565.g - got.g) > 12 ||
                      std::abs(want565.b - got.b) > 12) bad++;
                }
              states++;
              if (bad > worst_bad) worst_bad = bad;
            }
  // A window renders exactly what the full frame has in that box, and a
  // gaze-only change differs from the old frame only inside gaze_box().
  int window_bad = 0, outside_bad = 0;
  for (float gx : {-0.9f, 0.0f, 0.6f})
    for (float to : {-0.5f, 0.2f, 1.0f}) {
      EyeState a; a.gx = gx; a.gy = 0.2f;
      EyeState b = a; b.gx = to; b.gy = -0.3f;
      std::vector<uint16_t> old_full(SIZE * SIZE), new_full(SIZE * SIZE);
      prepare_frame(f, fr, a, 1.0f); render_rows(f, fr, 0, SIZE, old_full.data());
      prepare_frame(f, fr, b, 1.0f); render_rows(f, fr, 0, SIZE, new_full.data());
      Box bx = gaze_box(f.st, a, b);
      int w = bx.x1 - bx.x0;
      std::vector<uint16_t> win(size_t(w) * (bx.y1 - bx.y0));
      render_rows(f, fr, bx.y0, bx.y1 - bx.y0, win.data(), bx.x0, bx.x1);
      for (int y = 0; y < SIZE; y++)
        for (int x = 0; x < SIZE; x++) {
          bool in = x >= bx.x0 && x < bx.x1 && y >= bx.y0 && y < bx.y1;
          if (in && win[(y - bx.y0) * w + (x - bx.x0)] != new_full[y * SIZE + x]) window_bad++;
          if (!in && old_full[y * SIZE + x] != new_full[y * SIZE + x]) outside_bad++;
        }
    }
  std::printf("windows: %d px differ inside, %d px changed outside the gaze box\n", window_bad, outside_bad);
  if (window_bad || outside_bad) return 1;
  double worst_pct = 100.0 * worst_bad / (SIZE * SIZE);
  std::printf("fast vs reference: %d states, worst %d px (%.2f %%) beyond tolerance\n", states, worst_bad, worst_pct);
  return worst_pct < 0.5 ? 0 : 1;
}
