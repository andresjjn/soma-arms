// Host preview of SOMA's eyes: the very eye_model.h the board runs, drawn
// to a contact sheet (a PPM image). Each row is a mood; the columns are the
// gaze (center, left, right, up, down) and a half blink. Each cell shows
// both eyes, mirrored as on the board.
//
//   preview OUT.ppm [KEY=VALUE ...]     KEY as in `set` (iris_r=58, colors.iris_in=#ff8800)
//
// Built and run by tools/preview.py, which also turns the PPM into a PNG.
#include <cstdio>
#include <cstring>
#include <vector>
#include "../eyes/eye_model.h"
#include "../eyes/style_default.h"

int main(int argc, char** argv) {
  using namespace eyes;
  if (argc < 2) { std::fprintf(stderr, "usage: preview OUT.ppm [KEY=VALUE ...]\n"); return 2; }
  Style st = default_style();
  for (int i = 2; i < argc; i++) {
    char buf[128];
    std::strncpy(buf, argv[i], 127); buf[127] = 0;
    char* eq = std::strchr(buf, '=');
    if (!eq) { std::fprintf(stderr, "not KEY=VALUE: %s\n", argv[i]); return 2; }
    *eq = 0;
    if (!set_param(st, buf, eq + 1)) { std::fprintf(stderr, "unknown key or bad value: %s\n", argv[i]); return 2; }
  }
  struct Col { float gx, gy, blink; };
  const Col cols[] = {{0, 0, 0}, {-1, 0, 0}, {1, 0, 0}, {0, -0.8f, 0}, {0, 0.8f, 0}, {0, 0, 0.5f}};
  const int NC = 6, GAP = 12, CELL_W = 2 * SIZE + GAP, W = NC * CELL_W + GAP, H = MOODS * (SIZE + GAP) + GAP;
  std::vector<unsigned char> img(size_t(W) * H * 3, 40);
  for (int m = 0; m < MOODS; m++)
    for (int c = 0; c < NC; c++)
      for (int e = 0; e < 2; e++) {
        EyeState s;
        s.mood = Mood(m);
        s.gx = cols[c].gx; s.gy = cols[c].gy; s.blink = cols[c].blink;
        float mirror = e == 0 ? 1.0f : -1.0f;
        int ox = GAP + c * CELL_W + e * SIZE, oy = GAP + m * (SIZE + GAP);
        for (int y = 0; y < SIZE; y++)
          for (int x = 0; x < SIZE; x++) {
            Rgb p = pixel(st, s, x, y, mirror);
            unsigned char* d = &img[(size_t(oy + y) * W + ox + x) * 3];
            d[0] = p.r; d[1] = p.g; d[2] = p.b;
          }
      }
  FILE* f = std::fopen(argv[1], "wb");
  if (!f) { std::perror(argv[1]); return 1; }
  std::fprintf(f, "P6\n%d %d\n255\n", W, H);
  std::fwrite(img.data(), 1, img.size(), f);
  std::fclose(f);
  return 0;
}
