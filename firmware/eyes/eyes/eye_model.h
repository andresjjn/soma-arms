// SOMA's eyes: the eye as math, one pixel at a time. Pure C++, no hardware,
// so firmware/eyes/test builds and checks it on any machine.
//
// One eye is a 240x240 round GC9A01 panel. A handful of parameters
// (gaze, pupil, blink, mood) decide every pixel, so the gaze moves
// continuously instead of stepping between sprites.
#pragma once
#include <cmath>
#include <cstdint>

namespace eyes {

constexpr int SIZE = 240;
constexpr float R = SIZE / 2.0f;     // panel radius, px
constexpr float IRIS_R = 62.0f;
constexpr float PUPIL_R = 26.0f;
constexpr float GAZE_REACH = 46.0f;  // iris center travel at |gaze| = 1, px

enum class Mood : uint8_t { Neutral, Happy, Sad, Angry, Surprised, Sleepy };

struct EyeState {
  float gx = 0, gy = 0;   // gaze, -1..1 (x + toward the picture's right, y + down)
  float pupil = 1.0f;     // pupil scale, 0.5..1.5
  float blink = 0.0f;     // 0 open .. 1 shut
  Mood mood = Mood::Neutral;
};

struct Rgb { uint8_t r, g, b; };

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
constexpr Rgb LID{18, 18, 24};
constexpr Rgb SCLERA{232, 234, 236};
constexpr Rgb IRIS_IN{40, 210, 235};
constexpr Rgb IRIS_OUT{10, 90, 140};
constexpr Rgb GLINT{255, 255, 255};

// How far each lid comes in, in px from the panel edge toward the center,
// at a pixel column dx (-R..R). `mirror` is +1 for one eye and -1 for the
// other, so slanted lids slant toward the nose on both.
struct Lids { float upper, lower; };

inline Lids lids(const EyeState& s, float dx, float mirror) {
  float up = 22.0f, low = 18.0f;                 // a resting eye shows a little lid
  float u = dx / R * mirror;                     // -1 .. 1 across the eye
  switch (s.mood) {
    case Mood::Happy:     low = 70.0f - 40.0f * u * u; break;   // cheeks push the lower lid up, curved
    case Mood::Sad:       up = 40.0f + 30.0f * u; break;        // outer corner droops
    case Mood::Angry:     up = 40.0f - 34.0f * u; break;        // inner corner drops
    case Mood::Surprised: up = 4.0f; low = 4.0f; break;
    case Mood::Sleepy:    up = 92.0f; break;
    case Mood::Neutral:   break;
  }
  // A blink brings both lids to meet a little below the middle.
  up += (R + 10.0f - up) * s.blink;
  low += (R - 10.0f - low) * s.blink;
  return {up, low};
}

// The color of one pixel (x, y in 0..SIZE-1) of an eye.
inline Rgb pixel(const EyeState& s, int x, int y, float mirror) {
  float dx = x + 0.5f - R, dy = y + 0.5f - R;
  float r2 = dx * dx + dy * dy;
  if (r2 > R * R) return BLACK;                  // outside the round glass
  Lids l = lids(s, dx, mirror);
  if (dy < -R + l.upper || dy > R - l.lower) return LID;
  float cx = s.gx * GAZE_REACH, cy = s.gy * GAZE_REACH;
  float ix = dx - cx, iy = dy - cy;
  float d = std::sqrt(ix * ix + iy * iy);
  float pr = PUPIL_R * s.pupil;
  Rgb c;
  if (d < pr) {
    c = BLACK;
  } else if (d < IRIS_R) {
    c = mix(IRIS_IN, IRIS_OUT, (d - pr) / (IRIS_R - pr));
  } else {
    // the sclera darkens toward the rim: a sphere, not a disc
    c = mix(SCLERA, Rgb{150, 152, 160}, std::sqrt(r2) / R - 0.55f);
  }
  float gx = ix + 22.0f, gy = iy + 22.0f;        // the glint sits up and to one side
  if (gx * gx + gy * gy < 81.0f) c = GLINT;
  // a soft shadow under the upper lid
  float edge = dy - (-R + l.upper);
  if (edge < 10.0f) c = mix(c, LID, 0.5f * (1.0f - edge / 10.0f));
  return c;
}

}  // namespace eyes
