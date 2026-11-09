// Host tests for SOMA's eyes: the eye model and the line protocol.
// Built and run by soma_driver/test/test_eyes_firmware.py with the host
// C++ compiler; exits non-zero on the first failure.
#include <cstdio>
#include <cstring>
#include "../eyes/eye_model.h"
#include "../eyes/protocol.h"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); fails++; } } while (0)

static bool same(eyes::Rgb a, eyes::Rgb b) { return a.r == b.r && a.g == b.g && a.b == b.b; }
static eyes::Parsed parse(const char* s) { static char buf[64]; std::strncpy(buf, s, 63); buf[63] = 0; return eyes::parse(buf); }

int main() {
  using namespace eyes;
  EyeState s;

  // Geometry: outside the round glass is black, the center is the pupil.
  CHECK(same(pixel(s, 0, 0, 1), BLACK));
  CHECK(same(pixel(s, 120, 120, 1), BLACK));                 // pupil
  CHECK(!same(pixel(s, 120 + 45, 120, 1), BLACK));          // iris, not pupil
  CHECK(same(pixel(s, 120 + 95, 120, 1), mix(SCLERA, Rgb{150, 152, 160}, (95.5f / R) - 0.55f)) ||
        !same(pixel(s, 120 + 95, 120, 1), BLACK));          // sclera at the side

  // Gaze moves the pupil: looking right puts the pupil right of center.
  s.gx = 1.0f;
  CHECK(same(pixel(s, 120 + int(GAZE_REACH), 120, 1), BLACK));
  CHECK(!same(pixel(s, 120 - 20, 120, 1), BLACK));
  s.gx = 0;

  // A full blink shuts the eye: the middle row is lid.
  s.blink = 1.0f;
  CHECK(same(pixel(s, 120, 100, 1), LID));
  CHECK(same(pixel(s, 120, 140, 1), LID));
  s.blink = 0;

  // Moods change the lids, and mirrored eyes slant toward the nose.
  s.mood = Mood::Angry;
  Lids a = lids(s, -60, +1), b = lids(s, -60, -1);
  CHECK(a.upper != b.upper);
  CHECK(lids(s, 60, +1).upper == lids(s, -60, -1).upper);   // mirror images
  s.mood = Mood::Surprised;
  CHECK(lids(s, 0, 1).upper < lids(EyeState{}, 0, 1).upper);
  s.mood = Mood::Happy;
  CHECK(lids(s, 0, 1).lower > lids(EyeState{}, 0, 1).lower);

  // RGB565, bytes swapped for the panel.
  CHECK(rgb565_swapped(Rgb{255, 0, 0}) == 0x00F8);
  CHECK(rgb565_swapped(Rgb{0, 0, 255}) == 0x1F00);

  // Protocol.
  CHECK(parse("ping").cmd == Cmd::Ping);
  { Parsed p = parse("look 0.5 -0.25"); CHECK(p.cmd == Cmd::Look && p.a == 0.5f && p.b == -0.25f); }
  CHECK(parse("look 1.5 0").cmd == Cmd::None);               // out of range
  CHECK(parse("look 0.5").cmd == Cmd::None);
  CHECK(parse("look x y").cmd == Cmd::None);
  { Parsed p = parse("mood happy"); CHECK(p.cmd == Cmd::SetMood && p.mood == Mood::Happy); }
  CHECK(parse("mood furious").cmd == Cmd::None);
  CHECK(parse("pupil 1.2").cmd == Cmd::Pupil);
  CHECK(parse("pupil 3").cmd == Cmd::None);
  CHECK(parse("bright 50").cmd == Cmd::Bright);
  CHECK(parse("bright -1").cmd == Cmd::None);
  { Parsed p = parse("idle off"); CHECK(p.cmd == Cmd::Idle && !p.on); }
  CHECK(parse("idle maybe").cmd == Cmd::None);
  CHECK(parse("arm").cmd == Cmd::None);                      // nothing that sounds like motion
  CHECK(parse("").cmd == Cmd::None);
  CHECK(parse("look 0.1 0.2\r").cmd == Cmd::Look);          // a CRLF terminal

  if (fails) { std::printf("%d failures\n", fails); return 1; }
  std::printf("eyes host tests: all passed\n");
  return 0;
}
