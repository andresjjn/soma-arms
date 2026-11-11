// Host tests for SOMA's eyes: the eye model, the design as data and the
// line protocol. Built and run by soma_driver/test/test_eyes_firmware.py
// with the host C++ compiler; exits non-zero on any failure.
#include <cstdio>
#include <cstring>
#include "../eyes/eye_model.h"
#include "../eyes/protocol.h"
#include "../eyes/style_default.h"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); fails++; } } while (0)

static bool same(eyes::Rgb a, eyes::Rgb b) { return a.r == b.r && a.g == b.g && a.b == b.b; }
// Any part of the shutter: a blade, its machined edge or a joint (2026-11-09).
static bool shutter(const eyes::Style& st, eyes::Rgb c) {
  return same(c, st.lid) || same(c, st.lid_edge_color) || same(c, st.lid_seam_color);
}
static eyes::Parsed parse(const char* s) { static char buf[96]; std::strncpy(buf, s, 95); buf[95] = 0; return eyes::parse(buf); }

int main() {
  using namespace eyes;
  const Style st = default_style();
  EyeState s;

  // Geometry: outside the round glass is black, the center is the pupil.
  CHECK(same(pixel(st, s, 0, 0, 1), BLACK));
  CHECK(same(pixel(st, s, 120, 120, 1), st.pupil));
  CHECK(!same(pixel(st, s, 120 + 45, 120, 1), st.pupil));     // iris, not pupil
  CHECK(!same(pixel(st, s, 120 + 95, 120, 1), BLACK));        // sclera at the side

  // Gaze moves the pupil.
  s.gx = 1.0f;
  CHECK(same(pixel(st, s, 120 + int(st.gaze_reach), 120, 1), st.pupil));
  CHECK(!same(pixel(st, s, 120 - 20, 120, 1), st.pupil));
  s.gx = 0;

  // A full blink shuts the eye.
  s.blink = 1.0f;
  CHECK(shutter(st, pixel(st, s, 120, 100, 1)));
  CHECK(shutter(st, pixel(st, s, 120, 140, 1)));
  CHECK(same(pixel(st, s, 120, 100, 1), st.lid_seam_color));   // a joint runs down the middle
  CHECK(same(pixel(st, s, 60, 100, 1), st.lid));                // a blade beside it
  s.blink = 0;

  // Moods change the lids; mirrored eyes slant toward the nose.
  s.mood = Mood::Angry;
  CHECK(lids(st, s, -60, +1).upper != lids(st, s, -60, -1).upper);
  CHECK(lids(st, s, 60, +1).upper == lids(st, s, -60, -1).upper);
  s.mood = Mood::Surprised;
  // Surprise opens the eye wider than neutral: with curved lids (2026-11-09)
  // neutral is already open at the center, so the corners tell.
  CHECK(lids(st, s, 0, 1).upper <= lids(st, EyeState{}, 0, 1).upper);
  CHECK(lids(st, s, 90, 1).upper < lids(st, EyeState{}, 90, 1).upper);
  CHECK(lids(st, s, -90, 1).upper < lids(st, EyeState{}, -90, 1).upper);
  s.mood = Mood::Happy;
  CHECK(lids(st, s, 0, 1).lower > lids(st, EyeState{}, 0, 1).lower);
  s.mood = Mood::Neutral;

  // RGB565, bytes swapped for the panel.
  CHECK(rgb565_swapped(Rgb{255, 0, 0}) == 0x00F8);
  CHECK(rgb565_swapped(Rgb{0, 0, 255}) == 0x1F00);

  // The design as data: set changes the picture, bad keys and values are refused.
  Style live = st;
  CHECK(set_param(live, "colors.pupil", "#ff0000"));
  CHECK(same(pixel(live, s, 120, 120, 1), Rgb{255, 0, 0}));
  CHECK(set_param(live, "iris_r", "40"));
  CHECK(live.iris_r == 40.0f);
  CHECK(set_param(live, "happy.low", "80") && live.moods[int(Mood::Happy)].low == 80.0f);
  CHECK(set_param(live, "timing_ms.blink_close", "50") && live.blink_close == 50.0f);
  CHECK(!set_param(live, "iris_rr", "40"));
  CHECK(!set_param(live, "iris_r", "big"));
  CHECK(!set_param(live, "colors.pupil", "red"));
  CHECK(!set_param(live, "happy.wide", "3"));
  CHECK(!set_param(live, "hap.low", "3"));

  // Review 2026-11-10: a design that breaks the renderer is refused, and
  // the refused value leaves the style as it was.
  CHECK(valid(st));
  {
    Style t = st;
    CHECK(!set_param(t, "iris_r", "130") && t.iris_r == st.iris_r);     // would read past root[]
    CHECK(!set_param(t, "gaze_reach", "400") && t.gaze_reach == st.gaze_reach);
    CHECK(!set_param(t, "lid_seams", "9"));
    CHECK(!set_param(t, "timing_ms.look_holds", "-1"));
    CHECK(!set_param(t, "glint_r", "-2"));
    CHECK(set_param(t, "gaze_reach", "119") && t.gaze_reach == 119.0f);
    Style bad = st; bad.iris_r = 130.0f;
    CHECK(!valid(bad));                                           // what the board checks after NVS
  }

  // `style` prints JSON that carries what `set` changed.
  static char json[1400];
  int n = dump_json(live, json, sizeof(json));
  CHECK(n > 200 && n < int(sizeof(json)));
  CHECK(std::strstr(json, "\"iris_r\":40") != nullptr);
  CHECK(std::strstr(json, "\"pupil\":\"#ff0000\"") != nullptr);
  CHECK(std::strstr(json, "\"happy\":{\"up\":14,\"tilt\":0,\"arch\":22,\"low\":80,\"curve\":40}") != nullptr);
  CHECK(set_param(live, "neutral.arch", "30") && live.moods[int(Mood::Neutral)].arch == 30.0f);

  // Protocol.
  CHECK(parse("ping").cmd == Cmd::Ping);
  { Parsed p = parse("look 0.5 -0.25"); CHECK(p.cmd == Cmd::Look && p.a == 0.5f && p.b == -0.25f); }
  CHECK(parse("look 1.5 0").cmd == Cmd::None);
  CHECK(parse("look 0.5").cmd == Cmd::None);
  { Parsed p = parse("mood happy"); CHECK(p.cmd == Cmd::SetMood && p.mood == Mood::Happy); }
  CHECK(parse("mood furious").cmd == Cmd::None);
  CHECK(parse("pupil 1.2").cmd == Cmd::Pupil);
  CHECK(parse("bright -1").cmd == Cmd::None);
  { Parsed p = parse("idle off"); CHECK(p.cmd == Cmd::Idle && !p.on); }
  { Parsed p = parse("set colors.iris_in #ff8800"); CHECK(p.cmd == Cmd::Set && !std::strcmp(p.key, "colors.iris_in") && !std::strcmp(p.value, "#ff8800")); }
  CHECK(parse("set iris_r").cmd == Cmd::None);
  CHECK(parse("style").cmd == Cmd::Style);
  CHECK(parse("save").cmd == Cmd::Save);
  CHECK(parse("reset").cmd == Cmd::Reset);
  CHECK(parse("stats").cmd == Cmd::Stats);
  // Audio, phase 0 of docs/voice_plan.md.
  { Parsed p = parse("rec 3"); CHECK(p.cmd == Cmd::Rec && p.a == 3.0f); }
  CHECK(parse("rec 0").cmd == Cmd::None);
  CHECK(parse("rec 11").cmd == Cmd::None);
  CHECK(parse("rec").cmd == Cmd::None);
  { Parsed p = parse("play 32000"); CHECK(p.cmd == Cmd::Play && p.bytes == 32000); }
  CHECK(parse("play 32001").cmd == Cmd::None);               // half a sample
  CHECK(parse("play 0").cmd == Cmd::None);
  CHECK(parse("play -2").cmd == Cmd::None);
  CHECK(parse("play 2x").cmd == Cmd::None);
  CHECK(parse("play 320002").cmd == Cmd::None);              // past 10 s
  CHECK(parse("play 320000").cmd == Cmd::Play);
  CHECK(parse("selftest").cmd == Cmd::SelfTest);
  CHECK(parse("selftest now").cmd == Cmd::None);
  { Parsed p = parse("vol 70"); CHECK(p.cmd == Cmd::Vol && p.a == 70.0f); }
  CHECK(parse("vol 101").cmd == Cmd::None);
  CHECK(parse("arm").cmd == Cmd::None);                      // nothing that sounds like motion
  CHECK(parse("").cmd == Cmd::None);
  CHECK(parse("look 0.1 0.2\r").cmd == Cmd::Look);

  if (fails) { std::printf("%d failures\n", fails); return 1; }
  std::printf("eyes host tests: all passed\n");
  return 0;
}
