// Host tests for SOMA's eyes: the eye model, the design as data and the
// line protocol. Built and run by soma_driver/test/test_eyes_firmware.py
// with the host C++ compiler; exits non-zero on any failure.
#include <cmath>
#include <cstdio>
#include <cstring>
#include "../eyes/eye_model.h"
#include "../eyes/protocol.h"
#include "../eyes/style_default.h"
#include "../eyes/frames.h"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); fails++; } } while (0)

static bool same(eyes::Rgb a, eyes::Rgb b) { return a.r == b.r && a.g == b.g && a.b == b.b; }
// Any part of a lid: tip, metal rim, body, joint or bevel (spec v2, 2026-11-22).
static bool shutter(const eyes::Style& st, eyes::Rgb c) {
  return same(c, st.lid) || same(c, st.lid_tip_color) || same(c, st.lid_rim_color) || same(c, st.lid_seam) ||
         same(c, st.lid_seam_hi);
}
static eyes::Parsed parse(const char* s) { static char buf[96]; std::strncpy(buf, s, 95); buf[95] = 0; return eyes::parse(buf); }

int main() {
  using namespace eyes;
  const Style st = default_style();
  EyeState s;

  // Spec v2 (2026-11-22). The grid is the spec's: pixel (x, y) sits at
  // (x - 120, y - 120) from the eye's center. All of these were checked
  // against docs/eye_references, which the default design reproduces pixel
  // for pixel (the 9 states, both eyes) but for 8 px of sad whose lid edge
  // lands exactly on a row (see eye_model.h lids()).
  auto at = [&](const EyeState& e, int dx, int dy, float mirror = 1.0f) { return pixel(st, e, 120 + dx, 120 + dy, mirror); };

  // The bezel, the sclera and its rim, the iris rings and the pupil.
  CHECK(same(pixel(st, s, 0, 0, 1), BLACK));
  CHECK(same(at(s, 0, -120), BLACK) == false);                  // r = 120 is still glass
  CHECK(same(at(s, 85, 85), BLACK));                            // r = 120.2: outside
  CHECK(same(at(s, 0, 0), st.pupil));
  CHECK(same(at(s, 25, 0), st.pupil) && !same(at(s, 27, 0), st.pupil));   // pupil r 26
  CHECK(same(at(s, 29, 0), st.iris_inner) && same(at(s, 31, 0), st.iris_mid1));   // inner up to 26 + 4
  CHECK(same(at(s, 35, 0), st.iris_mid1) && same(at(s, 36, 0), st.iris_mid2));
  CHECK(same(at(s, 46, 0), st.iris_mid2) && same(at(s, 47, 0), st.iris_outer));
  CHECK(same(at(s, 57, 0), st.iris_outer) && same(at(s, 58, 0), st.iris_limbal));
  CHECK(same(at(s, 62, 0), st.iris_limbal) && same(at(s, 63, 0), st.sclera));
  CHECK(same(at(s, 115, 0), st.sclera) && same(at(s, 116, 0), st.sclera_rim) && same(at(s, 119, 0), st.sclera_rim));
  CHECK(same(at(s, -22, -22), st.glint) && same(at(s, -22, -22, -1), st.glint));   // the glint is not mirrored

  // Gaze moves the iris, pupil and glint together, on both eyes alike.
  {
    EyeState g; g.gx = 1.0f; g.gy = -0.5f;
    CHECK(same(at(g, 46, -23), st.pupil) && same(at(g, 46, -23, -1), st.pupil));
    CHECK(same(at(g, 46 - 22, -23 - 22), st.glint));
    CHECK(!same(at(g, 0, 0), st.pupil));
  }

  // The pupil: the mood's scale times the commanded one, clamped to 0.5..1.5.
  {
    EyeState e; e.mood = Mood::Surprised;                         // 0.65: r 16.9
    CHECK(same(at(e, 16, 0), st.pupil) && !same(at(e, 17, 0), st.pupil));
    e.pupil = 1.3f;                                               // 0.845: r 21.97
    CHECK(same(at(e, 21, 0), st.pupil) && !same(at(e, 22, 0), st.pupil));
    e.mood = Mood::Sad; e.pupil = 1.5f;                           // 1.8, clamped to 1.5: r 39
    CHECK(same(at(e, 39, 0), st.pupil) && !same(at(e, 40, 0), st.pupil));
    e.mood = Mood::Angry; e.pupil = 0.5f;                         // 0.375, clamped to 0.5: r 13
    CHECK(same(at(e, 13, 0), st.pupil) && !same(at(e, 14, 0), st.pupil));
  }

  // The lids: u = -1 is the nose. On the eye drawn on the picture's left
  // (mirror +1) the nose is to its right, as in the references: angry's
  // upper lid drops toward the nose, sad's toward the outer corner.
  {
    EyeState e; e.mood = Mood::Angry;
    CHECK(lids(st, st.moods[int(Mood::Angry)], 0, across(100, 1)).up > lids(st, st.moods[int(Mood::Angry)], 0, across(-100, 1)).up);
    CHECK(lids(st, st.moods[int(Mood::Angry)], 0, across(-100, -1)).up > lids(st, st.moods[int(Mood::Angry)], 0, across(100, -1)).up);
    CHECK(lids(st, st.moods[int(Mood::Sad)], 0, across(-100, 1)).up > lids(st, st.moods[int(Mood::Sad)], 0, across(100, 1)).up);
    // Angry's upper lid at the nose side of the left eye (dx 100: u -5/6) sits
    // at -24 + 34 * 5/6 + 8 * 25/36 = 9.89: lid down to row 9, open from 10.
    CHECK(shutter(st, at(e, 100, 9)) && !shutter(st, at(e, 100, 10)));
  }
  // Across a lid: 1 px tip, 4 px metal rim, then the body with its joints.
  // Neutral at dx 90: the upper edge at y = -62 + 20 * 0.5625 = -50.75.
  CHECK(same(at(s, 90, -51), st.lid_tip_color) && same(at(s, 90, -50), st.sclera_shadow));
  CHECK(same(at(s, 90, -52), st.lid_rim_color) && same(at(s, 90, -55), st.lid_rim_color));
  CHECK(same(at(s, 90, -56), st.lid));
  // The lower one at the center: y = 88.
  CHECK(same(at(s, 0, 88), st.lid_tip_color) && same(at(s, 0, 89), st.lid_rim_color) && same(at(s, 0, 93), st.lid));
  CHECK(!shutter(st, at(s, 0, 87)) && same(at(s, 0, 87), st.sclera));   // no shadow above the lower lid
  // The shadow band: 9 px under the upper lid (dy - y_up <= 9), on sclera, iris and glint.
  CHECK(same(at(s, 90, -42), st.sclera_shadow) && same(at(s, 90, -41), st.sclera));
  {
    EyeState up; up.gy = -0.75f;                                  // iris center at y -34.5
    CHECK(same(at(up, 30, -55), st.iris_shadow));                 // the iris under the lid's shadow
    CHECK(same(at(up, 0, -60), st.pupil));                        // but never the pupil
    up.gx = -0.75f;                                               // the glint at (-56.5, -56.5)
    CHECK(same(at(up, -57, -57), st.glint_shadow));
  }
  // The joints: | |x| - (45.6 + 0.216 d) | <= 1.5, a bevel beside them,
  // symmetric about the center. Neutral's upper body at y = -100: d about 38.
  {
    int joints = 0, bevels = 0;
    for (int dx = -119; dx <= 119; dx++) {
      Rgb c = at(s, dx, -100);
      joints += same(c, st.lid_seam);
      bevels += same(c, st.lid_seam_hi);
      CHECK(same(c, at(s, -dx, -100)) || dx == -120);
    }
    CHECK(joints == 6 && bevels == 4);                           // 3 + 2 px on each side
    CHECK(same(at(s, 54, -100), st.lid_seam) && same(at(s, -54, -100), st.lid_seam));
  }
  // The two eyes are mirror images, glint aside: x and 240 - x swap.
  {
    Style plain = st; plain.glint_dx = 300;                       // the glint off the panel
    int differ = 0;
    for (int m = 0; m < MOODS; m++)
      for (float c : {0.0f, 0.5f}) {
        EyeState e; e.mood = Mood(m); e.blink = c;
        for (int y = 0; y < SIZE; y++)
          for (int x = 1; x < SIZE; x++)
            if (!same(pixel(plain, e, x, y, 1), pixel(plain, e, SIZE - x, y, -1))) differ++;
      }
    CHECK(differ == 0);
  }

  // Closing: c draws both lids onto the seam y = 12 - 14 u^2, in every
  // mood and on both eyes; at c = 1 no glass is left and the joints of the
  // upper and lower plates meet at |x| = 45.6.
  for (int m = 0; m < MOODS; m++)
    for (float mirror : {1.0f, -1.0f}) {
      EyeState shut; shut.mood = Mood(m); shut.blink = 1.0f;
      int open = 0;
      for (int y = 0; y < SIZE; y++)
        for (int x = 0; x < SIZE; x++) {
          Rgb c = pixel(st, shut, x, y, mirror);
          if (!same(c, BLACK) && !shutter(st, c)) open++;
        }
      CHECK(open == 0);
      for (float k : {-120.0f, -60.0f, 0.0f, 90.0f}) {
        Lids l = lids(st, st.moods[m], 1.0f, k);
        double seam = 12.0 - 14.0 * (k / 120.0) * (k / 120.0);
        CHECK(std::fabs(l.up - seam) < 1e-9 && std::fabs(l.low - seam) < 1e-9);
      }
    }
  {
    EyeState shut; shut.blink = 1.0f;
    CHECK(same(at(shut, 0, 12), st.lid_tip_color));               // the closing line at the center
    CHECK(same(at(shut, 0, 11), st.lid_rim_color) && same(at(shut, 0, 13), st.lid_rim_color));
    CHECK(same(at(shut, 0, -2), st.lid) || same(at(shut, 0, -2), st.lid_seam));
    // The seam at dx 48 is y = 9.76: the joints 10 px up and down are in the same column.
    CHECK(same(at(shut, 48, -1), st.lid_seam) && same(at(shut, 48, 20), st.lid_seam));
    CHECK(same(at(shut, -48, -1), st.lid_seam) && same(at(shut, -48, 20), st.lid_seam));
    CHECK(same(at(shut, 48, 9), st.lid_tip_color) && same(at(shut, 48, 10), st.lid_tip_color));   // a 2 px line
    // Closing is monotonic: each step of the blink covers at least what the last one did.
    int last = -1;
    for (int i = 0; i <= 20; i++) {
      EyeState b; b.blink = i / 20.0f;
      int lid = 0;
      for (int y = 0; y < SIZE; y++)
        for (int x = 0; x < SIZE; x++) lid += shutter(st, pixel(st, b, x, y, 1));
      CHECK(lid >= last);
      last = lid;
    }
  }
  // The lower lid never rises past the upper one.
  {
    MoodLids crossed{50, 0, 0, -50, 0, 1};
    Lids l = lids(st, crossed, 0, 0.0f);
    CHECK(l.low == l.up && l.up == 50);
  }

  // Mood transitions blend lids and pupil from what was on screen.
  {
    EyeState e; e.mood = Mood::Angry; e.from = st.moods[int(Mood::Neutral)]; e.blend = 0;
    MoodLids a = shape(st, e);
    CHECK(a.up == st.moods[0].up && a.tilt == 0 && a.pupil == 1.0f);
    e.blend = 0.5f;
    a = shape(st, e);
    CHECK(a.up == (-62.0f + -24.0f) / 2 && a.tilt == -17.0f && a.pupil == (1.0f + 0.75f) / 2);
    e.blend = 1.0f;
    a = shape(st, e);
    CHECK(a.up == -24.0f && a.low == 54.0f && a.pupil == 0.75f);
    CHECK(mood_blend(st, 0) == 0 && mood_blend(st, 110) == 0.5f && mood_blend(st, 220) == 1 && mood_blend(st, 5000) == 1);
  }
  // Blink and sleep timing: 75 ms down along t^2, 35 ms held, 110 ms up
  // along the cubic ease-out; sleep shuts in 950 ms and opens in 800.
  CHECK(blink_closure(st, 0) == 0);
  CHECK(std::fabs(blink_closure(st, 37.5f) - 0.25f) < 1e-6);
  CHECK(blink_closure(st, 75) == 1 && blink_closure(st, 109) == 1);
  CHECK(std::fabs(blink_closure(st, 110 + 55) - 0.125f) < 1e-6);   // (1 - 0.5)^3
  CHECK(blink_closure(st, 219) > 0 && blink_closure(st, 219) < 1e-5f);
  CHECK(blink_closure(st, 220) < 0);
  {
    float c = 0;
    for (int ms = 0; ms < 949; ms += 1) c = sleep_step(st, c, 1, 1);
    CHECK(c > 0.99f && c < 1);
    c = sleep_step(st, c, 1, 1);
    c = sleep_step(st, c, 1, 1);
    CHECK(c == 1);
    for (int ms = 0; ms < 400; ms++) c = sleep_step(st, c, 0, 1);
    CHECK(std::fabs(c - 0.5f) < 1e-3);
    CHECK(sleep_step(st, 0.3f, 0.3f, 10) == 0.3f);
  }

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
  CHECK(set_param(live, "sad.pupil", "1.1") && live.moods[int(Mood::Sad)].pupil == 1.1f);
  CHECK(set_param(live, "timing_ms.blink_close", "50") && live.blink_close == 50.0f);
  CHECK(set_param(live, "timing_ms.mood_transition", "300") && live.mood_transition == 300.0f);
  CHECK(set_param(live, "colors.lid_seam_hi", "#405060") && same(live.lid_seam_hi, Rgb{0x40, 0x50, 0x60}));
  CHECK(set_param(live, "colors.lid_rim", "#a0b0c0") && same(live.lid_rim_color, Rgb{0xa0, 0xb0, 0xc0}));
  CHECK(live.lid_rim == st.lid_rim);                              // the color, not the width
  CHECK(set_param(live, "lid_rim", "5") && live.lid_rim == 5.0f);
  CHECK(set_param(live, "joint_x", "50") && live.joint_x == 50.0f);
  CHECK(set_param(live, "seam_y", "0") && live.seam_y == 0.0f);
  CHECK(!set_param(live, "iris_rr", "40"));
  CHECK(!set_param(live, "iris_r", "big"));
  CHECK(!set_param(live, "colors.pupil", "red"));
  CHECK(!set_param(live, "colors.iris_r", "#ffffff"));
  CHECK(!set_param(live, "timing_ms.iris_r", "3"));
  CHECK(!set_param(live, "happy.wide", "3"));
  CHECK(!set_param(live, "hap.low", "3"));

  // Review 2026-11-10: a design that breaks the renderer is refused, and
  // the refused value leaves the style as it was.
  CHECK(valid(st));
  {
    Style t = st;
    CHECK(!set_param(t, "iris_r", "130") && t.iris_r == st.iris_r);
    CHECK(!set_param(t, "gaze_reach", "400") && t.gaze_reach == st.gaze_reach);
    CHECK(!set_param(t, "sclera_r", "121"));
    CHECK(!set_param(t, "lid_rim", "30"));
    CHECK(!set_param(t, "lid_tip", "-1"));
    CHECK(!set_param(t, "joint_slope", "3"));
    CHECK(!set_param(t, "seam_y", "-121"));
    CHECK(!set_param(t, "sad.pupil", "3") && t.moods[int(Mood::Sad)].pupil == st.moods[int(Mood::Sad)].pupil);
    CHECK(!set_param(t, "sad.pupil", "0.1"));
    CHECK(!set_param(t, "angry.up", "-300"));
    CHECK(set_param(t, "angry.up", "-240"));
    CHECK(!set_param(t, "timing_ms.look_holds", "-1"));
    CHECK(!set_param(t, "timing_ms.sleep_close", "70000"));
    CHECK(!set_param(t, "glint_r", "-2"));
    CHECK(set_param(t, "gaze_reach", "119") && t.gaze_reach == 119.0f);
    Style bad = st; bad.iris_r = 130.0f;
    CHECK(!valid(bad));                                           // what the board checks after NVS
    Style zero{};                                                 // a zeroed save: pupils 0, refused
    CHECK(!valid(zero));
  }

  // `style` prints JSON that carries what `set` changed.
  static char json[2400];
  int n = dump_json(live, json, sizeof(json));
  CHECK(n > 200 && n < int(sizeof(json)));
  CHECK(std::strstr(json, "\"iris_r\":40,") != nullptr);
  CHECK(std::strstr(json, "\"pupil\":\"#ff0000\"") != nullptr);
  CHECK(std::strstr(json, "\"lid_rim\":\"#a0b0c0\"") != nullptr && std::strstr(json, "\"lid_rim\":5,") != nullptr);
  CHECK(std::strstr(json, "\"happy\":{\"up\":-56,\"tilt\":-4,\"arch\":38,\"low\":80,\"curve\":-46,\"pupil\":1.15}") != nullptr);
  CHECK(std::strstr(json, "\"mood_transition\":300") != nullptr);
  {
    static char d[2400];
    int k = dump_json(st, d, sizeof(d));
    CHECK(std::strstr(d, "\"joint_x\":45.6,") != nullptr && std::strstr(d, "\"joint_slope\":0.216,") != nullptr);
    CHECK(d[k - 1] == '}' && d[k - 2] == '}');
  }
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
  CHECK(parse("stream").cmd == Cmd::Stream);
  CHECK(parse("sleep").cmd == Cmd::Sleep);
  CHECK(parse("wake").cmd == Cmd::Wake);
  CHECK(parse("wake up").cmd == Cmd::None);

  // The stream's frames (phase 1): found in garbage, split anywhere.
  {
    static FrameReader r;
    const uint8_t text[] = "ok\r\nstream ended\r\n";
    for (uint8_t b : text) CHECK(!r.push(b));                  // plain text is skipped
    uint8_t f[FRAME_HEADER + 4];
    frame_header(f, 'P', 4);
    f[4] = 1; f[5] = 2; f[6] = 3; f[7] = 4;
    int done = 0;
    for (uint8_t b : f) done += r.push(b);
    CHECK(done == 1 && r.type() == 'P' && r.size() == 4 && r.payload()[3] == 4);
    uint8_t n0[FRAME_HEADER]; frame_header(n0, 'N', 0);
    done = 0;
    for (uint8_t b : n0) done += r.push(b);
    CHECK(done == 1 && r.type() == 'N' && r.size() == 0);
    uint8_t bad[] = {FRAME_MAGIC, 'M', 4, 0};                   // a board type: not ours
    for (uint8_t b : bad) CHECK(!r.push(b));
    uint8_t big[] = {FRAME_MAGIC, 'P', 0x01, 0x10};             // 4097 bytes: refused
    for (uint8_t b : big) CHECK(!r.push(b));
    done = 0;
    uint8_t again[] = {FRAME_MAGIC, FRAME_MAGIC, 'X', 0, 0};    // magic twice, then a frame
    for (uint8_t b : again) done += r.push(b);
    CHECK(done == 1 && r.type() == 'X');
  }
  // The speaker's ring: all or nothing, wraps, flushes on the reader's side.
  {
    static int16_t store[8];
    SampleRing ring(store, 8);
    int16_t a[6] = {1, 2, 3, 4, 5, 6}, out[8] = {};
    CHECK(ring.put(a, 6) && ring.queued() == 6 && ring.room() == 2);
    CHECK(!ring.put(a, 3) && ring.queued() == 6);              // a frame never goes in half
    CHECK(ring.take(out, 4) == 4 && out[0] == 1 && out[3] == 4);
    CHECK(ring.put(a, 6));                                      // wraps past the end
    CHECK(ring.take(out, 8) == 8 && out[0] == 5 && out[1] == 6 && out[2] == 1 && out[7] == 6);
    CHECK(ring.take(out, 8) == 0);
    CHECK(ring.put(a, 5));
    ring.flush();
    CHECK(ring.take(out, 8) == 0 && ring.queued() == 0);
  }
  CHECK(parse("arm").cmd == Cmd::None);                      // nothing that sounds like motion
  CHECK(parse("").cmd == Cmd::None);
  CHECK(parse("look 0.1 0.2\r").cmd == Cmd::Look);

  if (fails) { std::printf("%d failures\n", fails); return 1; }
  std::printf("eyes host tests: all passed\n");
  return 0;
}
