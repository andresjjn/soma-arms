// SOMA's eyes on a Waveshare ESP32-S3-DualEye-Touch-LCD-1.28.
//
// Two round 240x240 GC9A01A panels on one SPI bus (shared clock, data and
// D/C, one chip select each), drawn in stripes of STRIPE lines with two
// DMA buffers: while one stripe goes out over SPI the CPU computes the
// next. Each pixel comes from eye_model.h with the design in a Style
// (style.h): the default is compiled from style.json, `set` changes it
// live, `save` keeps it in NVS across reboots. Commands arrive as lines
// over the USB serial port (protocol.h). Pins from Waveshare's own examples
// (waveshareteam/ESP32-S3-DualEye-Touch-LCD-1.28, Arduino 01_LCD_Driver).
//
// The eyes never arm or move anything: they sit off SOMA's safety path.
#include <Arduino.h>
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "freertos/semphr.h"
#include <Preferences.h>
#include "esp_lcd_gc9a01.h"
#include "fast_render.h"
#include "protocol.h"
#include "style_default.h"

// Pins (Waveshare example, Display_GC9A01.h)
constexpr int PIN_SCLK = 41, PIN_MOSI = 42, PIN_MISO = 40, PIN_DC = 45;
constexpr int PIN_CS[2] = {47, 38};
constexpr int PIN_RST[2] = {48, 8};
constexpr int PIN_BL[2] = {46, 39};
constexpr int PIXEL_CLOCK_HZ = 80 * 1000 * 1000;
constexpr int STRIPE = 24;                       // lines per DMA transfer
constexpr float MIRROR[2] = {+1.0f, -1.0f};

static esp_lcd_panel_handle_t panel[2];
static uint16_t* stripe_buf[2];
static int next_buf = 0;                     // the stripe to fill next, shared by draw_eye and img
static SemaphoreHandle_t free_bufs;              // counts stripe buffers not in flight

static eyes::Style style;
static eyes::FastStyle fast;                     // tables built from `style`
static eyes::FastFrame frame;
static bool style_dirty = true;
static uint32_t frames_drawn = 0, frames_skipped = 0, stats_since = 0;
static eyes::EyeState state;
static Preferences prefs;
static float target_gx = 0, target_gy = 0;
static bool idle_on = true;
static uint32_t idle_pause_until = 0, next_saccade = 0, next_blink = 0, blink_start = 0;
static bool blinking = false;

static bool IRAM_ATTR on_stripe_sent(esp_lcd_panel_io_handle_t, esp_lcd_panel_io_event_data_t*, void*) {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(free_bufs, &woken);
  return woken == pdTRUE;
}

static void init_panel(int i) {
  esp_lcd_panel_io_handle_t io = nullptr;
  esp_lcd_panel_io_spi_config_t io_cfg = {};
  io_cfg.cs_gpio_num = PIN_CS[i];
  io_cfg.dc_gpio_num = PIN_DC;
  io_cfg.spi_mode = 0;
  io_cfg.pclk_hz = PIXEL_CLOCK_HZ;
  io_cfg.trans_queue_depth = 4;
  io_cfg.on_color_trans_done = on_stripe_sent;
  io_cfg.lcd_cmd_bits = 8;
  io_cfg.lcd_param_bits = 8;
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io));
  esp_lcd_panel_dev_config_t dev = {};
  dev.reset_gpio_num = PIN_RST[i];
  dev.rgb_endian = LCD_RGB_ENDIAN_BGR;
  dev.bits_per_pixel = 16;
  ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io, &dev, &panel[i]));
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel[i]));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel[i]));
  ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel[i], true));
  // Waveshare's 90 degree setting, per panel (they face opposite ways on the board).
  ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel[i], i == 1, i == 1));
  ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel[i], true));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel[i], true));
}

static void set_brightness(float percent) {
  uint32_t duty = uint32_t(percent * 1023.0f / 100.0f);
  for (int i = 0; i < 2; i++) ledcWrite(PIN_BL[i], duty);
}

// Draw box b of eye i: as many rows per DMA stripe as fit a stripe buffer.
static void draw_eye(int i, eyes::Box b) {
  if (b.empty()) return;                     // nothing on the panel moved
  eyes::prepare_frame(fast, frame, state, MIRROR[i]);
  int w = b.x1 - b.x0, rows_per = (eyes::SIZE * STRIPE) / w;
  for (int y0 = b.y0; y0 < b.y1; y0 += rows_per) {
    int rows = min(rows_per, b.y1 - y0);
    xSemaphoreTake(free_bufs, portMAX_DELAY);
    uint16_t* buf = stripe_buf[next_buf];
    next_buf ^= 1;
    eyes::render_rows(fast, frame, y0, rows, buf, b.x0, b.x1);
    esp_lcd_panel_draw_bitmap(panel[i], b.x0, y0, b.x1, y0 + rows, buf);
  }
}

static bool same_state(const eyes::EyeState& a, const eyes::EyeState& b) {
  return a.gx == b.gx && a.gy == b.gy && a.pupil == b.pupil && a.blink == b.blink && a.mood == b.mood;
}

// Idle life: small saccades and blinks of their own; a command pauses it.
static void animate(uint32_t now, float dt) {
  if (idle_on && now >= idle_pause_until) {
    if (now >= next_saccade) {
      target_gx = (random(-60, 61)) / 100.0f;
      target_gy = (random(-35, 36)) / 100.0f;
      next_saccade = now + random(long(style.saccade_min), long(style.saccade_max));
    }
    if (now >= next_blink && !blinking) {
      blinking = true;
      blink_start = now;
      next_blink = now + random(long(style.blink_every_min), long(style.blink_every_max));
    }
  }
  float k = fminf(1.0f, dt * 14.0f);            // fast, like a saccade
  state.gx += (target_gx - state.gx) * k;
  state.gy += (target_gy - state.gy) * k;
  if (blinking) {
    float t = now - blink_start, c = style.blink_close, o = style.blink_open;
    if (t < c) state.blink = t / c;              // quick down, slower up, like a real one
    else if (t < c + o) state.blink = 1.0f - (t - c) / o;
    else { state.blink = 0; blinking = false; }
  }
}

// A still picture from the host, for previewing designs on the real panels:
// `img E` then SIZE*SIZE*2 bytes of RGB565, high byte first (the panel's
// order), in blocks of IMG_BLOCK: the board answers `k` after each block
// and the host waits for it. Arduino's USB-Serial/JTAG driver drops bytes
// when its receive buffer fills (2026-11-09: 45 of 115 KB arrived), so the
// blocks are the flow control. The eyes stop animating until `live`,
// `mood` or `look`.
constexpr size_t IMG_BLOCK = 4096;
static bool showing_image = false, force_redraw = false;
static uint8_t* image_buf = nullptr;

static void receive_image(int eye) {
  const size_t want = eyes::SIZE * eyes::SIZE * 2;
  if (!image_buf) image_buf = (uint8_t*)heap_caps_malloc(want, MALLOC_CAP_SPIRAM);
  if (!image_buf) { Serial.println("err img: no memory"); return; }
  size_t got = 0, next_ack = IMG_BLOCK;
  uint32_t deadline = millis() + 5000;
  Serial.write('k');                             // ready for the first block
  while (got < want && millis() < deadline) {
    int n = Serial.read(image_buf + got, min(want, next_ack) - got);
    if (n <= 0) { delay(1); continue; }
    got += n;
    deadline = millis() + 2000;
    if (got == next_ack && got < want) { Serial.write('k'); next_ack += IMG_BLOCK; }
  }
  if (got < want) { Serial.printf("err img: got %u of %u bytes\n", (unsigned)got, (unsigned)want); return; }
  showing_image = true;
  for (int y0 = 0; y0 < eyes::SIZE; y0 += STRIPE) {
    xSemaphoreTake(free_bufs, portMAX_DELAY);
    uint16_t* buf = stripe_buf[next_buf];
    next_buf ^= 1;
    memcpy(buf, image_buf + size_t(y0) * eyes::SIZE * 2, size_t(STRIPE) * eyes::SIZE * 2);
    esp_lcd_panel_draw_bitmap(panel[eye], 0, y0, eyes::SIZE, y0 + STRIPE, buf);
  }
  Serial.println("ok");
}

static void handle_line(char* line) {
  if (!strncmp(line, "img ", 4) && (line[4] == '0' || line[4] == '1') && line[5] == '\0') {
    receive_image(line[4] - '0');
    return;
  }
  if (!strcmp(line, "live")) { showing_image = false; force_redraw = true; Serial.println("ok"); return; }
  eyes::Parsed p = eyes::parse(line);
  if (p.cmd == eyes::Cmd::SetMood || p.cmd == eyes::Cmd::Look) { showing_image = false; force_redraw = true; }
  uint32_t now = millis();
  switch (p.cmd) {
    case eyes::Cmd::Ping:    Serial.println("pong soma-eyes 0.1"); return;
    case eyes::Cmd::Look:    target_gx = p.a; target_gy = p.b; idle_pause_until = now + uint32_t(style.look_holds); break;
    case eyes::Cmd::Blink:   blinking = true; blink_start = now; break;
    case eyes::Cmd::SetMood: state.mood = p.mood; break;
    case eyes::Cmd::Pupil:   state.pupil = p.a; break;
    case eyes::Cmd::Bright:  set_brightness(p.a); break;
    case eyes::Cmd::Idle:    idle_on = p.on; break;
    case eyes::Cmd::Set:
      if (!eyes::set_param(style, p.key, p.value)) { Serial.println("err set: unknown key or bad value"); return; }
      style_dirty = true;
      break;
    case eyes::Cmd::Style: {
      static char json[1400];
      eyes::dump_json(style, json, sizeof(json));
      Serial.println(json);
      return;
    }
    case eyes::Cmd::Save:    prefs.putBytes("style", &style, sizeof(style)); break;
    case eyes::Cmd::Reset:   style = eyes::default_style(); prefs.remove("style"); style_dirty = true; break;
    case eyes::Cmd::Stats: {
      float secs = (now - stats_since) / 1000.0f;
      Serial.printf("fps %.1f drawn %lu skipped %lu over %.1f s\n", frames_drawn / secs,
                    (unsigned long)frames_drawn, (unsigned long)frames_skipped, secs);
      frames_drawn = frames_skipped = 0;
      stats_since = now;
      return;
    }
    case eyes::Cmd::None:    Serial.print("err "); Serial.println(p.error); return;
  }
  Serial.println("ok");
}

void setup() {
  Serial.setRxBufferSize(16384);
  Serial.begin(115200);
  spi_bus_config_t bus = {};
  bus.mosi_io_num = PIN_MOSI;
  bus.miso_io_num = PIN_MISO;
  bus.sclk_io_num = PIN_SCLK;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = eyes::SIZE * STRIPE * sizeof(uint16_t);
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
  for (int i = 0; i < 2; i++) {
    stripe_buf[i] = (uint16_t*)heap_caps_malloc(eyes::SIZE * STRIPE * sizeof(uint16_t), MALLOC_CAP_DMA);
    ledcAttach(PIN_BL[i], 20000, 10);
  }
  free_bufs = xSemaphoreCreateCounting(2, 2);
  // The saved design, if any and if it still fits this firmware's Style.
  style = eyes::default_style();
  prefs.begin("soma-eyes", false);
  if (prefs.getBytesLength("style") == sizeof(style)) {
    prefs.getBytes("style", &style, sizeof(style));
    if (!eyes::valid(style)) style = eyes::default_style();   // a bad save must not outlive a reboot
  }
  for (int i = 0; i < 2; i++) init_panel(i);
  set_brightness(80);
  randomSeed(esp_random());
}

void loop() {
  static char line[64];
  static size_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { line[len] = '\0'; if (len) handle_line(line); len = 0; }
    else if (len < sizeof(line) - 1) line[len++] = c;
  }
  static uint32_t last = millis();
  uint32_t now = millis();
  animate(now, (now - last) / 1000.0f);
  last = now;
  // Snap the eased gaze once it is within a hair, so a still eye is still.
  if (fabsf(target_gx - state.gx) < 0.002f) state.gx = target_gx;
  if (fabsf(target_gy - state.gy) < 0.002f) state.gy = target_gy;
  static eyes::EyeState drawn;
  static bool first = true;
  if (showing_image) { delay(5); return; }
  if (force_redraw) { first = true; force_redraw = false; }
  if (style_dirty) { eyes::prepare_style(fast, style); style_dirty = false; first = true; }
  if (!first && same_state(state, drawn)) { frames_skipped++; delay(5); return; }
  // Only the gaze moved: redraw the box around the iris, old and new.
  bool gaze_only = !first && state.mood == drawn.mood && state.blink == drawn.blink &&
                   state.pupil == drawn.pupil;
  eyes::Box box = gaze_only ? eyes::gaze_box(style, drawn, state) : eyes::Box{0, 0, eyes::SIZE, eyes::SIZE};
  draw_eye(0, box);
  draw_eye(1, box);
  drawn = state;
  first = false;
  frames_drawn++;
}
