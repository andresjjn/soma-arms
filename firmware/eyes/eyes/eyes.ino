// SOMA's eyes on a Waveshare ESP32-S3-DualEye-Touch-LCD-1.28.
//
// Two round 240x240 GC9A01A panels on one SPI bus (shared clock, data and
// D/C, one chip select each), drawn in stripes of STRIPE lines with two
// DMA buffers: while one stripe goes out over SPI the CPU computes the
// next. Each pixel comes from eye_model.h; commands arrive as lines over
// the USB serial port (protocol.h). Pins from Waveshare's own examples
// (waveshareteam/ESP32-S3-DualEye-Touch-LCD-1.28, Arduino 01_LCD_Driver).
//
// The eyes never arm or move anything: they sit off SOMA's safety path.
#include <Arduino.h>
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "freertos/semphr.h"
#include "esp_lcd_gc9a01.h"
#include "eye_model.h"
#include "protocol.h"

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
static SemaphoreHandle_t free_bufs;              // counts stripe buffers not in flight

static eyes::EyeState state;
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

static void draw_eye(int i) {
  static int k = 0;
  for (int y0 = 0; y0 < eyes::SIZE; y0 += STRIPE) {
    xSemaphoreTake(free_bufs, portMAX_DELAY);
    uint16_t* buf = stripe_buf[k];
    k ^= 1;
    for (int y = 0; y < STRIPE; y++)
      for (int x = 0; x < eyes::SIZE; x++)
        buf[y * eyes::SIZE + x] = eyes::rgb565_swapped(eyes::pixel(state, x, y0 + y, MIRROR[i]));
    esp_lcd_panel_draw_bitmap(panel[i], 0, y0, eyes::SIZE, y0 + STRIPE, buf);
  }
}

// Idle life: small saccades and blinks of their own; a command pauses it.
static void animate(uint32_t now, float dt) {
  if (idle_on && now >= idle_pause_until) {
    if (now >= next_saccade) {
      target_gx = (random(-60, 61)) / 100.0f;
      target_gy = (random(-35, 36)) / 100.0f;
      next_saccade = now + random(800, 3200);
    }
    if (now >= next_blink && !blinking) {
      blinking = true;
      blink_start = now;
      next_blink = now + random(2200, 6000);
    }
  }
  float k = fminf(1.0f, dt * 14.0f);            // fast, like a saccade
  state.gx += (target_gx - state.gx) * k;
  state.gy += (target_gy - state.gy) * k;
  if (blinking) {
    uint32_t t = now - blink_start;              // 70 ms down, 110 ms up
    if (t < 70) state.blink = t / 70.0f;
    else if (t < 180) state.blink = 1.0f - (t - 70) / 110.0f;
    else { state.blink = 0; blinking = false; }
  }
}

static void handle_line(char* line) {
  eyes::Parsed p = eyes::parse(line);
  uint32_t now = millis();
  switch (p.cmd) {
    case eyes::Cmd::Ping:    Serial.println("pong soma-eyes 0.1"); return;
    case eyes::Cmd::Look:    target_gx = p.a; target_gy = p.b; idle_pause_until = now + 3000; break;
    case eyes::Cmd::Blink:   blinking = true; blink_start = now; break;
    case eyes::Cmd::SetMood: state.mood = p.mood; break;
    case eyes::Cmd::Pupil:   state.pupil = p.a; break;
    case eyes::Cmd::Bright:  set_brightness(p.a); break;
    case eyes::Cmd::Idle:    idle_on = p.on; break;
    case eyes::Cmd::None:    Serial.print("err "); Serial.println(p.error); return;
  }
  Serial.println("ok");
}

void setup() {
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
  draw_eye(0);
  draw_eye(1);
}
