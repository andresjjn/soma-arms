// SOMA's voice, phase 0: codecs, I2S and the two serial transfers.
//
// Pins and codec settings from Waveshare's Arduino 03_Mic_test for this
// board (and its xiaozhi-esp32 board config): ES8311 (speaker DAC) at 0x18
// and ES7210 (microphone ADC) at 0x40, both on I2C bus 0 (SDA 11, SCL 10,
// shared with eye 0's touch controller); one I2S bus for both codecs, so
// input and output always run at the same rate; the speaker amplifier has
// its own enable on GPIO 9, kept off except while playing (no hiss, and no
// speaker in the recordings). es7210.* and es8311.* are Espressif's
// Apache-2.0 drivers as Waveshare adapted them to Arduino's Wire.
#include "audio.h"

#include <Arduino.h>
#include <ESP_I2S.h>
#include <Wire.h>
#include "esp_heap_caps.h"
#include "es7210.h"
#include "es8311.h"
#include "protocol.h"
#include "frames.h"

namespace audio {

constexpr int PIN_MCLK = 12, PIN_BCLK = 13, PIN_WS = 14, PIN_DIN = 15, PIN_DOUT = 16;
constexpr int PIN_PA = 9, PIN_SDA = 11, PIN_SCL = 10;
constexpr uint16_t ES7210_ADDR = 0x40;
constexpr int MCLK_RATIO = 256;
constexpr float SETTLE_S = 0.1f;            // dropped at the start of a recording

static I2SClass i2s;
static es8311_handle_t dac = nullptr;
static es7210_dev_handle_t adc = nullptr;
static uint32_t rate_hz = 0;
static bool ok = false;
static uint8_t* buf = nullptr;              // PSRAM, the longest recording
static size_t buf_size = 0;

bool ready() { return ok; }

bool begin(uint32_t rate) {
  pinMode(PIN_PA, OUTPUT);
  digitalWrite(PIN_PA, LOW);
  Wire.begin(PIN_SDA, PIN_SCL);
  i2s.setPins(PIN_BCLK, PIN_WS, PIN_DOUT, PIN_DIN, PIN_MCLK);
  i2s.setTimeout(100);
  if (!i2s.begin(I2S_MODE_STD, rate, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO)) return false;

  dac = es8311_create(I2C_NUM_0, ES8311_ADDRRES_0);
  if (!dac) return false;
  es8311_clock_config_t clk = {};
  clk.mclk_from_mclk_pin = true;
  clk.mclk_frequency = int(rate) * MCLK_RATIO;
  clk.sample_frequency = int(rate);
  if (es8311_init(dac, &clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16) != ESP_OK) return false;
  es8311_voice_volume_set(dac, 75, nullptr);     // Waveshare: 70 to 80 is the linear range
  es8311_microphone_config(dac, false);

  es7210_i2c_config_t bus = {};
  bus.i2c_port = I2C_NUM_0;
  bus.i2c_addr = ES7210_ADDR;
  if (es7210_new_codec(&bus, &adc) != ESP_OK) return false;
  es7210_codec_config_t cc = {};
  cc.sample_rate_hz = rate;
  cc.mclk_ratio = MCLK_RATIO;
  cc.i2s_format = ES7210_I2S_FMT_I2S;
  cc.bit_width = ES7210_I2S_BITS_16B;
  cc.mic_bias = ES7210_MIC_BIAS_2V87;
  cc.mic_gain = ES7210_MIC_GAIN_30DB;
  cc.flags.tdm_enable = true;
  if (es7210_config_codec(adc, &cc) != ESP_OK) return false;
  es7210_config_volume(adc, 10);

  buf_size = size_t(rate * eyes::AUDIO_MAX_S) * 4;   // stereo frames
  buf = (uint8_t*)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
  if (!buf) return false;
  rate_hz = rate;
  ok = true;
  return true;
}

void volume(int percent) {
  if (ok) es8311_voice_volume_set(dac, percent, nullptr);
}

static size_t read_frames(size_t bytes) {
  size_t got = 0;
  while (got < bytes) {
    size_t n = i2s.readBytes((char*)buf + got, min(bytes - got, size_t(1024)));
    if (n == 0) break;                           // timeout: the codec stopped clocking
    got += n;
  }
  return got;
}

static volatile bool stream_on = false;

void rec(float seconds) {
  if (!ok) { Serial.println("err audio: codecs did not start"); return; }
  if (stream_on) { Serial.println("err audio: streaming"); return; }
  read_frames(size_t(rate_hz * SETTLE_S) * 4);   // stale DMA and the codec's wake up
  size_t want = size_t(rate_hz * seconds) * 4;
  size_t got = read_frames(want);
  Serial.printf("pcm %u %u 2\n", (unsigned)got, (unsigned)rate_hz);
  Serial.write(buf, got);
  Serial.println(got == want ? "ok" : "err rec: short read");
}

void play(uint32_t bytes) {
  if (!ok) { Serial.println("err audio: codecs did not start"); return; }
  if (stream_on) { Serial.println("err audio: streaming"); return; }
  // Mono in the first half of the buffer, read in acknowledged blocks (the
  // same flow control as `img`: the USB serial driver drops what overflows).
  uint8_t* mono = buf;
  size_t got = 0, next_ack = eyes::AUDIO_BLOCK;
  uint32_t deadline = millis() + 5000;
  Serial.write('k');
  while (got < bytes && millis() < deadline) {
    int n = Serial.read(mono + got, min(size_t(bytes), next_ack) - got);
    if (n <= 0) { delay(1); continue; }
    got += n;
    deadline = millis() + 2000;
    if (got == next_ack && got < bytes) { Serial.write('k'); next_ack += eyes::AUDIO_BLOCK; }
  }
  if (got < bytes) { Serial.printf("err play: got %u of %u bytes\n", (unsigned)got, (unsigned)bytes); return; }
  digitalWrite(PIN_PA, HIGH);
  delay(50);
  const int16_t* in = (const int16_t*)mono;
  int16_t frame[512];                            // 256 stereo frames, the sample on both sides
  for (size_t i = 0, total = bytes / 2; i < total;) {
    size_t k = 0;
    for (; k < 256 && i < total; k++, i++) frame[2 * k] = frame[2 * k + 1] = in[i];
    size_t len = k * 4, sent = 0;
    while (sent < len) {
      size_t n = i2s.write((const uint8_t*)frame + sent, len - sent);
      if (n == 0) delay(1);
      sent += n;
    }
  }
  delay(50);                                     // let the last DMA buffer drain
  digitalWrite(PIN_PA, LOW);
  Serial.println("ok");
}

void selftest() {
  if (!ok) { Serial.println("err audio: codecs did not start"); return; }
  if (stream_on) { Serial.println("err audio: streaming"); return; }
  const size_t quiet = size_t(rate_hz / 2), loud = size_t(rate_hz) * 3 / 2;
  read_frames(size_t(rate_hz * SETTLE_S) * 4);
  size_t got = read_frames(quiet * 4);
  digitalWrite(PIN_PA, HIGH);
  delay(50);
  // Full duplex: each chunk of tone written is matched by a chunk read, so
  // the two DMA streams move together on the shared clock.
  int16_t frame[512];
  const float step = 2.0f * 3.14159265f * 1000.0f / float(rate_hz), amp = 32767.0f * 0.251f;
  size_t written = 0;
  while (written < loud) {
    size_t k = 0;
    for (; k < 256 && written < loud; k++, written++)
      frame[2 * k] = frame[2 * k + 1] = int16_t(amp * sinf(step * float(written)));
    size_t len = k * 4, sent = 0;
    while (sent < len) {
      size_t n = i2s.write((const uint8_t*)frame + sent, len - sent);
      if (n == 0) delay(1);
      sent += n;
    }
    size_t want = min(len, buf_size - got), r = 0;
    while (r < want) {
      size_t n = i2s.readBytes((char*)buf + got + r, want - r);
      if (n == 0) break;
      r += n;
    }
    got += r;
  }
  delay(50);
  digitalWrite(PIN_PA, LOW);
  Serial.printf("pcm %u %u 2\n", (unsigned)got, (unsigned)rate_hz);
  Serial.write(buf, got);
  Serial.println("ok");
}

// ---- Phase 1: the stream ----

constexpr uint32_t RING_SAMPLES = 1u << 18;      // 16.4 s at 16 kHz, 512 KB of PSRAM
constexpr size_t MIC_FRAME = 320;               // 20 ms at 16 kHz
static int16_t* ring_store = nullptr;
static eyes::SampleRing* ring = nullptr;
static TaskHandle_t mic_h = nullptr, spk_h = nullptr;
static volatile uint32_t last_host_ms = 0;
static volatile uint32_t dropped_frames = 0;

bool streaming() { return stream_on; }

// One Serial.write per frame: the USB serial driver holds its lock for the
// whole call, so a frame is never split by another task's text. The mic task
// (core 0) and the acks (core 1) share the assembly buffer, so it has its own
// lock: without it the two cores wrote over each other (2026-11-10, frames
// came out mixed).
static SemaphoreHandle_t frame_lock = nullptr;
static void send_frame(uint8_t type, const void* payload, size_t len) {
  static uint8_t out[eyes::FRAME_HEADER + eyes::MAX_PAYLOAD];
  xSemaphoreTake(frame_lock, portMAX_DELAY);
  eyes::frame_header(out, type, len);
  memcpy(out + eyes::FRAME_HEADER, payload, len);
  Serial.write(out, eyes::FRAME_HEADER + len);
  xSemaphoreGive(frame_lock);
}

static void mic_task(void*) {
  int16_t stereo[MIC_FRAME * 2], mono[MIC_FRAME];
  while (stream_on) {
    size_t want = sizeof(stereo), got = 0;
    while (got < want && stream_on) {
      size_t n = i2s.readBytes((char*)stereo + got, want - got);
      if (n == 0) break;
      got += n;
    }
    size_t frames = got / 4;
    for (size_t i = 0; i < frames; i++) mono[i] = stereo[2 * i];   // channel 0 is the microphone
    if (frames) send_frame('M', mono, frames * 2);
  }
  mic_h = nullptr;
  vTaskDelete(nullptr);
}

static void speaker_task(void*) {
  int16_t mono[256], stereo[512];
  while (stream_on) {
    uint32_t k = ring->take(mono, 256);
    for (uint32_t i = 0; i < 256; i++) stereo[2 * i] = stereo[2 * i + 1] = i < k ? mono[i] : 0;
    // Silence when the queue is empty: the I2S clock and the pace stay the same.
    size_t sent = 0;
    while (sent < sizeof(stereo) && stream_on) {
      size_t n = i2s.write((const uint8_t*)stereo + sent, sizeof(stereo) - sent);
      if (n == 0) delay(1);
      sent += n;
    }
  }
  spk_h = nullptr;
  vTaskDelete(nullptr);
}

bool stream_begin() {
  if (!ok) return false;
  if (stream_on) return true;
  if (!frame_lock) frame_lock = xSemaphoreCreateMutex();
  if (!ring_store) {
    ring_store = (int16_t*)heap_caps_malloc(RING_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!ring_store) return false;
    ring = new eyes::SampleRing(ring_store, RING_SAMPLES);
  }
  ring->flush();
  dropped_frames = 0;
  last_host_ms = millis();
  stream_on = true;
  digitalWrite(PIN_PA, HIGH);
  xTaskCreatePinnedToCore(mic_task, "mic", 6144, nullptr, 5, &mic_h, 0);
  xTaskCreatePinnedToCore(speaker_task, "speaker", 6144, nullptr, 5, &spk_h, 0);
  return true;
}

void stream_end() {
  if (!stream_on) return;
  stream_on = false;
  for (int i = 0; i < 50 && (mic_h || spk_h); i++) delay(10);   // each task finishes its block
  digitalWrite(PIN_PA, LOW);
  Serial.printf("stream ended, %lu speaker frames dropped\n", (unsigned long)dropped_frames);
}

void stream_frame(uint8_t type, const uint8_t* payload, size_t len) {
  last_host_ms = millis();
  if (type == 'X') { ring->flush(); return; }
  if (type != 'P') return;
  static int16_t samples[eyes::MAX_PAYLOAD / 2];
  size_t n = len / 2;
  for (size_t i = 0; i < n; i++) samples[i] = int16_t(payload[2 * i] | (payload[2 * i + 1] << 8));
  if (!ring->put(samples, n)) dropped_frames = dropped_frames + 1;
  uint32_t k[2] = {ring->queued() * 2, ring->capacity() * 2};      // bytes queued, bytes of room in all
  send_frame('K', k, sizeof(k));
}

void stream_poll() {
  if (stream_on && millis() - last_host_ms > eyes::STREAM_TIMEOUT_MS) stream_end();
}

}  // namespace audio
