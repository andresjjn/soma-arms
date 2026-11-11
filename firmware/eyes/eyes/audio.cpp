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

void rec(float seconds) {
  if (!ok) { Serial.println("err audio: codecs did not start"); return; }
  read_frames(size_t(rate_hz * SETTLE_S) * 4);   // stale DMA and the codec's wake up
  size_t want = size_t(rate_hz * seconds) * 4;
  size_t got = read_frames(want);
  Serial.printf("pcm %u %u 2\n", (unsigned)got, (unsigned)rate_hz);
  Serial.write(buf, got);
  Serial.println(got == want ? "ok" : "err rec: short read");
}

void play(uint32_t bytes) {
  if (!ok) { Serial.println("err audio: codecs did not start"); return; }
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

}  // namespace audio
