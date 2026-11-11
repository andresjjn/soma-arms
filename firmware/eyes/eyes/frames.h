// SOMA's voice, phase 1: the framed stream over USB serial. Pure C++, no
// hardware; test/test_eyes.cpp checks it and tools/voice_relay.py speaks it.
//
// After `stream` (answered `ok`), both directions carry frames:
//
//   0xA5  TYPE  LEN_LO  LEN_HI  PAYLOAD[LEN]
//
// Board to host:  'M' microphone, 16 bit mono at AUDIO_RATE, 20 ms a frame
//                 'K' a playback frame was taken: u32 bytes queued, u32 capacity
// Host to board:  'P' speaker audio, 16 bit mono at AUDIO_RATE, at most MAX_PAYLOAD
//                 'C' one command line (look, mood, pupil, ...; `stream off` ends it)
//                 'X' drop the queued speaker audio (the model was interrupted)
//                 'N' nothing, keeps the stream alive
//
// Text the board prints between frames (replies, logs) is plain ASCII and
// never contains 0xA5, so the host skips it and resynchronizes on the next
// frame. Without a frame from the host for STREAM_TIMEOUT_MS, the board
// leaves the stream by itself: a crashed relay never leaves the microphone on.
#pragma once
#include <cstddef>
#include <cstdint>

namespace eyes {

constexpr uint8_t FRAME_MAGIC = 0xA5;
constexpr size_t FRAME_HEADER = 4;
constexpr size_t MAX_PAYLOAD = 2048;
constexpr uint32_t STREAM_TIMEOUT_MS = 3000;

inline bool host_type(uint8_t t) { return t == 'P' || t == 'C' || t == 'X' || t == 'N'; }

inline size_t frame_header(uint8_t* out, uint8_t type, size_t len) {
  out[0] = FRAME_MAGIC;
  out[1] = type;
  out[2] = uint8_t(len & 0xFF);
  out[3] = uint8_t(len >> 8);
  return FRAME_HEADER;
}

// Byte by byte: push() returns true when a whole frame is in type()/payload().
// Garbage, an unknown type or an oversized length makes it hunt for the next
// magic byte again.
class FrameReader {
 public:
  bool push(uint8_t b) {
    switch (state_) {
      case 0: if (b == FRAME_MAGIC) state_ = 1; return false;
      case 1:
        if (host_type(b)) { type_ = b; state_ = 2; }
        else state_ = b == FRAME_MAGIC ? 1 : 0;
        return false;
      case 2: len_ = b; state_ = 3; return false;
      case 3:
        len_ |= size_t(b) << 8;
        if (len_ > MAX_PAYLOAD) { state_ = 0; return false; }
        got_ = 0;
        state_ = 4;
        if (len_ == 0) { state_ = 0; return true; }
        return false;
      default:
        payload_[got_++] = b;
        if (got_ == len_) { state_ = 0; return true; }
        return false;
    }
  }
  uint8_t type() const { return type_; }
  size_t size() const { return len_; }
  const uint8_t* payload() const { return payload_; }
  void reset() { state_ = 0; }

 private:
  int state_ = 0;
  uint8_t type_ = 0;
  size_t len_ = 0, got_ = 0;
  uint8_t payload_[MAX_PAYLOAD];
};

// The speaker's queue: a ring of 16 bit samples, filled from frames by the
// serial side and drained by the audio task. One writer, one reader: each
// advances only its own 32 bit counter (atomic on the ESP32), and the
// capacity is a power of two so the counters may wrap. A flush is asked by
// the writer and done by the reader, so the tail keeps a single owner.
class SampleRing {
 public:
  SampleRing(int16_t* storage, uint32_t capacity_pow2) : buf_(storage), cap_(capacity_pow2) {}
  uint32_t queued() const { return head_ - tail_; }
  uint32_t room() const { return cap_ - queued(); }
  uint32_t capacity() const { return cap_; }
  // All or nothing: a frame never goes in half.
  bool put(const int16_t* s, uint32_t n) {
    if (n > room()) return false;
    for (uint32_t i = 0; i < n; i++) buf_[(head_ + i) & (cap_ - 1)] = s[i];
    head_ = head_ + n;
    return true;
  }
  uint32_t take(int16_t* out, uint32_t n) {
    if (flush_) { tail_ = head_; flush_ = false; }
    uint32_t q = queued(), k = n < q ? n : q;
    for (uint32_t i = 0; i < k; i++) out[i] = buf_[(tail_ + i) & (cap_ - 1)];
    tail_ = tail_ + k;
    return k;
  }
  void flush() { flush_ = true; }

 private:
  int16_t* buf_;
  uint32_t cap_;
  volatile uint32_t head_ = 0, tail_ = 0;
  volatile bool flush_ = false;
};

}  // namespace eyes
