#include "airplay_image.h"

#ifdef USE_AIRPLAY_ARTWORK

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cinttypes>
#include <cstring>

namespace esphome::airplay {

ESPHOME_LOG_TAG(TAG, "airplay.image");

AirPlayImage::AirPlayImage(int width, int height, runtime_image::ImageFormat format, image::ImageType type,
                           image::Transparency transparency, image::Image *placeholder, bool is_big_endian,
                           size_t max_size)
    : RuntimeImage(format, type, transparency, placeholder, is_big_endian, width, height), max_size_(max_size) {}

void AirPlayImage::setup() {
  // Allocated once, so that cover art does not fragment the heap; goes to PSRAM when there is some
  RAMAllocator<uint8_t> allocator;
  this->encoded_ = allocator.allocate(this->max_size_);
  if (this->encoded_ == nullptr) {
    ESP_LOGE(TAG, "Could not allocate %zu bytes for cover art. Largest free block: %zu", this->max_size_,
             allocator.get_max_free_block_size());
    this->mark_failed();
    return;
  }
  this->disable_loop();
}

void AirPlayImage::dump_config() {
  ESP_LOGCONFIG(TAG,
                "AirPlay cover art:\n"
                "  Size: %dx%d\n"
                "  Largest image: %zu bytes",
                this->fixed_width_, this->fixed_height_, this->max_size_);
}

// THREAD CONTEXT: AirPlay task
bool AirPlayImage::artwork_begin(bool png, size_t size) {
  if (this->encoded_ == nullptr)
    return false;
  const runtime_image::ImageFormat format = this->get_format();
  if ((png && format == runtime_image::JPEG) || (!png && format == runtime_image::PNG)) {
    ESP_LOGD(TAG, "Skipping %s cover art", png ? LOG_STR_LITERAL("PNG") : LOG_STR_LITERAL("JPEG"));
    return false;
  }
  if (size > this->max_size_) {
    ESP_LOGW(TAG, "Skipping cover art of %zu bytes, larger than max_size (%zu bytes)", size, this->max_size_);
    return false;
  }
  // A whole image that was not decoded yet is replaced; one that is being decoded cannot be
  uint8_t state = this->state_.load(std::memory_order_acquire);
  do {
    if (state == STATE_DECODING) {
      ESP_LOGW(TAG, "Skipping cover art that arrived while the previous one was being decoded");
      return false;
    }
  } while (!this->state_.compare_exchange_weak(state, STATE_RECEIVING, std::memory_order_acq_rel));
  this->encoded_png_ = png;
  this->encoded_size_ = size;
  this->received_ = 0;
  return true;
}

// THREAD CONTEXT: AirPlay task
void AirPlayImage::artwork_data(const uint8_t *data, size_t length) {
  if (this->state_.load(std::memory_order_relaxed) != STATE_RECEIVING)
    return;
  length = std::min(length, this->encoded_size_ - this->received_);
  std::memcpy(this->encoded_ + this->received_, data, length);
  this->received_ += length;
}

// THREAD CONTEXT: AirPlay task
void AirPlayImage::artwork_end(bool complete) {
  if (this->state_.load(std::memory_order_relaxed) != STATE_RECEIVING)
    return;
  if (!complete || this->received_ != this->encoded_size_) {
    this->state_.store(STATE_IDLE, std::memory_order_release);
    return;
  }
  this->state_.store(STATE_READY, std::memory_order_release);
  this->enable_loop_soon_any_context();
}

// THREAD CONTEXT: AirPlay task
void AirPlayImage::artwork_clear() {
  // An image that is waiting is dropped; the main loop clears before it decodes anything newer
  uint8_t ready = STATE_READY;
  this->state_.compare_exchange_strong(ready, STATE_IDLE, std::memory_order_acq_rel);
  this->clear_requested_.store(true, std::memory_order_release);
  this->enable_loop_soon_any_context();
}

void AirPlayImage::loop() {
  if (this->clear_requested_.exchange(false, std::memory_order_acq_rel))
    this->clear_();
  uint8_t ready = STATE_READY;
  if (this->state_.compare_exchange_strong(ready, STATE_DECODING, std::memory_order_acq_rel)) {
    this->decode_();
    this->state_.store(STATE_IDLE, std::memory_order_release);
    return;  // Runs once more, in case a clear or another image arrived meanwhile
  }
  this->disable_loop();
}

void AirPlayImage::decode_() {
  const runtime_image::ImageFormat format = this->encoded_png_ ? runtime_image::PNG : runtime_image::JPEG;
  const size_t size = this->encoded_size_;
  const uint32_t start = millis();
  bool ok = this->begin_decode(size, format);
  size_t done = 0;
  while (ok && done < size) {
    const int consumed = this->feed_data(this->encoded_ + done, size - done);
    if (consumed <= 0) {
      // Below 0 is a decode error; 0 means the decoder cannot get any further with the data
      ESP_LOGE(TAG, "Cover art could not be decoded at byte %zu: %s", done,
               runtime_image::decode_error_to_string(consumed));
      ok = false;
      break;
    }
    done += consumed;
  }
  if (ok)
    ok = this->end_decode();
  if (!ok) {
    this->release();  // Ends the decode
    this->error_callback_.call();
    return;
  }
  ESP_LOGD(TAG, "Cover art decoded in %" PRIu32 " ms: %zu bytes, %dx%d", millis() - start, size, this->get_width(),
           this->get_height());
  this->update_callback_.call();
}

void AirPlayImage::clear_() {
  ESP_LOGD(TAG, "Cover art cleared");
  // The picture goes black; the buffer is kept for the next image
  if (this->buffer_ != nullptr)
    std::memset(this->buffer_, 0, this->get_buffer_size(this->buffer_width_, this->buffer_height_));
  this->clear_callback_.call();
}

}  // namespace esphome::airplay

#endif  // USE_AIRPLAY_ARTWORK
