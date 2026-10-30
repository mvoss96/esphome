#include "airplay_image.h"

#if defined(USE_ESP32) && defined(USE_AIRPLAY_ARTWORK)

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <cinttypes>
#include <cstring>

namespace esphome::airplay {

ESPHOME_LOG_TAG(TAG, "airplay.image");

static constexpr uint32_t DECODE_TASK_STACK_SIZE = 6144;
// Same as the main loop: a long decode shares the CPU with it instead of holding it up
static constexpr UBaseType_t DECODE_TASK_PRIORITY = 1;
/// iOS clears the cover and sends the same one again a moment later; a clear only takes effect after this
static constexpr uint32_t CLEAR_DELAY_MS = 1500;
static constexpr uint32_t FNV1A_OFFSET = 2166136261u;
static constexpr uint32_t FNV1A_PRIME = 16777619u;

/// Reads the size of a JPEG (from its frame header) or PNG (from IHDR) image, as the decoders see it.
static bool read_image_size(const uint8_t *data, size_t size, bool png, int *width, int *height) {
  if (png) {
    if (size < 24 || std::memcmp(data + 12, "IHDR", 4) != 0)
      return false;
    *width = static_cast<int>(encode_uint32(data[16], data[17], data[18], data[19]));
    *height = static_cast<int>(encode_uint32(data[20], data[21], data[22], data[23]));
    return *width > 0 && *height > 0;
  }
  size_t i = 2;  // After the start of image marker
  while (i + 3 < size) {
    if (data[i] != 0xFF)
      return false;
    const uint8_t marker = data[i + 1];
    if (marker == 0xFF) {  // Fill byte
      i++;
      continue;
    }
    if (marker == 0x01 || (marker >= 0xD0 && marker <= 0xD8)) {  // Markers without a length
      i += 2;
      continue;
    }
    if (marker == 0xD9 || marker == 0xDA)  // End of image or start of scan before any frame header
      return false;
    // Start of frame: C0 to CF, except C4 (Huffman tables), C8 (reserved) and CC (arithmetic coding)
    if (marker >= 0xC0 && marker <= 0xCF && marker != 0xC4 && marker != 0xC8 && marker != 0xCC) {
      if (i + 8 >= size)
        return false;
      *height = encode_uint16(data[i + 5], data[i + 6]);
      *width = encode_uint16(data[i + 7], data[i + 8]);
      return *width > 0 && *height > 0;
    }
    i += 2 + encode_uint16(data[i + 2], data[i + 3]);
  }
  return false;
}

AirPlayImage::AirPlayImage(int width, int height, runtime_image::ImageFormat format, image::ImageType type,
                           image::Transparency transparency, image::Image *placeholder, bool is_big_endian,
                           size_t max_size)
    : image::Image(nullptr, width, height, type, transparency),
      sink_(format, type, transparency, nullptr, is_big_endian, width, height),
      placeholder_(placeholder),
      max_width_(width),
      max_height_(height),
      max_size_(max_size) {}

void AirPlayImage::setup() {
  this->frame_size_ = this->sink_.get_buffer_size(this->max_width_, this->max_height_);
  if (this->frame_size_ == 0) {
    ESP_LOGE(TAG, "Cannot decode cover art at %dx%d", this->max_width_, this->max_height_);
    this->mark_failed();
    return;
  }
  // Allocated once, so that cover art does not fragment the heap; in PSRAM when there is some
  RAMAllocator<uint8_t> allocator;
  this->encoded_ = allocator.allocate(this->max_size_);
  for (uint8_t *&frame : this->frames_) {
    frame = allocator.allocate(this->frame_size_);
    if (frame != nullptr)
      std::memset(frame, 0, this->frame_size_);
  }
  if (this->encoded_ == nullptr || this->frames_[0] == nullptr || this->frames_[1] == nullptr) {
    ESP_LOGE(TAG, "Could not allocate %zu bytes for cover art. Largest free block: %zu",
             this->max_size_ + 2 * this->frame_size_, allocator.get_max_free_block_size());
    this->mark_failed();
    return;
  }
  // A black frame until the first cover arrives, so that LVGL can be given this image from the start
  this->set_frame_(0, this->max_width_, this->max_height_);
  if (!this->task_.create(AirPlayImage::decode_task, "airplay_image", DECODE_TASK_STACK_SIZE, this,
                          DECODE_TASK_PRIORITY, false)) {
    ESP_LOGE(TAG, "Failed to create the decode task");
    this->mark_failed();
    return;
  }
  this->disable_loop();
}

void AirPlayImage::dump_config() {
  ESP_LOGCONFIG(TAG,
                "AirPlay cover art:\n"
                "  Size: %dx%d\n"
                "  Largest image: %zu bytes\n"
                "  Frame buffers: 2 x %zu bytes",
                this->max_width_, this->max_height_, this->max_size_, this->frame_size_);
}

void AirPlayImage::draw(int x, int y, display::Display *display, Color color_on, Color color_off) {
  if (!this->showing_artwork_) {
    if (this->placeholder_ != nullptr)
      this->placeholder_->draw(x, y, display, color_on, color_off);
    return;
  }
  image::Image::draw(x, y, display, color_on, color_off);
}

void AirPlayImage::set_frame_(int index, int width, int height) {
  this->data_start_ = this->frames_[index];
  this->width_ = width;
  this->height_ = height;
#ifdef USE_LVGL
  // Keeps the descriptor that LVGL was given in step; widgets redraw when their source is set again
  this->get_lv_image_dsc();
#endif
}

// THREAD CONTEXT: AirPlay task
bool AirPlayImage::artwork_begin(bool png, size_t size) {
  if (this->encoded_ == nullptr)
    return false;
  const runtime_image::ImageFormat format = this->sink_.get_format();
  if ((png && format == runtime_image::JPEG) || (!png && format == runtime_image::PNG)) {
    ESP_LOGD(TAG, "Skipping %s cover art", png ? LOG_STR_LITERAL("PNG") : LOG_STR_LITERAL("JPEG"));
    return false;
  }
  if (size > this->max_size_) {
    ESP_LOGW(TAG, "Skipping cover art of %zu bytes, larger than max_size (%zu bytes)", size, this->max_size_);
    return false;
  }
  // An image that waits for the decode task is replaced; one that is being decoded cannot be
  uint8_t state = this->encoded_state_.load(std::memory_order_acquire);
  do {
    if (state == ENCODED_DECODING) {
      ESP_LOGW(TAG, "Skipping cover art that arrived while the previous one was being decoded");
      return false;
    }
  } while (!this->encoded_state_.compare_exchange_weak(state, ENCODED_RECEIVING, std::memory_order_acq_rel));
  this->encoded_png_ = png;
  this->encoded_size_ = size;
  this->received_ = 0;
  this->encoded_hash_ = FNV1A_OFFSET;
  this->encoded_generation_ = this->generation_.load(std::memory_order_acquire);
  // The new image replaces the picture, so a clear that has not taken effect yet is dropped
  this->clear_pending_.store(false, std::memory_order_release);
  return true;
}

// THREAD CONTEXT: AirPlay task
void AirPlayImage::artwork_data(const uint8_t *data, size_t length) {
  if (this->encoded_state_.load(std::memory_order_relaxed) != ENCODED_RECEIVING)
    return;
  length = std::min(length, this->encoded_size_ - this->received_);
  std::memcpy(this->encoded_ + this->received_, data, length);
  this->received_ += length;
  uint32_t hash = this->encoded_hash_;
  for (size_t i = 0; i < length; i++)
    hash = (hash ^ data[i]) * FNV1A_PRIME;
  this->encoded_hash_ = hash;
}

// THREAD CONTEXT: AirPlay task
void AirPlayImage::artwork_end(bool complete) {
  if (this->encoded_state_.load(std::memory_order_relaxed) != ENCODED_RECEIVING)
    return;
  if (!complete || this->received_ != this->encoded_size_) {
    this->encoded_state_.store(ENCODED_IDLE, std::memory_order_release);
    return;
  }
  if (this->shown_size_.load(std::memory_order_acquire) == this->encoded_size_ &&
      this->shown_hash_.load(std::memory_order_acquire) == this->encoded_hash_) {
    ESP_LOGD(TAG, "Same cover art as on screen");
    this->encoded_state_.store(ENCODED_IDLE, std::memory_order_release);
    return;
  }
  this->encoded_state_.store(ENCODED_READY, std::memory_order_release);
  xTaskNotifyGive(this->task_.get_handle());
}

// THREAD CONTEXT: AirPlay task
void AirPlayImage::artwork_clear() {
  // A waiting image is dropped, and one that is being decoded will not be shown
  uint8_t ready = ENCODED_READY;
  this->encoded_state_.compare_exchange_strong(ready, ENCODED_IDLE, std::memory_order_acq_rel);
  this->generation_.fetch_add(1, std::memory_order_acq_rel);
  this->clear_requested_ms_.store(millis(), std::memory_order_relaxed);
  this->clear_pending_.store(true, std::memory_order_release);
  this->enable_loop_soon_any_context();
}

// THREAD CONTEXT: decode task
void AirPlayImage::decode_task(void *params) {
  auto *self = static_cast<AirPlayImage *>(params);
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // The back frame is free once the main loop has put the last picture on screen; it wakes this task then
    if (self->decoded_.load(std::memory_order_acquire))
      continue;
    uint8_t ready = ENCODED_READY;
    if (!self->encoded_state_.compare_exchange_strong(ready, ENCODED_DECODING, std::memory_order_acq_rel))
      continue;
    const bool ok = self->decode_();
    self->decoded_generation_ = self->encoded_generation_;
    self->decoded_hash_ = self->encoded_hash_;
    self->decoded_size_ = static_cast<uint32_t>(self->encoded_size_);
    self->encoded_state_.store(ENCODED_IDLE, std::memory_order_release);
    if (ok) {
      self->decoded_.store(true, std::memory_order_release);
    } else {
      self->decode_failed_.store(true, std::memory_order_release);
    }
    self->enable_loop_soon_any_context();
  }
}

// THREAD CONTEXT: decode task
bool AirPlayImage::decode_() {
  const size_t size = this->encoded_size_;
  int image_width;
  int image_height;
  if (!read_image_size(this->encoded_, size, this->encoded_png_, &image_width, &image_height)) {
    ESP_LOGE(TAG, "Cover art has no readable image size");
    return false;
  }
  // Scaled to fit and keeping its aspect ratio, as RuntimeImage::resize() computes it
  const float scale = std::min(static_cast<float>(this->max_width_) / image_width,
                               static_cast<float>(this->max_height_) / image_height);
  const int width = static_cast<int>(image_width * scale);
  const int height = static_cast<int>(image_height * scale);

  uint8_t *target = this->back_frame_();
  if (!this->sink_.set_external_buffer(target, width, height))
    return false;
  const uint32_t start = millis();
  bool ok = this->sink_.begin_decode(size, this->encoded_png_ ? runtime_image::PNG : runtime_image::JPEG);
  size_t done = 0;
  while (ok && done < size) {
    const int consumed = this->sink_.feed_data(this->encoded_ + done, size - done);
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
    ok = this->sink_.end_decode() && this->sink_.decoded_into(target);
  // Lets go of the frame and ends a failed decode; the decoder is kept for the next image
  this->sink_.release();
  if (!ok)
    return false;
  this->decoded_width_ = width;
  this->decoded_height_ = height;
  ESP_LOGD(TAG, "Cover art decoded in %" PRIu32 " ms: %zu bytes, %dx%d to %dx%d", millis() - start, size, image_width,
           image_height, width, height);
  return true;
}

void AirPlayImage::loop() {
  bool waiting = false;
  if (this->clear_pending_.load(std::memory_order_acquire)) {
    const uint32_t now = App.get_loop_component_start_time();
    if (now - this->clear_requested_ms_.load(std::memory_order_relaxed) < CLEAR_DELAY_MS) {
      waiting = true;
    } else if (this->clear_pending_.exchange(false, std::memory_order_acq_rel)) {
      ESP_LOGD(TAG, "Cover art cleared");
      // The picture on screen goes black; drawing shows the placeholder
      std::memset(this->frames_[this->front_], 0, this->frame_size_);
      this->showing_artwork_ = false;
      this->shown_size_.store(0, std::memory_order_release);
      this->shown_hash_.store(0, std::memory_order_release);
      this->clear_callback_.call();
    }
  }
  if (this->decode_failed_.exchange(false, std::memory_order_acq_rel))
    this->error_callback_.call();
  if (this->decoded_.load(std::memory_order_acquire)) {
    // A picture that was decoded before a clear is dropped
    if (this->decoded_generation_ == this->generation_.load(std::memory_order_acquire)) {
      this->front_ ^= 1;
      this->set_frame_(this->front_, this->decoded_width_, this->decoded_height_);
      this->showing_artwork_ = true;
      this->shown_hash_.store(this->decoded_hash_, std::memory_order_release);
      this->shown_size_.store(this->decoded_size_, std::memory_order_release);
      this->update_callback_.call();
    }
    this->decoded_.store(false, std::memory_order_release);
    xTaskNotifyGive(this->task_.get_handle());  // An image may be waiting for the back frame
    return;
  }
  if (!waiting)
    this->disable_loop();
}

}  // namespace esphome::airplay

#endif  // USE_ESP32 && USE_AIRPLAY_ARTWORK
