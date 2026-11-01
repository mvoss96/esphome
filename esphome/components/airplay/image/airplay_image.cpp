#include "airplay_image.h"

#if defined(USE_ESP32) && defined(USE_AIRPLAY_ARTWORK)

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <cstring>

namespace esphome::airplay_ {

ESPHOME_LOG_TAG(TAG, "airplay.image");

static constexpr uint32_t DECODE_TASK_STACK_SIZE = 6144;
// Same as the main loop: a long decode shares the CPU with it instead of holding it up
static constexpr UBaseType_t DECODE_TASK_PRIORITY = 1;
/// The encoded image passes through this buffer while it is decoded
static constexpr size_t RING_SIZE = 16 * 1024;
static_assert(RING_SIZE >= 2 * ::airplay::ARTWORK_RESERVE, "The receiver needs room beyond ARTWORK_RESERVE");
/// Most that the decode task waits for at once. Must stay below what the ring holds before the receiver stops reading
/// (RING_SIZE - ARTWORK_RESERVE), or it would wait for data that cannot come.
static constexpr size_t MAX_WAIT_BYTES = 4096;
/// An image is taken as the one on screen when its size and the hash of this many bytes from its start match
static constexpr size_t HASHED_BYTES = MAX_WAIT_BYTES;
/// PNG data is fed in pieces of at least this many bytes, the least the PNG decoder takes before the end
static constexpr size_t PNG_PIECE_SIZE = 512;
/// The decode task checks this often whether an image was cut off, in case it missed a notification
static constexpr uint32_t WAIT_CHECK_MS = 100;
/// iOS clears the cover and sends the same one again a moment later; a clear only takes effect after this
static constexpr uint32_t CLEAR_DELAY_MS = 1500;
static constexpr uint32_t FNV1A_OFFSET = 2166136261u;
static constexpr uint32_t FNV1A_PRIME = 16777619u;

/// Reads the size of a PNG image from its IHDR chunk
static bool read_png_size(const uint8_t *data, size_t size, int *width, int *height) {
  if (size < 24 || std::memcmp(data + 12, "IHDR", 4) != 0)
    return false;
  *width = static_cast<int>(encode_uint32(data[16], data[17], data[18], data[19]));
  *height = static_cast<int>(encode_uint32(data[20], data[21], data[22], data[23]));
  return *width > 0 && *height > 0;
}

// ===== JPEG decoder =====

#ifdef USE_RUNTIME_IMAGE_JPEG
bool AirPlayJpegDecoder::open(size_t size, int *width, int *height) {
  this->prepare(size);
  if (!this->jpeg_.open(this->source_, static_cast<int>(size), nullptr, read_callback, seek_callback, draw_callback)) {
    ESP_LOGE(TAG, "Cover art is no readable JPEG image: %d", this->jpeg_.getLastError());
    return false;
  }
  if (this->jpeg_.getJPEGType() != JPEG_MODE_BASELINE) {
    ESP_LOGE(TAG, "Progressive JPEG cover art is not supported");
    this->jpeg_.close();
    return false;
  }
  *width = this->jpeg_.getWidth();
  *height = this->jpeg_.getHeight();
  return *width > 0 && *height > 0;
}

bool AirPlayJpegDecoder::decode_opened() {
  this->jpeg_.setUserPointer(this);
  this->jpeg_.setPixelType(RGB8888);
  const int width = this->jpeg_.getWidth();
  const int height = this->jpeg_.getHeight();
  if (!this->set_size(width, height)) {
    this->jpeg_.close();
    return false;
  }
  // Decode at 1/2, 1/4 or 1/8 of the size while that is still at least as large as the buffer: much faster, and the
  // picture is averaged instead of sampled
  int scale = 1;
  while (scale < 8 && width / (scale * 2) >= this->image_->get_buffer_width() &&
         height / (scale * 2) >= this->image_->get_buffer_height()) {
    scale *= 2;
  }
  int options = 0;
  if (scale > 1) {
    options = scale == 2 ? JPEG_SCALE_HALF : (scale == 4 ? JPEG_SCALE_QUARTER : JPEG_SCALE_EIGHTH);
    this->x_scale_ = static_cast<double>(this->image_->get_buffer_width()) * scale / width;
    this->y_scale_ = static_cast<double>(this->image_->get_buffer_height()) * scale / height;
  }
  const bool ok = this->jpeg_.decode(0, 0, options) != 0;
  if (!ok) {
    ESP_LOGE(TAG, "Cover art could not be decoded: %d", this->jpeg_.getLastError());
  }
  this->jpeg_.close();
  return ok;
}

int32_t AirPlayJpegDecoder::read_callback(JPEGFILE *file, uint8_t *buffer, int32_t length) {
  if (length <= 0)
    return 0;
  auto *source = static_cast<AirPlayImage *>(file->fHandle);
  const size_t read = source->read_encoded_(buffer, static_cast<size_t>(length));
  file->iPos += static_cast<int32_t>(read);
  return static_cast<int32_t>(read);
}

int32_t AirPlayJpegDecoder::seek_callback(JPEGFILE *file, int32_t position) {
  // JPEGDEC only seeks forward, to skip large header segments
  auto *source = static_cast<AirPlayImage *>(file->fHandle);
  if (position >= 0 && source->skip_encoded_(static_cast<size_t>(position)))
    file->iPos = position;
  return file->iPos;
}

int AirPlayJpegDecoder::draw_callback(JPEGDRAW *draw) {
  auto *decoder = static_cast<AirPlayJpegDecoder *>(draw->pUser);
  size_t position = 0;
  for (int y = 0; y < draw->iHeight; y++) {
    for (int x = 0; x < draw->iWidth; x++) {
      // RGB8888 takes two 16-bit words per pixel
      const auto rg = decode_value(draw->pPixels[position++]);
      const auto ba = decode_value(draw->pPixels[position++]);
      decoder->draw(draw->x + x, draw->y + y, 1, 1, Color(rg[1], rg[0], ba[1], ba[0]));
    }
  }
  return 1;
}
#endif  // USE_RUNTIME_IMAGE_JPEG

// ===== Image =====

AirPlayImage::AirPlayImage(int width, int height, runtime_image::ImageFormat format, image::ImageType type,
                           image::Transparency transparency, image::Image *placeholder, bool is_big_endian)
    : image::Image(nullptr, width, height, type, transparency),
      sink_(format, type, transparency, nullptr, is_big_endian, width, height),
      placeholder_(placeholder),
      max_width_(width),
      max_height_(height) {}

void AirPlayImage::setup() {
  this->frame_size_ = this->sink_.get_buffer_size(this->max_width_, this->max_height_);
  if (this->frame_size_ == 0) {
    ESP_LOGE(TAG, "Cannot decode cover art at %dx%d", this->max_width_, this->max_height_);
    this->mark_failed();
    return;
  }
  // Allocated once, so that cover art does not fragment the heap; in PSRAM when there is some
  RAMAllocator<uint8_t> allocator;
  this->ring_ = allocator.allocate(RING_SIZE);
  for (uint8_t *&frame : this->frames_) {
    frame = allocator.allocate(this->frame_size_);
    if (frame != nullptr)
      std::memset(frame, 0, this->frame_size_);
  }
  if (this->ring_ == nullptr || this->frames_[0] == nullptr || this->frames_[1] == nullptr) {
    ESP_LOGE(TAG, "Could not allocate %zu bytes for cover art. Largest free block: %zu",
             RING_SIZE + 2 * this->frame_size_, allocator.get_max_free_block_size());
    this->mark_failed();
    return;
  }
#ifdef USE_RUNTIME_IMAGE_JPEG
  if (this->sink_.get_format() != runtime_image::PNG)
    this->jpeg_decoder_ = make_unique<AirPlayJpegDecoder>(&this->sink_, this);
#endif
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
                "  Frame buffers: 2 x %zu bytes",
                this->max_width_, this->max_height_, this->frame_size_);
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
  if (this->ring_ == nullptr || size == 0)
    return false;
  const runtime_image::ImageFormat format = this->sink_.get_format();
  if ((png && format == runtime_image::JPEG) || (!png && format == runtime_image::PNG)) {
    ESP_LOGD(TAG, "Skipping %s cover art", png ? LOG_STR_LITERAL("PNG") : LOG_STR_LITERAL("JPEG"));
    return false;
  }
  if (this->busy_.load(std::memory_order_acquire)) {
    ESP_LOGW(TAG, "Skipping cover art that arrived while the previous one was being decoded");
    return false;
  }
  this->encoded_png_ = png;
  this->encoded_size_ = size;
  this->encoded_generation_ = this->generation_.load(std::memory_order_acquire);
  this->received_.store(0, std::memory_order_relaxed);
  this->consumed_.store(0, std::memory_order_relaxed);
  this->discard_.store(false, std::memory_order_relaxed);
  this->transfer_.store(TRANSFER_RECEIVING, std::memory_order_relaxed);
  // The new image replaces the picture, so a clear that has not taken effect yet is dropped
  this->clear_pending_.store(false, std::memory_order_release);
  this->busy_.store(true, std::memory_order_release);
  xTaskNotifyGive(this->task_.get_handle());
  return true;
}

// THREAD CONTEXT: AirPlay task
void AirPlayImage::artwork_data(const uint8_t *data, size_t length) {
  if (this->transfer_.load(std::memory_order_relaxed) != TRANSFER_RECEIVING ||
      this->discard_.load(std::memory_order_acquire))
    return;
  const size_t received = this->received_.load(std::memory_order_relaxed);
  length = std::min(length, this->encoded_size_ - received);
  const size_t used = received - this->consumed_.load(std::memory_order_acquire);
  if (length > RING_SIZE - used) {
    // The receiver passed more than artwork_room() allowed
    ESP_LOGE(TAG, "Cover art buffer overflow");
    this->transfer_.store(TRANSFER_ABORTED, std::memory_order_release);
    xTaskNotifyGive(this->task_.get_handle());
    return;
  }
  const size_t offset = received % RING_SIZE;
  const size_t first = std::min(length, RING_SIZE - offset);
  std::memcpy(this->ring_ + offset, data, first);
  std::memcpy(this->ring_, data + first, length - first);
  this->received_.store(received + length, std::memory_order_release);
  xTaskNotifyGive(this->task_.get_handle());
}

// THREAD CONTEXT: AirPlay task
size_t AirPlayImage::artwork_room() const {
  if (this->transfer_.load(std::memory_order_relaxed) != TRANSFER_RECEIVING ||
      this->discard_.load(std::memory_order_acquire))
    return SIZE_MAX;
  return RING_SIZE -
         (this->received_.load(std::memory_order_relaxed) - this->consumed_.load(std::memory_order_acquire));
}

// THREAD CONTEXT: AirPlay task
void AirPlayImage::artwork_end(bool complete) {
  if (this->transfer_.load(std::memory_order_relaxed) != TRANSFER_RECEIVING)
    return;
  const bool whole = complete && this->received_.load(std::memory_order_relaxed) == this->encoded_size_;
  this->transfer_.store(whole ? TRANSFER_COMPLETE : TRANSFER_ABORTED, std::memory_order_release);
  xTaskNotifyGive(this->task_.get_handle());
}

// THREAD CONTEXT: AirPlay task
void AirPlayImage::artwork_clear() {
  // An image that is being decoded will not be shown
  this->generation_.fetch_add(1, std::memory_order_acq_rel);
  this->clear_requested_ms_.store(millis(), std::memory_order_relaxed);
  this->clear_pending_.store(true, std::memory_order_release);
  this->enable_loop_soon_any_context();
}

// ===== Decode task =====

void AirPlayImage::decode_task(void *params) {
  auto *self = static_cast<AirPlayImage *>(params);
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (!self->busy_.load(std::memory_order_acquire))
      continue;
    const DecodeResult result = self->decode_();
    // The rest of the image, e.g. padding after the end of a JPEG, is not needed any more
    self->discard_.store(true, std::memory_order_release);
    if (result == DECODE_RESULT_DECODED) {
      self->decoded_generation_ = self->encoded_generation_;
      self->decoded_hash_ = self->encoded_hash_;
      self->decoded_size_ = static_cast<uint32_t>(self->encoded_size_);
      self->decoded_.store(true, std::memory_order_release);
    } else if (result == DECODE_RESULT_FAILED) {
      self->decode_failed_.store(true, std::memory_order_release);
    }
    self->busy_.store(false, std::memory_order_release);
    self->enable_loop_soon_any_context();
  }
}

bool AirPlayImage::wait_for_(size_t count) {
  while (this->received_.load(std::memory_order_acquire) < count) {
    // The transfer state is set after the last data, so the count read after it is final
    if (this->transfer_.load(std::memory_order_acquire) != TRANSFER_RECEIVING)
      return this->received_.load(std::memory_order_acquire) >= count;
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(WAIT_CHECK_MS));
  }
  return true;
}

size_t AirPlayImage::read_encoded_(uint8_t *buffer, size_t length) {
  const size_t consumed = this->consumed_.load(std::memory_order_relaxed);
  length = std::min({length, this->encoded_size_ - consumed, MAX_WAIT_BYTES});
  if (!this->wait_for_(consumed + length))
    length = this->received_.load(std::memory_order_acquire) - consumed;
  const size_t offset = consumed % RING_SIZE;
  const size_t first = std::min(length, RING_SIZE - offset);
  std::memcpy(buffer, this->ring_ + offset, first);
  std::memcpy(buffer + first, this->ring_, length - first);
  this->consumed_.store(consumed + length, std::memory_order_release);
  return length;
}

bool AirPlayImage::skip_encoded_(size_t position) {
  size_t consumed = this->consumed_.load(std::memory_order_relaxed);
  if (position < consumed || position > this->encoded_size_)
    return false;
  while (consumed < position) {
    const size_t step = std::min(position - consumed, MAX_WAIT_BYTES);
    if (!this->wait_for_(consumed + step))
      return false;
    consumed += step;
    this->consumed_.store(consumed, std::memory_order_release);
  }
  return true;
}

AirPlayImage::DecodeResult AirPlayImage::decode_() {
  const size_t size = this->encoded_size_;
  // The start of the image tells whether it is the one on screen; nothing has been read from the ring yet, so it
  // is still there in one piece
  const size_t hashed = std::min(size, HASHED_BYTES);
  if (!this->wait_for_(hashed))
    return DECODE_RESULT_SKIPPED;
  uint32_t hash = FNV1A_OFFSET;
  for (size_t i = 0; i < hashed; i++)
    hash = (hash ^ this->ring_[i]) * FNV1A_PRIME;
  this->encoded_hash_ = hash;
  if (this->shown_size_.load(std::memory_order_acquire) == size &&
      this->shown_hash_.load(std::memory_order_acquire) == hash) {
    ESP_LOGD(TAG, "Same cover art as on screen");
    return DECODE_RESULT_SKIPPED;
  }
  // The back frame is free once the main loop has put the last picture on screen; it wakes this task then
  while (this->decoded_.load(std::memory_order_acquire))
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(WAIT_CHECK_MS));

  uint8_t *target = this->back_frame_();
  const uint32_t start = millis();
  int image_width = 0;
  int image_height = 0;
  int width = 0;
  int height = 0;
  bool ok;
  if (this->encoded_png_) {
    ok = read_png_size(this->ring_, hashed, &image_width, &image_height) &&
         this->decode_png_(target, image_width, image_height, &width, &height);
    if (!ok && image_width == 0) {
      ESP_LOGE(TAG, "Cover art has no readable image size");
    }
  } else {
    ok = this->decode_jpeg_(target, &image_width, &image_height, &width, &height);
  }
  // Lets go of the frame and ends a failed decode; the decoder is kept for the next image
  this->sink_.release();
  if (!ok) {
    // An image that was cut off, e.g. because the session ended, is no error
    return this->transfer_.load(std::memory_order_acquire) == TRANSFER_ABORTED ? DECODE_RESULT_SKIPPED
                                                                               : DECODE_RESULT_FAILED;
  }
  this->decoded_width_ = width;
  this->decoded_height_ = height;
  ESP_LOGD(TAG, "Cover art decoded in %" PRIu32 " ms: %zu bytes, %dx%d to %dx%d", millis() - start, size, image_width,
           image_height, width, height);
  return DECODE_RESULT_DECODED;
}

/// Scaled to fit and keeping its aspect ratio, as RuntimeImage::resize() computes it
static void fit_size(int image_width, int image_height, int max_width, int max_height, int *width, int *height) {
  const float scale =
      std::min(static_cast<float>(max_width) / image_width, static_cast<float>(max_height) / image_height);
  *width = static_cast<int>(image_width * scale);
  *height = static_cast<int>(image_height * scale);
}

bool AirPlayImage::decode_png_(uint8_t *target, int image_width, int image_height, int *width, int *height) {
#ifdef USE_RUNTIME_IMAGE_PNG
  const size_t size = this->encoded_size_;
  fit_size(image_width, image_height, this->max_width_, this->max_height_, width, height);
  if (!this->sink_.set_external_buffer(target, *width, *height) || !this->sink_.begin_decode(size, runtime_image::PNG))
    return false;
  size_t consumed = 0;
  uint8_t piece[PNG_PIECE_SIZE];
  while (consumed < size) {
    const size_t wanted = std::min(size - consumed, PNG_PIECE_SIZE);
    if (!this->wait_for_(consumed + wanted))
      return false;
    const size_t offset = consumed % RING_SIZE;
    const size_t available = std::min(this->received_.load(std::memory_order_acquire) - consumed, MAX_WAIT_BYTES);
    uint8_t *data = this->ring_ + offset;
    size_t length = std::min(available, RING_SIZE - offset);
    if (length < wanted) {
      // Wraps around the end of the ring; the decoder wants a larger piece
      std::memcpy(piece, data, length);
      std::memcpy(piece + length, this->ring_, wanted - length);
      data = piece;
      length = wanted;
    }
    const int fed = this->sink_.feed_data(data, length);
    if (fed <= 0) {
      ESP_LOGE(TAG, "Cover art could not be decoded at byte %zu: %s", consumed,
               runtime_image::decode_error_to_string(fed));
      return false;
    }
    consumed += static_cast<size_t>(fed);
    this->consumed_.store(consumed, std::memory_order_release);
  }
  return this->sink_.end_decode() && this->sink_.decoded_into(target);
#else
  return false;
#endif
}

bool AirPlayImage::decode_jpeg_(uint8_t *target, int *image_width, int *image_height, int *width, int *height) {
#ifdef USE_RUNTIME_IMAGE_JPEG
  if (this->jpeg_decoder_ == nullptr || !this->jpeg_decoder_->open(this->encoded_size_, image_width, image_height))
    return false;
  fit_size(*image_width, *image_height, this->max_width_, this->max_height_, width, height);
  if (!this->sink_.set_external_buffer(target, *width, *height)) {
    this->jpeg_decoder_->close();
    return false;
  }
  return this->jpeg_decoder_->decode_opened() && this->sink_.decoded_into(target);
#else
  return false;
#endif
}

// ===== Main loop =====

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

}  // namespace esphome::airplay_

#endif  // USE_ESP32 && USE_AIRPLAY_ARTWORK
