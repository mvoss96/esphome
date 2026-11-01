#pragma once

#include "esphome/core/defines.h"

#if defined(USE_ESP32) && defined(USE_AIRPLAY_ARTWORK)

#include "esphome/components/airplay/airplay_hub.h"
#include "esphome/components/image/image.h"
#include "esphome/components/runtime_image/image_decoder.h"
#include "esphome/components/runtime_image/runtime_image.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/static_task.h"

#ifdef USE_RUNTIME_IMAGE_JPEG
#include <JPEGDEC.h>
#endif

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace esphome::airplay_ {

class AirPlayImage;

/// Decodes into a frame buffer owned by AirPlayImage. Used on the decode task only: decoding changes the fields
/// that drawing reads, so it is never the image on screen.
class AirPlayDecodeSink : public runtime_image::RuntimeImage {
 public:
  using runtime_image::RuntimeImage::RuntimeImage;

  /// True if the decode ended with this buffer still in place. A decode that needs other dimensions drops an
  /// external buffer; the PNG decoder reports success anyway.
  bool decoded_into(const uint8_t *buffer) const { return this->buffer_ == buffer; }
};

#ifdef USE_RUNTIME_IMAGE_JPEG
/// Decodes a JPEG while it arrives: JPEGDEC reads it through AirPlayImage::read_encoded_(), so that the image does
/// not have to be held in memory as a whole.
class AirPlayJpegDecoder : public runtime_image::ImageDecoder {
 public:
  AirPlayJpegDecoder(AirPlayDecodeSink *sink, AirPlayImage *source)
      : runtime_image::ImageDecoder(sink, runtime_image::JPEG), source_(source) {}

  /// Not used: the data is pulled from the source instead
  int decode(uint8_t * /*buffer*/, size_t /*size*/) override { return runtime_image::DECODE_ERROR_UNSUPPORTED_FORMAT; }

  /// @brief Reads the header of an image of `size` bytes. @return false if it is no JPEG that can be decoded.
  bool open(size_t size, int *width, int *height);
  /// @brief Decodes the image opened before into the sink, whose buffer is set to the scaled size by then.
  bool decode_opened();
  void close() { this->jpeg_.close(); }

 protected:
  static int32_t read_callback(JPEGFILE *file, uint8_t *buffer, int32_t length);
  static int32_t seek_callback(JPEGFILE *file, int32_t position);
  static int draw_callback(JPEGDRAW *draw);

  AirPlayImage *source_;
  JPEGDEC jpeg_{};
};
#endif

/// Cover art of the track that plays over AirPlay, decoded and scaled to fit a fixed size.
///
/// THREADING: the AirPlay task writes the encoded image into a small ring buffer as it arrives, and a decode task
/// of its own decodes it from there at the same time into the frame buffer that is not on screen. While the ring
/// buffer is full, the receiver stops reading the sender's connection (artwork_room()). The main loop then swaps
/// the two frames and runs the automations, so drawing never sees a frame that is being decoded.
class AirPlayImage : public AirPlayChild, public image::Image {
 public:
  AirPlayImage(int width, int height, runtime_image::ImageFormat format, image::ImageType type,
               image::Transparency transparency, image::Image *placeholder, bool is_big_endian);

  void setup() override;
  void loop() override;
  void dump_config() override;

  void draw(int x, int y, display::Display *display, Color color_on, Color color_off) override;

  template<typename F> void add_on_image_update_callback(F &&callback) {
    this->update_callback_.add(std::forward<F>(callback));
  }
  template<typename F> void add_on_image_clear_callback(F &&callback) {
    this->clear_callback_.add(std::forward<F>(callback));
  }
  template<typename F> void add_on_image_error_callback(F &&callback) {
    this->error_callback_.add(std::forward<F>(callback));
  }

  // THREAD CONTEXT: AirPlay task. The calls follow the order of airplay::Listener.
  /// @return false to skip the image
  bool artwork_begin(bool png, size_t size);
  void artwork_data(const uint8_t *data, size_t length);
  size_t artwork_room() const;
  void artwork_end(bool complete);
  void artwork_clear();

 protected:
#ifdef USE_RUNTIME_IMAGE_JPEG
  friend class AirPlayJpegDecoder;
#endif
  /// How the transfer of the current image goes; written by the AirPlay task
  enum Transfer : uint8_t {
    TRANSFER_RECEIVING = 0,
    TRANSFER_COMPLETE,
    TRANSFER_ABORTED,
  };
  enum DecodeResult : uint8_t {
    DECODE_RESULT_DECODED = 0,
    DECODE_RESULT_FAILED,
    DECODE_RESULT_SKIPPED,  // Already on screen, cut off or cleared
  };

  static void decode_task(void *params);
  /// THREAD CONTEXT: decode task. Waits until `count` bytes of the image arrived. @return false if they never will.
  bool wait_for_(size_t count);
  /// @brief THREAD CONTEXT: decode task. Copies the next bytes of the image, waiting for them to arrive.
  /// @return the number of bytes copied; less than `length` only at the end of the image or if it was cut off.
  size_t read_encoded_(uint8_t *buffer, size_t length);
  /// @brief THREAD CONTEXT: decode task. Skips ahead to `position`. @return false if that cannot be reached.
  bool skip_encoded_(size_t position);
  /// THREAD CONTEXT: decode task
  DecodeResult decode_();
  bool decode_png_(uint8_t *target, int image_width, int image_height, int *width, int *height);
  bool decode_jpeg_(uint8_t *target, int *image_width, int *image_height, int *width, int *height);
  void set_frame_(int index, int width, int height);
  uint8_t *back_frame_() { return this->frames_[this->front_ ^ 1]; }

  AirPlayDecodeSink sink_;
#ifdef USE_RUNTIME_IMAGE_JPEG
  AirPlayJpegDecoder *jpeg_decoder_{nullptr};  // Lives as long as the image
#endif
  image::Image *placeholder_;
  int max_width_;
  int max_height_;
  size_t frame_size_{0};

  // The encoded image passes through ring_. Positions count bytes from the start of the image.
  uint8_t *ring_{nullptr};
  std::atomic<size_t> received_{0};  // Written by the AirPlay task
  std::atomic<size_t> consumed_{0};  // Written by the decode task
  std::atomic<uint8_t> transfer_{TRANSFER_COMPLETE};
  // Set by artwork_begin() while the decode task owns the image; cleared by the decode task when it is done
  std::atomic<bool> busy_{false};
  // The decode task is done with the image before it ended; the AirPlay task drops the rest
  std::atomic<bool> discard_{false};
  // Written by the AirPlay task before it sets busy_
  size_t encoded_size_{0};
  bool encoded_png_{false};
  uint32_t encoded_generation_{0};
  uint32_t encoded_hash_{0};  // Decode task: hash of the start of the image

  // frames_[front_] is on screen; the decode task writes the other one while decoded_ is false
  uint8_t *frames_[2]{};
  uint8_t front_{0};      // Main loop only; stable while the decode task runs
  int decoded_width_{0};  // Written by the decode task before it sets decoded_
  int decoded_height_{0};
  uint32_t decoded_generation_{0};
  bool showing_artwork_{false};

  std::atomic<bool> decoded_{false};
  std::atomic<bool> decode_failed_{false};
  // A clear waits a moment, because senders clear the cover just before they send it again
  std::atomic<bool> clear_pending_{false};
  std::atomic<uint32_t> clear_requested_ms_{0};
  // Size and hash of the start of the picture on screen, to skip an image that is sent again; 0 when none
  std::atomic<uint32_t> shown_size_{0};
  std::atomic<uint32_t> shown_hash_{0};
  uint32_t decoded_hash_{0};
  uint32_t decoded_size_{0};
  // Counts clears, so that an image decoded before a clear is not shown after it
  std::atomic<uint32_t> generation_{0};

  StaticTask task_;

  LazyCallbackManager<void()> update_callback_;
  LazyCallbackManager<void()> clear_callback_;
  LazyCallbackManager<void()> error_callback_;
};

}  // namespace esphome::airplay_

#endif  // USE_ESP32 && USE_AIRPLAY_ARTWORK
