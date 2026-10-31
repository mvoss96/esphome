#pragma once

#include "esphome/core/defines.h"

#if defined(USE_ESP32) && defined(USE_AIRPLAY_ARTWORK)

#include "esphome/components/airplay/airplay_hub.h"
#include "esphome/components/image/image.h"
#include "esphome/components/runtime_image/runtime_image.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/static_task.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace esphome::airplay_ {

/// Decodes into a frame buffer owned by AirPlayImage. Used on the decode task only: decoding changes the fields
/// that drawing reads, so it is never the image on screen.
class AirPlayDecodeSink : public runtime_image::RuntimeImage {
 public:
  using runtime_image::RuntimeImage::RuntimeImage;

  /// True if the decode ended with this buffer still in place. A decode that needs other dimensions drops an
  /// external buffer; the PNG decoder reports success anyway.
  bool decoded_into(const uint8_t *buffer) const { return this->buffer_ == buffer; }
};

/// Cover art of the track that plays over AirPlay, decoded and scaled to fit a fixed size.
///
/// THREADING: the AirPlay task stores the encoded image as it arrives. A decode task of its own turns it into
/// pixels in the frame buffer that is not on screen. The main loop then swaps the two frames and runs the
/// automations, so drawing never sees a frame that is being decoded.
class AirPlayImage : public AirPlayChild, public image::Image {
 public:
  AirPlayImage(int width, int height, runtime_image::ImageFormat format, image::ImageType type,
               image::Transparency transparency, image::Image *placeholder, bool is_big_endian, size_t max_size);

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
  void artwork_end(bool complete);
  void artwork_clear();

 protected:
  /// Who owns encoded_
  enum EncodedState : uint8_t {
    ENCODED_IDLE = 0,
    ENCODED_RECEIVING,  // The AirPlay task writes it
    ENCODED_READY,      // A whole image waits for the decode task
    ENCODED_DECODING,   // The decode task reads it
  };

  static void decode_task(void *params);
  /// THREAD CONTEXT: decode task. @return true if back_frame_() holds the new picture
  bool decode_();
  void set_frame_(int index, int width, int height);
  uint8_t *back_frame_() { return this->frames_[this->front_ ^ 1]; }

  AirPlayDecodeSink sink_;
  image::Image *placeholder_;
  int max_width_;
  int max_height_;
  size_t frame_size_{0};

  uint8_t *encoded_{nullptr};
  size_t max_size_;
  // Written by the AirPlay task while receiving, read by the decode task while decoding
  size_t encoded_size_{0};
  size_t received_{0};
  bool encoded_png_{false};
  uint32_t encoded_generation_{0};
  uint32_t encoded_hash_{0};

  // frames_[front_] is on screen; the decode task writes the other one while decoded_ is false
  uint8_t *frames_[2]{};
  uint8_t front_{0};      // Main loop only; stable while the decode task runs
  int decoded_width_{0};  // Written by the decode task before it sets decoded_
  int decoded_height_{0};
  uint32_t decoded_generation_{0};
  bool showing_artwork_{false};

  std::atomic<uint8_t> encoded_state_{ENCODED_IDLE};
  std::atomic<bool> decoded_{false};
  std::atomic<bool> decode_failed_{false};
  // A clear waits a moment, because senders clear the cover just before they send it again
  std::atomic<bool> clear_pending_{false};
  std::atomic<uint32_t> clear_requested_ms_{0};
  // Size and hash of the picture on screen, to skip an image that is sent again; 0 when none
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
