#pragma once

#include "esphome/core/defines.h"

#ifdef USE_AIRPLAY_ARTWORK

#include "esphome/components/runtime_image/runtime_image.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace esphome::airplay {

/// Cover art of the track that plays over AirPlay, decoded and scaled to a fixed size.
///
/// The AirPlay task stores the encoded image as it arrives; the main loop decodes it.
class AirPlayImage : public Component, public runtime_image::RuntimeImage {
 public:
  AirPlayImage(int width, int height, runtime_image::ImageFormat format, image::ImageType type,
               image::Transparency transparency, image::Image *placeholder, bool is_big_endian, size_t max_size);

  void setup() override;
  void loop() override;
  void dump_config() override;

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
  enum State : uint8_t {
    STATE_IDLE = 0,
    STATE_RECEIVING,  // The AirPlay task writes encoded_
    STATE_READY,      // A whole image waits in encoded_
    STATE_DECODING,   // The main loop reads encoded_
  };

  void decode_();
  void clear_();

  uint8_t *encoded_{nullptr};
  size_t max_size_;
  // Written by the AirPlay task while receiving, read by the main loop while decoding
  size_t encoded_size_{0};
  size_t received_{0};
  bool encoded_png_{false};

  std::atomic<uint8_t> state_{STATE_IDLE};
  std::atomic<bool> clear_requested_{false};

  LazyCallbackManager<void()> update_callback_;
  LazyCallbackManager<void()> clear_callback_;
  LazyCallbackManager<void()> error_callback_;
};

}  // namespace esphome::airplay

#endif  // USE_AIRPLAY_ARTWORK
