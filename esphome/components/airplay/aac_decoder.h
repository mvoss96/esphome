#pragma once

#include "esphome/core/defines.h"

#ifdef USE_AIRPLAY_AAC

#include <airplay.h>

#include <cstddef>
#include <cstdint>

namespace esphome::airplay_ {

/// AAC decoder for AirPlay 2 buffered audio, using Espressif's esp_audio_codec. The decoder is opened with the first
/// stream and kept, so that later streams allocate nothing.
class AirPlayAacDecoder final : public ::airplay::AacDecoder {
 public:
  bool begin() override;
  size_t decode(const uint8_t *data, size_t length, int16_t *samples, size_t max_frames) override;
  /// Logs how long decoding took during the stream
  void end() override;

 protected:
  bool open_();
  void close_();

  void *decoder_{nullptr};
  uint8_t *frame_{nullptr};  // The frame behind an ADTS header
  // Decoding time of the current stream
  uint64_t total_us_{0};
  uint32_t max_us_{0};
  uint32_t count_{0};
};

}  // namespace esphome::airplay_

#endif  // USE_AIRPLAY_AAC
