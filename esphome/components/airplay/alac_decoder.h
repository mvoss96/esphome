#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

namespace esphome::airplay {

/// Parameters of an Apple Lossless stream, as sent by AirPlay in the SDP "fmtp" line:
/// frame_length compatible_version bit_depth pb mb kb num_channels max_run max_frame_bytes avg_bit_rate sample_rate
struct ALACConfig {
  uint32_t frame_length{352};
  uint8_t bit_depth{16};
  uint8_t pb{40};
  uint8_t mb{10};
  uint8_t kb{14};
  uint8_t num_channels{2};
  uint16_t max_run{255};
  uint32_t sample_rate{44100};
};

/// Minimal Apple Lossless (ALAC) decoder for 16-bit mono or stereo streams.
///
/// Follows the reference decoder Apple published under the Apache 2.0 license. Only the element types that
/// AirPlay senders produce are supported (single and paired channel elements, fill and data stream elements).
class ALACDecoder {
 public:
  /// @brief Allocates the work buffers. Call once during setup.
  /// @param max_frame_length Largest frame length that will ever be configured.
  /// @return false if the allocation failed.
  bool init(uint32_t max_frame_length);

  /// @brief Sets the stream parameters.
  /// @return false if the parameters are not supported by this decoder.
  bool configure(const ALACConfig &config);

  const ALACConfig &get_config() const { return this->config_; }

  /// @brief Decodes one ALAC packet into interleaved 16-bit samples.
  /// @param data Compressed packet.
  /// @param length Length of the compressed packet in bytes.
  /// @param output Buffer for the decoded samples, room for frame_length * num_channels samples.
  /// @return Number of frames decoded, or a negative value if the packet is invalid.
  int32_t decode(const uint8_t *data, size_t length, int16_t *output);

 protected:
  std::unique_ptr<int32_t[]> predictor_;
  std::unique_ptr<int32_t[]> mix_u_;
  std::unique_ptr<int32_t[]> mix_v_;
  uint32_t max_frame_length_{0};
  ALACConfig config_;
  bool configured_{false};
};

}  // namespace esphome::airplay
