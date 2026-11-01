#include "aac_decoder.h"

#ifdef USE_AIRPLAY_AAC

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <decoder/impl/esp_aac_dec.h>
#include <esp_audio_dec.h>

#include <algorithm>
#include <cinttypes>
#include <cstring>

namespace esphome::airplay_ {

ESPHOME_LOG_TAG(TAG, "airplay.aac");

static constexpr size_t ADTS_HEADER_SIZE = 7;
/// Larger than any frame of a buffered stream, whose packets fit the receiver's 2 KB buffer
static constexpr size_t MAX_FRAME_SIZE = 2048;
static constexpr uint32_t SAMPLE_RATE = 44100;
static constexpr uint8_t CHANNELS = 2;
// ADTS fields: AAC-LC (object type 2, written as 1) at 44.1 kHz (sampling frequency index 4)
static constexpr uint8_t ADTS_PROFILE_LC = 1;
static constexpr uint8_t ADTS_SAMPLE_RATE_44100 = 4;

bool AirPlayAacDecoder::begin() {
  if (this->frame_ == nullptr) {
    RAMAllocator<uint8_t> allocator;
    this->frame_ = allocator.allocate(ADTS_HEADER_SIZE + MAX_FRAME_SIZE);
    if (this->frame_ == nullptr) {
      ESP_LOGE(TAG, "Failed to allocate the frame buffer");
      return false;
    }
  }
  return this->decoder_ != nullptr || this->open_();
}

bool AirPlayAacDecoder::open_() {
  esp_aac_dec_cfg_t config = ESP_AAC_DEC_CONFIG_DEFAULT();
  config.sample_rate = SAMPLE_RATE;
  config.channel = CHANNELS;
  config.bits_per_sample = 16;
  // Raw frames get an ADTS header, as airplay-esp32 does with this decoder
  config.no_adts_header = false;
  config.aac_plus_enable = false;
  const esp_audio_err_t err = esp_aac_dec_open(&config, sizeof(config), &this->decoder_);
  if (err != ESP_AUDIO_ERR_OK) {
    ESP_LOGE(TAG, "Failed to open the AAC decoder: %d", static_cast<int>(err));
    this->decoder_ = nullptr;
    return false;
  }
  return true;
}

void AirPlayAacDecoder::close_() {
  if (this->decoder_ != nullptr)
    esp_aac_dec_close(this->decoder_);
  this->decoder_ = nullptr;
}

size_t AirPlayAacDecoder::decode(const uint8_t *data, size_t length, int16_t *samples, size_t max_frames) {
  if (this->decoder_ == nullptr || this->frame_ == nullptr || length > MAX_FRAME_SIZE)
    return 0;
  const size_t total = ADTS_HEADER_SIZE + length;
  uint8_t *header = this->frame_;
  header[0] = 0xff;  // Sync word, MPEG-4, no CRC
  header[1] = 0xf1;
  header[2] = static_cast<uint8_t>((ADTS_PROFILE_LC << 6) | (ADTS_SAMPLE_RATE_44100 << 2) | (CHANNELS >> 2));
  header[3] = static_cast<uint8_t>(((CHANNELS & 3) << 6) | (total >> 11));
  header[4] = static_cast<uint8_t>(total >> 3);
  header[5] = static_cast<uint8_t>(((total & 7) << 5) | 0x1f);  // Buffer fullness: variable bit rate
  header[6] = 0xfc;
  std::memcpy(this->frame_ + ADTS_HEADER_SIZE, data, length);

  esp_audio_dec_in_raw_t raw{};
  raw.buffer = this->frame_;
  raw.len = static_cast<uint32_t>(total);
  raw.frame_recover = ESP_AUDIO_DEC_RECOVERY_NONE;
  esp_audio_dec_out_frame_t frame{};
  frame.buffer = reinterpret_cast<uint8_t *>(samples);
  frame.len = static_cast<uint32_t>(max_frames * CHANNELS * sizeof(int16_t));
  esp_audio_dec_info_t info{};
  const uint32_t start = micros();
  const esp_audio_err_t err = esp_aac_dec_decode(this->decoder_, &raw, &frame, &info);
  const uint32_t took = micros() - start;
  this->total_us_ += took;
  this->max_us_ = std::max(this->max_us_, took);
  this->count_++;
  if (err != ESP_AUDIO_ERR_OK || info.channel != CHANNELS) {
    // A broken frame can leave the decoder in a bad state
    ESP_LOGW(TAG, "Failed to decode a frame (%d), restarting the decoder", static_cast<int>(err));
    this->close_();
    this->open_();
    return 0;
  }
  return frame.decoded_size / (CHANNELS * sizeof(int16_t));
}

void AirPlayAacDecoder::end() {
  if (this->count_ > 0) {
    ESP_LOGD(TAG, "Decoded %" PRIu32 " packets: %" PRIu32 " us on average, at most %" PRIu32 " us", this->count_,
             static_cast<uint32_t>(this->total_us_ / this->count_), this->max_us_);
  }
  this->total_us_ = 0;
  this->max_us_ = 0;
  this->count_ = 0;
}

}  // namespace esphome::airplay_

#endif  // USE_AIRPLAY_AAC
