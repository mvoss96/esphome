#include "airplay_media_source.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"

#include <esp_timer.h>
#include <lwip/sockets.h>
#include <unistd.h>

#include <algorithm>
#include <cinttypes>
#include <cstring>

namespace esphome::airplay {

ESPHOME_LOG_TAG(TAG, "airplay.audio");

static constexpr uint32_t AUDIO_SELECT_TIMEOUT_MS = 5;
static constexpr uint32_t STOP_TIMEOUT_MS = 1000;
static constexpr size_t MAX_PACKETS_PER_READ = 16;
static constexpr uint32_t MAX_PACKETS_PER_STEP = 8;

static constexpr size_t RTP_HEADER_SIZE = 12;
static constexpr size_t RETRANSMIT_HEADER_SIZE = 4;
static constexpr size_t TIMING_PACKET_SIZE = 32;
static constexpr size_t AES_BLOCK_SIZE = 16;

// RTP payload types (second header byte without the marker bit)
static constexpr uint8_t PAYLOAD_TIMING_REQUEST = 0x52;
static constexpr uint8_t PAYLOAD_SYNC = 0x54;
static constexpr uint8_t PAYLOAD_RESEND_REQUEST = 0x55;
static constexpr uint8_t PAYLOAD_RETRANSMIT = 0x56;
static constexpr uint8_t PAYLOAD_AUDIO = 0x60;

/// A missing packet is replaced by silence once the speaker has less than this queued (100 ms)
static constexpr uint32_t CONCEAL_THRESHOLD_FRAMES = SAMPLE_RATE / 10;
/// Clock drift is corrected one frame per packet once the buffer is this far from its target (20 ms)
static constexpr int32_t DRIFT_THRESHOLD_FRAMES = SAMPLE_RATE / 50;
/// Smoothing of the buffer level for drift correction, as a right shift (1/16 per packet)
static constexpr int32_t DRIFT_SMOOTHING_SHIFT = 4;

/// Seconds between 1900 (NTP epoch) and 1970
static constexpr uint64_t NTP_UNIX_OFFSET = 2208988800ULL;

static inline uint16_t read_be16(const uint8_t *data) { return static_cast<uint16_t>((data[0] << 8) | data[1]); }

static void write_ntp_time(uint8_t *out) {
  // Time since boot is fine: senders only use the timing exchange to measure offsets
  const uint64_t us = static_cast<uint64_t>(esp_timer_get_time());
  const uint32_t seconds = static_cast<uint32_t>(us / 1000000 + NTP_UNIX_OFFSET);
  const uint32_t fraction = static_cast<uint32_t>(((us % 1000000) << 32) / 1000000);
  for (int i = 0; i < 4; i++) {
    out[i] = static_cast<uint8_t>(seconds >> (24 - 8 * i));
    out[4 + i] = static_cast<uint8_t>(fraction >> (24 - 8 * i));
  }
}

void AirPlayMediaSource::audio_task(void *params) {
  auto *self = static_cast<AirPlayMediaSource *>(params);
  while (true) {
    xEventGroupWaitBits(self->audio_events_, AUDIO_EVENT_RUN, pdFALSE, pdTRUE, portMAX_DELAY);
    self->run_audio_session_();
    xEventGroupSetBits(self->audio_events_, AUDIO_EVENT_STOPPED);
  }
}

// THREAD CONTEXT: RTSP task
bool AirPlayMediaSource::start_audio_() {
  if (this->codec_ == Codec::NONE || this->audio_fd_ < 0)
    return false;
  this->stop_audio_();
  xEventGroupClearBits(this->audio_events_, AUDIO_EVENT_STOPPED);
  xEventGroupSetBits(this->audio_events_, AUDIO_EVENT_RUN);
  this->audio_running_ = true;
  return true;
}

// THREAD CONTEXT: RTSP task
void AirPlayMediaSource::stop_audio_() {
  if (!this->audio_running_)
    return;
  xEventGroupClearBits(this->audio_events_, AUDIO_EVENT_RUN);
  if ((xEventGroupWaitBits(this->audio_events_, AUDIO_EVENT_STOPPED, pdFALSE, pdTRUE, pdMS_TO_TICKS(STOP_TIMEOUT_MS)) &
       AUDIO_EVENT_STOPPED) == 0) {
    ESP_LOGW(TAG, "Audio task did not stop in time");
  }
  this->audio_running_ = false;
}

// THREAD CONTEXT: RTSP task, while the audio task is stopped
void AirPlayMediaSource::close_udp_sockets_() {
  for (int *fd : {&this->audio_fd_, &this->control_fd_, &this->timing_fd_}) {
    if (*fd >= 0) {
      close(*fd);
      *fd = -1;
    }
  }
}

// THREAD CONTEXT: Audio task. Everything below this point runs in this task.
void AirPlayMediaSource::run_audio_session_() {
  this->reset_playback_();
  this->accept_from_seq_ = this->initial_seq_;
  this->flush_request_.store(FLUSH_REQUEST_NONE, std::memory_order_relaxed);
  this->bytes_written_ = 0;
  this->frames_played_.store(0, std::memory_order_relaxed);
  this->lost_packets_ = 0;
  this->late_packets_ = 0;
  if (this->encrypted_)
    mbedtls_aes_setkey_dec(&this->aes_, this->aes_key_, sizeof(this->aes_key_) * 8);

  const int max_fd = std::max({this->audio_fd_, this->control_fd_, this->timing_fd_});
  while ((xEventGroupGetBits(this->audio_events_) & AUDIO_EVENT_RUN) != 0) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(this->audio_fd_, &read_fds);
    FD_SET(this->control_fd_, &read_fds);
    FD_SET(this->timing_fd_, &read_fds);
    struct timeval timeout {};
    timeout.tv_usec = AUDIO_SELECT_TIMEOUT_MS * 1000;
    const int ready = select(max_fd + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready < 0) {
      vTaskDelay(pdMS_TO_TICKS(AUDIO_SELECT_TIMEOUT_MS));
      continue;
    }

    const uint8_t flush = this->flush_request_.exchange(FLUSH_REQUEST_NONE, std::memory_order_acq_rel);
    if (flush != FLUSH_REQUEST_NONE) {
      // Paused or seeking: drop the buffered audio and wait for the sender to continue
      if (flush == FLUSH_REQUEST_FROM_SEQ) {
        this->accept_from_seq_ = this->flush_seq_.load(std::memory_order_relaxed);
      } else if (this->have_seq_) {
        this->accept_from_seq_ = this->highest_seq_ + 1;
      }
      this->reset_playback_();
    }

    if (ready > 0) {
      if (FD_ISSET(this->audio_fd_, &read_fds))
        this->receive_udp_(this->audio_fd_);
      if (FD_ISSET(this->control_fd_, &read_fds))
        this->receive_udp_(this->control_fd_);
      if (FD_ISSET(this->timing_fd_, &read_fds))
        this->receive_udp_(this->timing_fd_);
    }

    this->playback_step_();
  }

  if (this->lost_packets_ > 0 || this->late_packets_ > 0) {
    ESP_LOGD(TAG, "Session ended: %" PRIu32 " packets lost, %" PRIu32 " arrived too late", this->lost_packets_,
             this->late_packets_);
  }
}

void AirPlayMediaSource::reset_playback_() {
  for (uint32_t i = 0; i < this->slot_count_; i++)
    this->slots_[i].valid = false;
  this->have_seq_ = false;
  this->playback_started_ = false;
  this->head_slot_ = 0;
  this->output_offset_ = 0;
  this->output_length_ = 0;
  this->drift_error_ = 0;
}

void AirPlayMediaSource::receive_udp_(int fd) {
  uint8_t *packet = this->packet_buffer_.get();
  for (size_t i = 0; i < MAX_PACKETS_PER_READ; i++) {
    struct sockaddr_in source {};
    socklen_t source_length = sizeof(source);
    const ssize_t length = recvfrom(fd, packet, PACKET_BUFFER_SIZE, MSG_DONTWAIT,
                                    reinterpret_cast<struct sockaddr *>(&source), &source_length);
    if (length <= 0)
      return;
    if (length < static_cast<ssize_t>(RETRANSMIT_HEADER_SIZE))
      continue;

    switch (packet[1] & 0x7f) {
      case PAYLOAD_AUDIO:
        if (length > static_cast<ssize_t>(RTP_HEADER_SIZE))
          this->handle_audio_packet_(read_be16(packet + 2), packet + RTP_HEADER_SIZE, length - RTP_HEADER_SIZE);
        break;
      case PAYLOAD_RETRANSMIT:
        // A short header followed by the original RTP packet
        if (length > static_cast<ssize_t>(RETRANSMIT_HEADER_SIZE + RTP_HEADER_SIZE)) {
          this->handle_audio_packet_(read_be16(packet + RETRANSMIT_HEADER_SIZE + 2),
                                     packet + RETRANSMIT_HEADER_SIZE + RTP_HEADER_SIZE,
                                     length - RETRANSMIT_HEADER_SIZE - RTP_HEADER_SIZE);
        }
        break;
      case PAYLOAD_TIMING_REQUEST:
        this->handle_timing_request_(fd, packet, length, &source, source_length);
        break;
      case PAYLOAD_SYNC:
      default:
        // Playback is paced by the arrival of audio, so sync packets are not needed
        break;
    }
  }
}

void AirPlayMediaSource::handle_audio_packet_(uint16_t seq, const uint8_t *payload, size_t length) {
  if (!this->have_seq_) {
    if (static_cast<int16_t>(seq - this->accept_from_seq_) < 0) {
      this->late_packets_++;  // Sent before the last flush
      return;
    }
    this->have_seq_ = true;
    this->next_seq_ = seq;
    this->highest_seq_ = seq - 1;
    this->head_slot_ = 0;
    uint8_t paused = SESSION_STATE_PAUSED;
    if (this->session_state_.compare_exchange_strong(paused, SESSION_STATE_PLAYING, std::memory_order_acq_rel))
      this->enable_loop_soon_any_context();
  }

  int32_t ahead = static_cast<int16_t>(seq - this->next_seq_);
  if (ahead < 0) {
    this->late_packets_++;
    return;
  }
  if (static_cast<uint32_t>(ahead) >= this->slot_count_) {
    // Buffer full: give up on the oldest packets to make room
    const uint32_t drop = static_cast<uint32_t>(ahead) - this->slot_count_ + 1;
    for (uint32_t i = 0; i < drop; i++) {
      PacketSlot &old = this->slots_[this->head_slot_];
      if (!old.valid || old.seq != this->next_seq_)
        this->lost_packets_++;
      old.valid = false;
      this->head_slot_ = (this->head_slot_ + 1) % this->slot_count_;
      this->next_seq_++;
    }
    ahead = static_cast<int32_t>(this->slot_count_) - 1;
  }

  PacketSlot &slot = this->slots_[(this->head_slot_ + static_cast<uint32_t>(ahead)) % this->slot_count_];
  if (slot.valid && slot.seq == seq)
    return;  // Duplicate

  const uint8_t *data = payload;
  if (this->encrypted_) {
    // Only whole AES blocks are encrypted, and the IV restarts with every packet
    uint8_t *plain = this->decrypt_buffer_.get();
    const size_t encrypted_length = length & ~(AES_BLOCK_SIZE - 1);
    uint8_t iv[AES_BLOCK_SIZE];
    std::memcpy(iv, this->aes_iv_, sizeof(iv));
    if (encrypted_length > 0)
      mbedtls_aes_crypt_cbc(&this->aes_, MBEDTLS_AES_DECRYPT, encrypted_length, iv, payload, plain);
    std::memcpy(plain + encrypted_length, payload + encrypted_length, length - encrypted_length);
    data = plain;
  }

  int32_t frames;
  if (this->codec_ == Codec::ALAC) {
    frames = this->decoder_.decode(data, length, slot.samples);
  } else {
    // Big endian 16-bit PCM
    frames = static_cast<int32_t>(std::min<size_t>(length / BYTES_PER_FRAME, FRAMES_PER_PACKET));
    for (int32_t i = 0; i < frames * static_cast<int32_t>(NUM_CHANNELS); i++)
      slot.samples[i] = static_cast<int16_t>(read_be16(data + 2 * i));
  }
  if (frames <= 0) {
    ESP_LOGV(TAG, "Failed to decode packet %u", seq);
    return;  // Treated as lost
  }
  slot.seq = seq;
  slot.frames = static_cast<uint16_t>(frames);
  slot.valid = true;

  const int32_t new_ahead = static_cast<int16_t>(seq - this->highest_seq_);
  if (new_ahead > 0) {
    if (new_ahead > 1 && static_cast<uint32_t>(new_ahead) < this->slot_count_)
      this->request_resend_(this->highest_seq_ + 1, static_cast<uint16_t>(new_ahead - 1));
    this->highest_seq_ = seq;
  }
}

void AirPlayMediaSource::handle_timing_request_(int fd, const uint8_t *packet, size_t length, const void *source,
                                                uint32_t source_length) {
  if (length < TIMING_PACKET_SIZE)
    return;
  uint8_t reply[TIMING_PACKET_SIZE] = {0x80, 0xd3, 0x00, 0x07};
  // Origin time is the sender's transmit time, then our receive and transmit times
  std::memcpy(reply + 8, packet + 24, 8);
  write_ntp_time(reply + 16);
  std::memcpy(reply + 24, reply + 16, 8);
  sendto(fd, reply, sizeof(reply), 0, static_cast<const struct sockaddr *>(source), source_length);
}

void AirPlayMediaSource::request_resend_(uint16_t first, uint16_t count) {
  if (this->client_control_port_ == 0)
    return;
  const uint8_t request[8] = {
      0x80,
      0x80 | PAYLOAD_RESEND_REQUEST,
      static_cast<uint8_t>(this->resend_seq_ >> 8),
      static_cast<uint8_t>(this->resend_seq_),
      static_cast<uint8_t>(first >> 8),
      static_cast<uint8_t>(first),
      static_cast<uint8_t>(count >> 8),
      static_cast<uint8_t>(count),
  };
  this->resend_seq_++;
  struct sockaddr_in address {};
  address.sin_family = AF_INET;
  address.sin_port = htons(this->client_control_port_);
  address.sin_addr.s_addr = this->client_address_;
  sendto(this->control_fd_, request, sizeof(request), 0, reinterpret_cast<struct sockaddr *>(&address),
         sizeof(address));
  ESP_LOGV(TAG, "Requested %u packets from %u again", count, first);
}

uint32_t AirPlayMediaSource::speaker_queued_frames_() const {
  const auto written = static_cast<uint32_t>(this->bytes_written_ / BYTES_PER_FRAME);
  const uint32_t queued = written - this->frames_played_.load(std::memory_order_relaxed);
  // Reports from audio of an earlier session can exceed what this session wrote
  return queued > INT32_MAX ? 0 : queued;
}

uint32_t AirPlayMediaSource::buffered_packets_() const {
  if (!this->have_seq_)
    return 0;
  const int32_t span = static_cast<int16_t>(this->highest_seq_ - this->next_seq_) + 1;
  return span > 0 ? static_cast<uint32_t>(span) : 0;
}

void AirPlayMediaSource::playback_step_() {
  if (!this->have_seq_)
    return;

  if (!this->playback_started_) {
    const uint32_t buffered = this->buffered_packets_();
    const uint32_t level = this->speaker_queued_frames_() + buffered * FRAMES_PER_PACKET;
    if (level < this->start_frames_ && buffered + 1 < this->slot_count_)
      return;
    this->playback_started_ = true;
    ESP_LOGD(TAG, "Starting playback with %" PRIu32 " packets buffered", buffered);
  }

  const auto *output = reinterpret_cast<const uint8_t *>(this->output_buffer_.get());
  for (uint32_t i = 0; i < MAX_PACKETS_PER_STEP; i++) {
    if (this->output_offset_ < this->output_length_) {
      const size_t written = this->write_output(output + this->output_offset_,
                                                this->output_length_ - this->output_offset_, 0, this->stream_info_);
      this->output_offset_ += written;
      this->bytes_written_ += written;
      if (this->output_offset_ < this->output_length_)
        return;  // The speaker is full, or this source is not the active one yet
    }

    const uint32_t buffered = this->buffered_packets_();
    const uint32_t queued = this->speaker_queued_frames_();
    if (buffered == 0) {
      if (queued == 0)
        this->playback_started_ = false;  // Ran dry, so build up the buffer again
      return;
    }

    PacketSlot &slot = this->slots_[this->head_slot_];
    uint32_t frames = FRAMES_PER_PACKET;
    if (slot.valid && slot.seq == this->next_seq_) {
      frames = slot.frames;
      std::memcpy(this->output_buffer_.get(), slot.samples, frames * BYTES_PER_FRAME);
      slot.valid = false;
    } else {
      // Missing packet: wait for the retransmission while the speaker still has audio
      if (queued > CONCEAL_THRESHOLD_FRAMES && buffered + 1 < this->slot_count_)
        return;
      std::memset(this->output_buffer_.get(), 0, frames * BYTES_PER_FRAME);
      this->lost_packets_++;
    }
    this->head_slot_ = (this->head_slot_ + 1) % this->slot_count_;
    this->next_seq_++;

    // The sender's clock and the speaker's clock drift apart slowly. Keep the buffer level on target by
    // dropping or repeating a single frame. Needs the speaker's playback reports to measure the level.
    if (this->frames_played_.load(std::memory_order_relaxed) != 0) {
      const int32_t level = static_cast<int32_t>(queued + (buffered - 1) * FRAMES_PER_PACKET);
      const int32_t error = level - static_cast<int32_t>(this->start_frames_);
      this->drift_error_ += (error - this->drift_error_) >> DRIFT_SMOOTHING_SHIFT;
      if (this->drift_error_ > DRIFT_THRESHOLD_FRAMES && frames > 1) {
        frames--;
      } else if (this->drift_error_ < -DRIFT_THRESHOLD_FRAMES) {
        int16_t *samples = this->output_buffer_.get();
        std::memcpy(samples + frames * NUM_CHANNELS, samples + (frames - 1) * NUM_CHANNELS, BYTES_PER_FRAME);
        frames++;
      }
    }
    this->output_offset_ = 0;
    this->output_length_ = frames * BYTES_PER_FRAME;
  }
}

}  // namespace esphome::airplay

#endif  // USE_ESP32
