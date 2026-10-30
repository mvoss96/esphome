#include "airplay_media_source.h"

#ifdef USE_ESP32

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <esp_timer.h>
#include <lwip/sockets.h>
#include <unistd.h>

#include <algorithm>
#include <cinttypes>
#include <cstdlib>
#include <cstring>

namespace esphome::airplay {

ESPHOME_LOG_TAG(TAG, "airplay.audio");

static constexpr uint32_t AUDIO_SELECT_TIMEOUT_MS = 5;
static constexpr uint32_t STOP_TIMEOUT_MS = 1000;
static constexpr size_t MAX_PACKETS_PER_READ = 16;
static constexpr uint32_t MAX_CHUNKS_PER_STEP = 8;

static constexpr size_t RTP_HEADER_SIZE = 12;
static constexpr size_t RETRANSMIT_HEADER_SIZE = 4;
static constexpr size_t TIMING_PACKET_SIZE = 32;
static constexpr size_t SYNC_PACKET_SIZE = 20;
static constexpr size_t AES_BLOCK_SIZE = 16;

// RTP payload types (second header byte without the marker bit)
static constexpr uint8_t PAYLOAD_TIMING_REQUEST = 0x52;
static constexpr uint8_t PAYLOAD_TIMING_REPLY = 0x53;
static constexpr uint8_t PAYLOAD_SYNC = 0x54;
static constexpr uint8_t PAYLOAD_RESEND_REQUEST = 0x55;
static constexpr uint8_t PAYLOAD_RETRANSMIT = 0x56;
static constexpr uint8_t PAYLOAD_AUDIO = 0x60;

// Clock exchange with the sender: a quick burst until the clock locks, then a slow refresh
static constexpr uint32_t TIMING_BURST_COUNT = 3;
static constexpr uint32_t TIMING_BURST_INTERVAL_MS = 100;
static constexpr uint32_t TIMING_INTERVAL_MS = 3000;
static constexpr uint32_t TIMING_LOCK_SAMPLES = 2;
static constexpr int64_t TIMING_MAX_ROUND_TRIP_NS = 500000000;
/// Added to the latency of the sync packets by current senders (see shairport-sync rtp.c)
static constexpr uint32_t FIXED_LATENCY_OFFSET_FRAMES = 11025;
static constexpr uint32_t MAX_SENDER_LATENCY_FRAMES = 5 * SAMPLE_RATE;

// Retransmission: wait briefly for reordered packets, then ask again every 150 ms until the packet is due
static constexpr uint32_t RESEND_FIRST_DELAY_MS = 20;
static constexpr uint32_t RESEND_RETRY_INTERVAL_MS = 150;
static constexpr uint32_t RESEND_SCAN_INTERVAL_MS = 50;
static constexpr uint32_t MAX_RESEND_RANGES = 4;

// Synchronized playback
/// Audio is handed to the speaker at most this far ahead, so that a pause or seek takes effect quickly
static constexpr int64_t MAX_SPEAKER_QUEUE_US = 150000;
/// Silence to line up the first packet is only written once it is due within this window
static constexpr int64_t START_WINDOW_US = 100000;
/// Packets later than this are dropped; packets earlier than this are lined up with silence
static constexpr int64_t EARLY_LATE_THRESHOLD_US = 50000;
/// A missing packet is replaced by silence once it is due within this time
static constexpr int64_t CONCEAL_MARGIN_US = 10000;
/// Time to wait for the sender's clock before falling back to unsynchronized playback
static constexpr uint32_t SYNC_WAIT_MS = 2000;
/// Extra buffer needed on top of the sender's latency for synchronized playback (250 ms)
static constexpr uint32_t SYNC_MARGIN_FRAMES = SAMPLE_RATE / 4;
/// Timestamps of playback reports older than this are not used
static constexpr int64_t REPORT_MAX_AGE_US = 200000;
/// The speaker must report played audio within this time, or playback cannot be synchronized
static constexpr uint32_t PLAYED_REPORT_TIMEOUT_MS = 1000;

// Position servo for synchronized playback (after airplay-esp32 audio_timing.c): trims single samples
// at the quietest point of a packet when the filtered timing error leaves a dead band
static constexpr int32_t SERVO_FILTER_DIV = 16;
static constexpr int32_t SERVO_INNOVATION_CLAMP_US = 1500;
static constexpr int32_t SERVO_ENGAGE_US = 5000;
static constexpr int32_t SERVO_DISENGAGE_US = 1500;
static constexpr uint32_t SERVO_TRIM_INTERVAL = 4;
static constexpr int32_t SERVO_FAST_US = 10000;

// Unsynchronized playback: the buffer level is kept on target instead
/// Default buffer level before playback starts (500 ms)
static constexpr uint32_t FILL_TARGET_FRAMES = SAMPLE_RATE / 2;
/// A missing packet is replaced by silence once the speaker has less than this queued (60 ms). The speaker
/// is fed the whole buffer ahead, so that there is time for retransmissions.
static constexpr uint32_t CONCEAL_THRESHOLD_FRAMES = SAMPLE_RATE * 60 / 1000;
/// Audio beyond this much above the target buffer level is dropped in whole packets (100 ms)
static constexpr uint32_t OVERFILL_FRAMES = SAMPLE_RATE / 10;
/// Dead band of the buffer level control (20 ms) and its smoothing, as a right shift (about 2 s)
static constexpr int32_t FILL_DEADBAND_FRAMES = SAMPLE_RATE / 50;
static constexpr int32_t FILL_SMOOTHING_SHIFT = 8;

/// Time for the orchestrator to reset its count of played frames after this source becomes active
static constexpr uint32_t FRAME_COUNT_SETTLE_MS = 20;
/// The session shows as paused when a playing sender sends no audio for this long
static constexpr uint32_t SENDER_IDLE_MS = 10000;

/// Seconds between 1900 (NTP epoch) and 1970
static constexpr int64_t NTP_UNIX_OFFSET_NS = 2208988800LL * 1000000000LL;

static inline uint16_t read_be16(const uint8_t *data) { return static_cast<uint16_t>((data[0] << 8) | data[1]); }

static inline uint32_t read_be32(const uint8_t *data) {
  return (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) | data[3];
}

/// Local time on the NTP time scale, in nanoseconds. Only offsets matter, so time since boot is used.
static inline int64_t local_ntp_ns() { return esp_timer_get_time() * 1000LL + NTP_UNIX_OFFSET_NS; }

static int64_t read_ntp_ns(const uint8_t *data) {
  const uint64_t seconds = read_be32(data);
  const uint64_t fraction = read_be32(data + 4);
  return static_cast<int64_t>(seconds * 1000000000ULL + ((fraction * 1000000000ULL) >> 32));
}

static void write_ntp_ns(uint8_t *out, int64_t ns) {
  const auto value = static_cast<uint64_t>(ns);
  const auto seconds = static_cast<uint32_t>(value / 1000000000ULL);
  const auto fraction = static_cast<uint32_t>(((value % 1000000000ULL) << 32) / 1000000000ULL);
  for (int i = 0; i < 4; i++) {
    out[i] = static_cast<uint8_t>(seconds >> (24 - 8 * i));
    out[4 + i] = static_cast<uint8_t>(fraction >> (24 - 8 * i));
  }
}

static inline int64_t frames_to_us(int64_t frames) { return frames * 1000000 / SAMPLE_RATE; }

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
  this->accept_any_seq_ = !this->have_initial_seq_;
  this->flush_request_.store(FLUSH_REQUEST_NONE, std::memory_order_relaxed);
  this->last_packet_ms_ = millis();
  this->last_resend_scan_ms_ = 0;
  this->lost_packets_ = 0;
  this->late_packets_ = 0;
  // The written and played frame counts are not reset here: the speaker may still play audio of an earlier
  // session, and the orchestrator keeps counting it. They restart when this source becomes active.

  this->timing_requests_sent_ = 0;
  this->last_timing_request_ms_ = 0;
  this->timing_sample_count_ = 0;
  this->timing_sample_index_ = 0;
  this->clock_locked_ = false;
  this->anchor_valid_ = false;
  this->played_reports_missing_ = false;
  this->sync_logged_ = false;

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
        this->accept_any_seq_ = false;
      } else if (this->have_seq_) {
        this->accept_from_seq_ = this->highest_seq_ + 1;
        this->accept_any_seq_ = false;
      }
      this->reset_playback_();
      // The timeline may jump after a seek, so wait for the next sync packet
      this->anchor_valid_ = false;
      // Set here rather than by the RTSP task, so that audio from before the flush cannot switch back to playing
      uint8_t playing = SESSION_STATE_PLAYING;
      if (this->session_state_.compare_exchange_strong(playing, SESSION_STATE_PAUSED, std::memory_order_acq_rel))
        this->enable_loop_soon_any_context();
    }

    if (ready > 0) {
      if (FD_ISSET(this->audio_fd_, &read_fds))
        this->receive_udp_(this->audio_fd_);
      if (FD_ISSET(this->control_fd_, &read_fds))
        this->receive_udp_(this->control_fd_);
      if (FD_ISSET(this->timing_fd_, &read_fds))
        this->receive_udp_(this->timing_fd_);
    }

    const uint32_t now = millis();
    // Keep asking quickly until the clock locks, so that a lost packet does not delay synchronized playback
    const uint32_t timing_interval = this->timing_requests_sent_ < TIMING_BURST_COUNT || !this->clock_locked_
                                         ? TIMING_BURST_INTERVAL_MS
                                         : TIMING_INTERVAL_MS;
    if (this->timing_requests_sent_ == 0 || now - this->last_timing_request_ms_ >= timing_interval)
      this->send_timing_request_();

    if (now - this->last_resend_scan_ms_ >= RESEND_SCAN_INTERVAL_MS) {
      this->last_resend_scan_ms_ = now;
      this->retry_resends_();
    }

    // A sender that stops sending without a FLUSH (e.g. iOS after a long pause) is shown as paused.
    // Dead senders are detected by TCP keepalive on the RTSP connection.
    if (this->session_state_.load(std::memory_order_relaxed) == SESSION_STATE_PLAYING &&
        now - this->last_packet_ms_ > SENDER_IDLE_MS) {
      uint8_t playing = SESSION_STATE_PLAYING;
      if (this->session_state_.compare_exchange_strong(playing, SESSION_STATE_PAUSED, std::memory_order_acq_rel))
        this->enable_loop_soon_any_context();
    }

    this->playback_step_();
  }

  if (this->lost_packets_ > 0 || this->late_packets_ > 0) {
    ESP_LOGD(TAG, "Session ended: %" PRIu32 " packets lost, %" PRIu32 " arrived too late", this->lost_packets_,
             this->late_packets_);
  }
}

void AirPlayMediaSource::reset_playback_() {
  for (uint32_t i = 0; i < this->slot_count_; i++) {
    this->slots_[i].valid = false;
    this->slots_[i].missing_since_ms = 0;
  }
  this->have_seq_ = false;
  this->have_next_rtp_ = false;
  this->playback_started_ = false;
  this->head_slot_ = 0;
  this->output_offset_ = 0;
  this->output_length_ = 0;
  this->silence_pending_frames_ = 0;
  this->drift_error_ = 0;
  this->servo_error_us_ = 0;
  this->servo_engaged_ = false;
  this->trim_counter_ = 0;
}

void AirPlayMediaSource::receive_udp_(int fd) {
  uint8_t *packet = this->packet_buffer_.get();
  for (size_t i = 0; i < MAX_PACKETS_PER_READ; i++) {
    struct sockaddr_in source {};
    socklen_t source_length = sizeof(source);
    const ssize_t received = recvfrom(fd, packet, PACKET_BUFFER_SIZE, MSG_DONTWAIT,
                                      reinterpret_cast<struct sockaddr *>(&source), &source_length);
    if (received <= 0)
      return;
    const auto length = static_cast<size_t>(received);
    if (length < RETRANSMIT_HEADER_SIZE)
      continue;

    switch (packet[1] & 0x7f) {
      case PAYLOAD_AUDIO:
        if (length > RTP_HEADER_SIZE) {
          this->handle_audio_packet_(read_be16(packet + 2), read_be32(packet + 4), packet + RTP_HEADER_SIZE,
                                     length - RTP_HEADER_SIZE);
        }
        break;
      case PAYLOAD_RETRANSMIT: {
        // A short header followed by the original RTP packet
        const uint8_t *original = packet + RETRANSMIT_HEADER_SIZE;
        if (length > RETRANSMIT_HEADER_SIZE + RTP_HEADER_SIZE) {
          this->handle_audio_packet_(read_be16(original + 2), read_be32(original + 4), original + RTP_HEADER_SIZE,
                                     length - RETRANSMIT_HEADER_SIZE - RTP_HEADER_SIZE);
        }
        break;
      }
      case PAYLOAD_SYNC:
        this->handle_sync_packet_(packet, length);
        break;
      case PAYLOAD_TIMING_REPLY:
        this->handle_timing_reply_(packet, length);
        break;
      case PAYLOAD_TIMING_REQUEST:
        this->handle_timing_request_(fd, packet, length, &source, source_length);
        break;
      default:
        break;
    }
  }
}

void AirPlayMediaSource::handle_audio_packet_(uint16_t seq, uint32_t rtp, const uint8_t *payload, size_t length) {
  const uint32_t now = millis();
  this->last_packet_ms_ = now;
  if (!this->have_seq_) {
    if (!this->accept_any_seq_ && static_cast<int16_t>(seq - this->accept_from_seq_) < 0) {
      this->late_packets_++;  // Sent before the last flush
      return;
    }
    this->have_seq_ = true;
    this->next_seq_ = seq;
    this->highest_seq_ = seq - 1;
    this->head_slot_ = 0;
    this->first_packet_ms_ = now;
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
      const PacketSlot &old = this->slots_[this->head_slot_];
      if (!old.valid || old.seq != this->next_seq_)
        this->lost_packets_++;
      this->advance_head_();
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
  slot.rtp = rtp;
  slot.frames = static_cast<uint16_t>(frames);
  slot.valid = true;
  slot.missing_since_ms = 0;

  const int32_t new_ahead = static_cast<int16_t>(seq - this->highest_seq_);
  if (new_ahead > 0) {
    // Mark the packets in between as missing; retry_resends_() asks for them after a short delay
    for (int32_t i = 1; i < std::min(new_ahead, ahead + 1); i++) {
      PacketSlot &gap = this->slots_[(this->head_slot_ + static_cast<uint32_t>(ahead - i)) % this->slot_count_];
      gap.valid = false;
      gap.missing_since_ms = now;
      gap.last_request_ms = 0;
    }
    this->highest_seq_ = seq;
  }

  // Leave the paused state when real audio arrives. iOS sends silence for a moment after a pause.
  if (this->session_state_.load(std::memory_order_relaxed) == SESSION_STATE_PAUSED) {
    const int16_t *samples = slot.samples;
    const bool audible = std::any_of(samples, samples + frames * NUM_CHANNELS, [](int16_t s) { return s != 0; });
    uint8_t paused = SESSION_STATE_PAUSED;
    if (audible &&
        this->session_state_.compare_exchange_strong(paused, SESSION_STATE_PLAYING, std::memory_order_acq_rel)) {
      this->enable_loop_soon_any_context();
    }
  }
}

void AirPlayMediaSource::handle_sync_packet_(const uint8_t *packet, size_t length) {
  if (length < SYNC_PACKET_SIZE)
    return;
  // The frame at bytes 4-7 is due at the sender's time in bytes 8-15, plus a fixed offset for current
  // senders. Bytes 16-19 hold the frame being sent now, so their difference is the sender's latency.
  const uint16_t flags = read_be16(packet + 2);
  const uint32_t rtp_less_latency = read_be32(packet + 4);
  const uint32_t rtp_now = read_be32(packet + 16);
  const uint32_t latency = rtp_now - rtp_less_latency;
  if (latency > MAX_SENDER_LATENCY_FRAMES)
    return;

  const uint16_t version = this->sender_version_;
  const bool fixed_offset = flags == 7 || (version > 0 && version <= 353) || version >= 371;
  this->anchor_rtp_ = rtp_less_latency;
  this->anchor_remote_ns_ = read_ntp_ns(packet + 8);
  if (fixed_offset)
    this->anchor_remote_ns_ += frames_to_us(FIXED_LATENCY_OFFSET_FRAMES) * 1000;
  this->sender_latency_frames_ = latency + (fixed_offset ? FIXED_LATENCY_OFFSET_FRAMES : 0);
  this->anchor_valid_ = true;
}

void AirPlayMediaSource::send_timing_request_() {
  this->last_timing_request_ms_ = millis();
  this->timing_requests_sent_++;
  if (this->client_timing_port_ == 0)
    return;
  uint8_t request[TIMING_PACKET_SIZE] = {0x80, 0x80 | PAYLOAD_TIMING_REQUEST, 0x00, 0x07};
  write_ntp_ns(request + 24, local_ntp_ns());
  struct sockaddr_in address {};
  address.sin_family = AF_INET;
  address.sin_port = htons(this->client_timing_port_);
  address.sin_addr.s_addr = this->client_address_;
  sendto(this->timing_fd_, request, sizeof(request), 0, reinterpret_cast<struct sockaddr *>(&address), sizeof(address));
}

void AirPlayMediaSource::handle_timing_reply_(const uint8_t *packet, size_t length) {
  if (length < TIMING_PACKET_SIZE)
    return;
  const int64_t arrival = local_ntp_ns();
  const int64_t origin = read_ntp_ns(packet + 8);     // Our transmit time, echoed back
  const int64_t receive = read_ntp_ns(packet + 16);   // Sender's receive time
  const int64_t transmit = read_ntp_ns(packet + 24);  // Sender's transmit time
  const int64_t round_trip = (arrival - origin) - (transmit - receive);
  if (round_trip < 0 || round_trip > TIMING_MAX_ROUND_TRIP_NS)
    return;

  TimingSample &sample = this->timing_samples_[this->timing_sample_index_];
  sample.offset_ns = ((receive - origin) + (transmit - arrival)) / 2;
  sample.round_trip_ns = round_trip;
  this->timing_sample_index_ = (this->timing_sample_index_ + 1) % TIMING_SAMPLE_COUNT;
  if (this->timing_sample_count_ < TIMING_SAMPLE_COUNT)
    this->timing_sample_count_++;

  // The sample with the shortest round trip has the smallest error
  const TimingSample *best = &this->timing_samples_[0];
  for (uint32_t i = 1; i < this->timing_sample_count_; i++) {
    if (this->timing_samples_[i].round_trip_ns < best->round_trip_ns)
      best = &this->timing_samples_[i];
  }
  this->clock_offset_ns_ = best->offset_ns;
  if (!this->clock_locked_ && this->timing_sample_count_ >= TIMING_LOCK_SAMPLES) {
    this->clock_locked_ = true;
    ESP_LOGD(TAG, "Clock locked to the sender (round trip %" PRId32 " us)",
             static_cast<int32_t>(best->round_trip_ns / 1000));
  }
}

void AirPlayMediaSource::handle_timing_request_(int fd, const uint8_t *packet, size_t length, const void *source,
                                                uint32_t source_length) {
  if (length < TIMING_PACKET_SIZE)
    return;
  uint8_t reply[TIMING_PACKET_SIZE] = {0x80, 0x80 | PAYLOAD_TIMING_REPLY, 0x00, 0x07};
  // Origin time is the requester's transmit time, then our receive and transmit times
  std::memcpy(reply + 8, packet + 24, 8);
  write_ntp_ns(reply + 16, local_ntp_ns());
  std::memcpy(reply + 24, reply + 16, 8);
  sendto(fd, reply, sizeof(reply), 0, static_cast<const struct sockaddr *>(source), source_length);
}

void AirPlayMediaSource::retry_resends_() {
  if (!this->have_seq_)
    return;
  const uint32_t now = millis();
  const uint32_t buffered = this->buffered_packets_();
  uint32_t ranges = 0;
  uint32_t i = 0;
  while (i < buffered && ranges < MAX_RESEND_RANGES) {
    PacketSlot *slot = &this->slots_[(this->head_slot_ + i) % this->slot_count_];
    const auto due = [now](const PacketSlot &s) {
      if (s.valid || s.missing_since_ms == 0)
        return false;
      return s.last_request_ms == 0 ? now - s.missing_since_ms >= RESEND_FIRST_DELAY_MS
                                    : now - s.last_request_ms >= RESEND_RETRY_INTERVAL_MS;
    };
    if (!due(*slot)) {
      i++;
      continue;
    }
    // Ask for a run of consecutive missing packets at once
    const uint16_t first = this->next_seq_ + i;
    uint16_t count = 0;
    while (i < buffered) {
      slot = &this->slots_[(this->head_slot_ + i) % this->slot_count_];
      if (!due(*slot))
        break;
      slot->last_request_ms = now;
      count++;
      i++;
    }
    this->request_resend_(first, count);
    ranges++;
  }
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

void AirPlayMediaSource::advance_head_() {
  PacketSlot &slot = this->slots_[this->head_slot_];
  // Track the timestamp of the next packet, so that a missing one can still be timed
  if (slot.valid && slot.seq == this->next_seq_) {
    this->next_rtp_ = slot.rtp + slot.frames;
    this->have_next_rtp_ = true;
  } else if (this->have_next_rtp_) {
    this->next_rtp_ += FRAMES_PER_PACKET;
  }
  slot.valid = false;
  slot.missing_since_ms = 0;
  this->head_slot_ = (this->head_slot_ + 1) % this->slot_count_;
  this->next_seq_++;
}

bool AirPlayMediaSource::rtp_to_local_us_(uint32_t rtp, int64_t *local_us) const {
  if (!this->anchor_valid_ || !this->clock_locked_)
    return false;
  const int64_t remote_ns =
      this->anchor_remote_ns_ + static_cast<int64_t>(static_cast<int32_t>(rtp - this->anchor_rtp_)) * 1000000000LL /
                                    static_cast<int64_t>(SAMPLE_RATE);
  *local_us = (remote_ns - this->clock_offset_ns_ - NTP_UNIX_OFFSET_NS) / 1000;
  return true;
}

bool AirPlayMediaSource::write_pending_output_() {
  if (this->output_offset_ >= this->output_length_)
    return true;
  if (this->reset_frame_counts_.load(std::memory_order_acquire))
    return false;  // Became active just now: playback_step_() restarts the counts first
  const auto *output = reinterpret_cast<const uint8_t *>(this->output_buffer_.get());
  const size_t written = this->write_output(output + this->output_offset_, this->output_length_ - this->output_offset_,
                                            0, this->stream_info_);
  if (written > 0 && this->first_write_ms_ == 0)
    this->first_write_ms_ = millis();
  this->output_offset_ += written;
  this->bytes_written_ += written;
  // Not done if the speaker is full, or this source is not the active one yet
  return this->output_offset_ >= this->output_length_;
}

void AirPlayMediaSource::queue_packet_output_(const PacketSlot &slot, int32_t trim) {
  int16_t *samples = this->output_buffer_.get();
  uint32_t frames = slot.frames;
  std::memcpy(samples, slot.samples, frames * BYTES_PER_FRAME);

  if (trim != 0 && frames > 2) {
    // Drop or repeat one frame where the audio is quietest, which makes the seam inaudible
    uint32_t quietest = 1;
    int32_t quietest_level = INT32_MAX;
    for (uint32_t i = 1; i + 1 < frames; i++) {
      const int32_t level = std::abs(samples[i * NUM_CHANNELS]) + std::abs(samples[i * NUM_CHANNELS + 1]);
      if (level < quietest_level) {
        quietest_level = level;
        quietest = i;
      }
    }
    int16_t *at = samples + quietest * NUM_CHANNELS;
    const size_t tail_bytes = (frames - quietest - 1) * BYTES_PER_FRAME;
    if (trim < 0) {
      std::memmove(at, at + NUM_CHANNELS, tail_bytes);
      frames--;
    } else {
      std::memmove(at + NUM_CHANNELS, at, tail_bytes + BYTES_PER_FRAME);
      frames++;
    }
  }
  this->output_offset_ = 0;
  this->output_length_ = frames * BYTES_PER_FRAME;
}

void AirPlayMediaSource::queue_silence_output_() {
  const uint32_t frames = std::min(this->silence_pending_frames_, FRAMES_PER_PACKET + 1);
  std::memset(this->output_buffer_.get(), 0, frames * BYTES_PER_FRAME);
  this->silence_pending_frames_ -= frames;
  this->output_offset_ = 0;
  this->output_length_ = frames * BYTES_PER_FRAME;
}

void AirPlayMediaSource::playback_step_() {
  if (this->reset_frame_counts_.exchange(false, std::memory_order_acq_rel)) {
    this->frame_count_reset_pending_ = true;
    this->frame_count_reset_ms_ = millis();
  }
  if (this->frame_count_reset_pending_) {
    // play_uri() runs just before the orchestrator resets its count; writing in between would go uncounted
    if (millis() - this->frame_count_reset_ms_ < FRAME_COUNT_SETTLE_MS)
      return;
    this->frame_count_reset_pending_ = false;
    this->bytes_written_ = 0;
    this->first_write_ms_ = 0;
    this->frames_played_.store(0, std::memory_order_relaxed);
    this->played_timestamp_us_.store(0, std::memory_order_relaxed);
  }

  if (!this->played_reports_missing_ && this->first_write_ms_ != 0 &&
      this->frames_played_.load(std::memory_order_relaxed) == 0 &&
      millis() - this->first_write_ms_ > PLAYED_REPORT_TIMEOUT_MS) {
    this->played_reports_missing_ = true;
    ESP_LOGW(TAG, "The speaker does not report played audio; playback cannot be synchronized");
    this->synced_ = false;
  }

  if (!this->have_seq_)
    return;
  if (!this->playback_started_ && !this->start_playback_())
    return;

  for (uint32_t i = 0; i < MAX_CHUNKS_PER_STEP; i++) {
    if (!this->write_pending_output_())
      return;
    if (this->silence_pending_frames_ > 0) {
      this->queue_silence_output_();
      continue;
    }
    if (this->buffered_packets_() == 0) {
      if (this->speaker_queued_frames_() == 0)
        this->playback_started_ = false;  // Ran dry, so start again once there is enough audio
      return;
    }
    if (!(this->synced_ ? this->synced_step_() : this->unsynced_step_()))
      return;
  }
}

bool AirPlayMediaSource::start_playback_() {
  const uint32_t buffered = this->buffered_packets_();
  const uint32_t capacity_frames = this->slot_count_ * FRAMES_PER_PACKET;
  const bool fits = capacity_frames >= this->sender_latency_frames_ + SYNC_MARGIN_FRAMES;

  if (this->clock_locked_ && this->anchor_valid_ && fits && !this->played_reports_missing_) {
    this->synced_ = true;
  } else {
    // Wait a moment for the sender's clock, unless synchronized playback is impossible anyway
    const bool may_sync = !this->played_reports_missing_ && (!this->anchor_valid_ || fits);
    if (may_sync && millis() - this->first_packet_ms_ < SYNC_WAIT_MS && buffered + 1 < this->slot_count_)
      return false;
    const uint32_t level = this->speaker_queued_frames_() + buffered * FRAMES_PER_PACKET;
    if (level < this->fill_target_frames_ && buffered + 1 < this->slot_count_)
      return false;
    this->synced_ = false;
  }

  if (!this->sync_logged_) {
    this->sync_logged_ = true;
    if (this->synced_) {
      ESP_LOGI(TAG, "Playback synchronized to the sender (latency %" PRIu32 " ms)",
               static_cast<uint32_t>(frames_to_us(this->sender_latency_frames_) / 1000));
    } else if (this->anchor_valid_ && !fits) {
      ESP_LOGW(TAG,
               "Playback not synchronized: the sender needs %" PRIu32 " ms of buffer, set buffer_duration to at "
               "least %" PRIu32 " ms",
               static_cast<uint32_t>(frames_to_us(this->sender_latency_frames_) / 1000),
               static_cast<uint32_t>(frames_to_us(this->sender_latency_frames_ + SYNC_MARGIN_FRAMES) / 1000));
    } else {
      ESP_LOGI(TAG, "Playback not synchronized to the sender");
    }
  }
  this->playback_started_ = true;
  this->servo_error_us_ = 0;
  this->servo_engaged_ = false;
  this->drift_error_ = 0;
  return true;
}

bool AirPlayMediaSource::synced_step_() {
  const PacketSlot &slot = this->slots_[this->head_slot_];
  const bool present = slot.valid && slot.seq == this->next_seq_;
  if (!present && !this->have_next_rtp_) {
    // Nothing to time a missing first packet against
    this->advance_head_();
    return true;
  }
  const uint32_t rtp = present ? slot.rtp : this->next_rtp_;
  int64_t target_us;
  if (!this->rtp_to_local_us_(rtp, &target_us)) {
    // Lost the timing reference (e.g. a seek and no sync packet yet): wait, then start over
    this->playback_started_ = false;
    return false;
  }

  const int64_t queued_us = frames_to_us(this->speaker_queued_frames_());
  if (queued_us >= MAX_SPEAKER_QUEUE_US)
    return false;
  // The speaker reports played audio in chunks. The audio it reported last ended at the report's timestamp,
  // and the queued audio follows on from there, which is more precise than counting from now.
  const int64_t now = esp_timer_get_time();
  int64_t heard_us = now + queued_us;
  const int64_t reported_at = this->played_timestamp_us_.load(std::memory_order_relaxed);
  if (queued_us > 0 && reported_at != 0 && now - reported_at < REPORT_MAX_AGE_US)
    heard_us = std::max(now, reported_at + queued_us);
  // Positive: the packet would be heard too early; negative: too late
  const int64_t error_us = target_us - heard_us;
  if (error_us > START_WINDOW_US)
    return false;

  if (!present) {
    if (error_us > CONCEAL_MARGIN_US)
      return false;  // Still time for the retransmission
    // Fill the gap with silence, unless it is long past already
    if (error_us > -EARLY_LATE_THRESHOLD_US)
      this->silence_pending_frames_ = FRAMES_PER_PACKET;
    this->lost_packets_++;
    this->advance_head_();
    return true;
  }

  if (error_us < -EARLY_LATE_THRESHOLD_US) {
    this->late_packets_++;
    this->advance_head_();
    return true;
  }
  // Line up with silence when starting from an empty speaker, or when far too early
  if (error_us > EARLY_LATE_THRESHOLD_US || (queued_us == 0 && error_us > 0)) {
    this->silence_pending_frames_ = static_cast<uint32_t>(error_us * SAMPLE_RATE / 1000000);
    this->servo_error_us_ = 0;
    if (this->silence_pending_frames_ > 0)
      return true;
  }

  // Filter the timing error; one reading can move it only a little, because late readings are common
  // when this task was briefly held up
  const auto innovation = static_cast<int32_t>(
      std::clamp<int64_t>(error_us - this->servo_error_us_, -SERVO_INNOVATION_CLAMP_US, SERVO_INNOVATION_CLAMP_US));
  this->servo_error_us_ += innovation / SERVO_FILTER_DIV;
  const int32_t magnitude = std::abs(this->servo_error_us_);
  if (magnitude > SERVO_ENGAGE_US) {
    this->servo_engaged_ = true;
  } else if (magnitude < SERVO_DISENGAGE_US) {
    this->servo_engaged_ = false;
  }
  // Larger errors (e.g. after the clock estimate moved) are corrected on every packet, small ones slowly
  const uint32_t interval = magnitude > SERVO_FAST_US ? 1 : SERVO_TRIM_INTERVAL;
  int32_t trim = 0;
  if (this->servo_engaged_ && ++this->trim_counter_ >= interval) {
    this->trim_counter_ = 0;
    trim = this->servo_error_us_ > 0 ? 1 : -1;  // Early: repeat a frame; late: drop one
  }

  this->queue_packet_output_(slot, trim);
  this->advance_head_();
  return true;
}

bool AirPlayMediaSource::unsynced_step_() {
  const uint32_t buffered = this->buffered_packets_();
  const uint32_t queued = this->speaker_queued_frames_();
  const bool have_reports = this->frames_played_.load(std::memory_order_relaxed) != 0;

  const PacketSlot &slot = this->slots_[this->head_slot_];
  if (have_reports && buffered > 1 &&
      queued + buffered * FRAMES_PER_PACKET > this->fill_target_frames_ + OVERFILL_FRAMES) {
    // Far too much audio, e.g. it piled up while another source was being stopped: skip whole packets,
    // which is much quicker than the level control
    this->advance_head_();
    return true;
  }

  if (!slot.valid || slot.seq != this->next_seq_) {
    // Missing packet: wait for the retransmission while the speaker still has audio
    if (queued > CONCEAL_THRESHOLD_FRAMES && buffered + 1 < this->slot_count_)
      return false;
    this->silence_pending_frames_ = FRAMES_PER_PACKET;
    this->lost_packets_++;
    this->advance_head_();
    return true;
  }

  // The sender's clock and the speaker's clock drift apart slowly. Keep the buffer level on target by
  // dropping or repeating single frames. Needs the speaker's playback reports to measure the level.
  int32_t trim = 0;
  if (have_reports) {
    const auto level = static_cast<int32_t>(queued + (buffered - 1) * FRAMES_PER_PACKET);
    const int32_t error = level - static_cast<int32_t>(this->fill_target_frames_);
    this->drift_error_ += (error - this->drift_error_) >> FILL_SMOOTHING_SHIFT;
    if (std::abs(this->drift_error_) > FILL_DEADBAND_FRAMES && ++this->trim_counter_ >= SERVO_TRIM_INTERVAL) {
      this->trim_counter_ = 0;
      trim = this->drift_error_ > 0 ? -1 : 1;  // Too much audio: drop a frame; too little: repeat one
    }
  }
  this->queue_packet_output_(slot, trim);
  this->advance_head_();
  return true;
}

}  // namespace esphome::airplay

#endif  // USE_ESP32
