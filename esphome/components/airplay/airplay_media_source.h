#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32

#include "esphome/components/audio/audio.h"
#include "esphome/components/media_source/media_source.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/static_task.h"

#include "alac_decoder.h"
#include "raop_crypto.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include <mbedtls/aes.h>

#include <atomic>
#include <cstdint>
#include <string>

namespace esphome::airplay {

/// Frames in one AirPlay audio packet
static constexpr uint32_t FRAMES_PER_PACKET = 352;
static constexpr uint32_t SAMPLE_RATE = 44100;
static constexpr uint32_t NUM_CHANNELS = 2;
static constexpr size_t BYTES_PER_FRAME = NUM_CHANNELS * sizeof(int16_t);

/// Number of RTSP connections handled at once. One owns the playback session, the other can be a sender that
/// probes the receiver or takes over.
static constexpr size_t MAX_RTSP_CONNECTIONS = 2;
static constexpr size_t RTSP_BUFFER_SIZE = 2048;

/// Large enough for a UDP packet with the largest audio payload (uncompressed ALAC or PCM) plus headers
static constexpr size_t PACKET_BUFFER_SIZE = 1600;

static constexpr size_t DACP_ID_SIZE = 17;
static constexpr size_t ACTIVE_REMOTE_SIZE = 12;

/// AirPlay 1 (RAOP) receiver that feeds the received audio to a media player as a media source.
class AirPlayMediaSource final : public Component, public media_source::MediaSource {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  // Registers its mDNS service, so it must run after the mdns component
  float get_setup_priority() const override { return setup_priority::LATE; }

  AirPlayMediaSource(const char *name, uint16_t port, uint32_t buffer_duration_ms)
      : name_(name), buffer_duration_ms_(buffer_duration_ms), port_(port) {}

  void set_task_stack_in_psram(bool task_stack_in_psram) { this->task_stack_in_psram_ = task_stack_in_psram; }

  // MediaSource interface
  bool play_uri(const std::string &uri) override;
  void handle_command(media_source::MediaSourceCommand command) override;
  bool can_handle(const std::string &uri) const override;
  // Next and previous are forwarded to the sender
  bool has_internal_playlist() const override { return true; }
  void notify_audio_played(uint32_t frames, int64_t timestamp) override;

 protected:
  /// Playback session state, written by the RTSP and audio tasks, read by the main loop
  enum SessionState : uint8_t {
    SESSION_STATE_NONE = 0,
    SESSION_STATE_PLAYING,
    SESSION_STATE_PAUSED,
  };

  /// Requests from the main loop to the RTSP task, sent as bits of an atomic
  enum RemoteRequest : uint8_t {
    REMOTE_REQUEST_DISCONNECT = 1 << 0,
    REMOTE_REQUEST_PLAY = 1 << 1,
    REMOTE_REQUEST_PAUSE = 1 << 2,
    REMOTE_REQUEST_NEXT = 1 << 3,
    REMOTE_REQUEST_PREVIOUS = 1 << 4,
  };

  enum FlushRequest : uint8_t {
    FLUSH_REQUEST_NONE = 0,
    FLUSH_REQUEST_FROM_SEQ,  // Drop audio before flush_seq_
    FLUSH_REQUEST_ALL,       // Drop all audio received so far
  };

  static constexpr EventBits_t AUDIO_EVENT_RUN = 1 << 0;
  static constexpr EventBits_t AUDIO_EVENT_STOPPED = 1 << 1;

  enum class Codec : uint8_t {
    NONE,
    ALAC,
    PCM,
  };

  struct RTSPConnection {
    int fd{-1};
    size_t rx_length{0};
    size_t discard_remaining{0};
    uint32_t peer_address{0};  // IPv4, network byte order
    char *buffer{nullptr};
  };

  struct PacketSlot {
    uint16_t seq;
    uint16_t frames;
    bool valid;
    int16_t samples[FRAMES_PER_PACKET * NUM_CHANNELS];
  };

  // ===== RTSP task (airplay_rtsp.cpp) =====
  static void rtsp_task(void *params);
  void rtsp_task_loop_();
  bool open_listen_socket_();
  void accept_connection_();
  void close_connection_(RTSPConnection &connection);
  void receive_from_connection_(RTSPConnection &connection);
  /// @return false if the connection must be closed
  bool process_request_(RTSPConnection &connection, char *request, size_t header_length, char *body,
                        size_t body_length);
  bool handle_announce_(const char *body, size_t body_length);
  bool handle_setup_(RTSPConnection &connection, const char *transport, char *response_headers, size_t size);
  void handle_set_parameter_(const char *content_type, const char *body, size_t body_length);
  bool add_apple_response_(const RTSPConnection &connection, const char *challenge, char *output, size_t size);
  void end_session_();
  void process_remote_requests_();
  bool send_dacp_command_(const char *command);

  // ===== Audio task (airplay_audio.cpp) =====
  static void audio_task(void *params);
  bool start_audio_();
  void stop_audio_();
  void close_udp_sockets_();
  void run_audio_session_();
  void reset_playback_();
  void receive_udp_(int fd);
  void handle_audio_packet_(uint16_t seq, const uint8_t *payload, size_t length);
  void handle_timing_request_(int fd, const uint8_t *packet, size_t length, const void *source, uint32_t source_length);
  void request_resend_(uint16_t first, uint16_t count);
  void playback_step_();
  /// Frames waiting in the speaker pipeline, taken from what was written minus what the speaker reported as played
  uint32_t speaker_queued_frames_() const;
  uint32_t buffered_packets_() const;

  void set_session_state_(SessionState state);
  void register_mdns_service_();

  RAOPCrypto crypto_;
  ALACDecoder decoder_;

  // Configuration
  const char *name_;
  uint32_t buffer_duration_ms_;
  uint16_t port_;
  bool task_stack_in_psram_{false};

  uint8_t mac_[6]{};

  StaticTask rtsp_task_;
  StaticTask audio_task_;
  EventGroupHandle_t audio_events_{nullptr};

  // Cross-thread state
  std::atomic<uint8_t> session_state_{SESSION_STATE_NONE};
  std::atomic<uint8_t> remote_requests_{0};
  std::atomic<uint32_t> frames_played_{0};
  std::atomic<uint8_t> flush_request_{FLUSH_REQUEST_NONE};
  std::atomic<uint16_t> flush_seq_{0};

  // Main loop state
  bool play_requested_{false};

  // RTSP task state
  RTSPConnection connections_[MAX_RTSP_CONNECTIONS];
  RAMUniquePtr<char[]> rtsp_buffers_;
  RTSPConnection *session_owner_{nullptr};
  int listen_fd_{-1};
  char dacp_id_[DACP_ID_SIZE]{};
  char active_remote_[ACTIVE_REMOTE_SIZE]{};
  uint16_t dacp_port_{0};
  bool airplay_muted_{false};
  bool audio_running_{false};
  float airplay_volume_{-15.0f};

  // Session parameters, written by the RTSP task while the audio task is stopped
  Codec codec_{Codec::NONE};
  bool encrypted_{false};
  uint8_t aes_key_[16]{};
  uint8_t aes_iv_[16]{};
  int audio_fd_{-1};
  int control_fd_{-1};
  int timing_fd_{-1};
  uint32_t client_address_{0};  // IPv4, network byte order
  uint16_t client_control_port_{0};
  uint16_t initial_seq_{0};

  // Audio task state
  mbedtls_aes_context aes_;
  RAMUniquePtr<PacketSlot[]> slots_;
  RAMUniquePtr<uint8_t[]> packet_buffer_;
  RAMUniquePtr<uint8_t[]> decrypt_buffer_;
  RAMUniquePtr<int16_t[]> output_buffer_;
  uint32_t slot_count_{0};
  uint32_t head_slot_{0};    // slot of next_seq_
  uint16_t next_seq_{0};     // next sequence number to play
  uint16_t highest_seq_{0};  // highest sequence number received
  uint16_t accept_from_seq_{0};
  uint16_t resend_seq_{0};
  bool have_seq_{false};
  bool playback_started_{false};
  size_t output_offset_{0};  // bytes of output_buffer_ already written
  size_t output_length_{0};  // bytes in output_buffer_
  uint64_t bytes_written_{0};
  uint32_t start_frames_{0};
  int32_t drift_error_{0};
  uint32_t lost_packets_{0};
  uint32_t late_packets_{0};
  audio::AudioStreamInfo stream_info_{16, NUM_CHANNELS, SAMPLE_RATE};
};

}  // namespace esphome::airplay

#endif  // USE_ESP32
