#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32

#include "esphome/components/audio/audio.h"
#include "esphome/components/media_source/media_source.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/static_task.h"

#ifdef USE_AIRPLAY_METADATA
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#include "alac_decoder.h"
#include "raop_crypto.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>

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

/// Clock samples kept from the timing exchange with the sender
static constexpr uint32_t TIMING_SAMPLE_COUNT = 8;

/// AirPlay 1 (RAOP) receiver that feeds the received audio to a media player as a media source.
class AirPlayMediaSource final : public Component, public media_source::MediaSource {
 public:
  AirPlayMediaSource(const char *name, uint16_t port, uint32_t buffer_duration_ms)
      : name_(name), buffer_duration_ms_(buffer_duration_ms), port_(port) {}

  void setup() override;
  void loop() override;
  void dump_config() override;
  // Registers its mDNS service, so it must run after the mdns component
  float get_setup_priority() const override { return setup_priority::LATE; }

  void set_task_stack_in_psram(bool task_stack_in_psram) { this->task_stack_in_psram_ = task_stack_in_psram; }

#ifdef USE_AIRPLAY_METADATA
  void set_title_text_sensor(text_sensor::TextSensor *sensor) { this->title_text_sensor_ = sensor; }
  void set_artist_text_sensor(text_sensor::TextSensor *sensor) { this->artist_text_sensor_ = sensor; }
  void set_album_text_sensor(text_sensor::TextSensor *sensor) { this->album_text_sensor_ = sensor; }
#endif

  // MediaSource interface
  bool play_uri(const std::string &uri) override;
  void handle_command(media_source::MediaSourceCommand command) override;
  bool can_handle(const std::string &uri) const override;
  // Next and previous are forwarded to the sender
  bool has_internal_playlist() const override { return true; }
  void notify_volume_changed(float volume) override;
  void notify_audio_played(uint32_t frames, int64_t timestamp) override;

 protected:
  /// Playback session state, written by the RTSP and audio tasks, read by the main loop
  enum SessionState : uint8_t {
    SESSION_STATE_NONE = 0,
    SESSION_STATE_PLAYING,
    SESSION_STATE_PAUSED,
  };

  /// Requests to the remote control task, sent as bits of an atomic
  enum RemoteRequest : uint8_t {
    REMOTE_REQUEST_DISCOVER = 1 << 0,
    REMOTE_REQUEST_PLAY = 1 << 1,
    REMOTE_REQUEST_PAUSE = 1 << 2,
    REMOTE_REQUEST_NEXT = 1 << 3,
    REMOTE_REQUEST_PREVIOUS = 1 << 4,
    REMOTE_REQUEST_VOLUME = 1 << 5,
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
    uint32_t rtp;
    uint32_t missing_since_ms;  // Non-zero while the packet is missing and may be asked for again
    uint32_t last_request_ms;
    uint16_t seq;
    uint16_t frames;
    bool valid;
    int16_t samples[FRAMES_PER_PACKET * NUM_CHANNELS];
  };

  struct TimingSample {
    int64_t offset_ns;  // Sender's clock minus ours
    int64_t round_trip_ns;
  };

  /// Remote control (DACP) details of the sender, shared between the RTSP and remote control tasks
  struct RemoteTarget {
    char dacp_id[DACP_ID_SIZE];
    char active_remote[ACTIVE_REMOTE_SIZE];
    uint32_t address;  // IPv4, network byte order
    uint16_t port;     // 0 until found through mDNS
    bool session_active;
    uint32_t session_end_ms;
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
  void update_remote_target_(const char *dacp_id, const char *active_remote, uint32_t address);
  void end_session_();

  // ===== Remote control task (airplay_remote.cpp) =====
  static void remote_task(void *params);
  void remote_task_loop_();
  void send_remote_request_(uint8_t request);
  bool discover_remote_port_(RemoteTarget &target);
  bool send_remote_command_(const RemoteTarget &target, const char *command);

  // ===== Audio task (airplay_audio.cpp) =====
  static void audio_task(void *params);
  bool start_audio_();
  void stop_audio_();
  void close_udp_sockets_();
  void run_audio_session_();
  void reset_playback_();
  void receive_udp_(int fd);
  void handle_audio_packet_(uint16_t seq, uint32_t rtp, const uint8_t *payload, size_t length);
  void handle_sync_packet_(const uint8_t *packet, size_t length);
  void send_timing_request_();
  void handle_timing_reply_(const uint8_t *packet, size_t length);
  void handle_timing_request_(int fd, const uint8_t *packet, size_t length, const void *source, uint32_t source_length);
  void retry_resends_();
  void request_resend_(uint16_t first, uint16_t count);
  void advance_head_();
  /// Converts an RTP timestamp to the local time (esp_timer) at which it should be heard
  bool rtp_to_local_us_(uint32_t rtp, int64_t *local_us) const;
  bool write_pending_output_();
  /// @param trim 0, or +1/-1 to repeat/drop one frame for clock correction
  void queue_packet_output_(const PacketSlot &slot, int32_t trim);
  void queue_silence_output_();
  void playback_step_();
  bool start_playback_();
  bool synced_step_();
  bool unsynced_step_();
  /// Frames waiting in the speaker pipeline, taken from what was written minus what the speaker reported as played
  uint32_t speaker_queued_frames_() const;
  uint32_t buffered_packets_() const;

#ifdef USE_AIRPLAY_METADATA
  static constexpr size_t METADATA_FIELD_SIZE = 128;
  struct Metadata {
    char title[METADATA_FIELD_SIZE];
    char artist[METADATA_FIELD_SIZE];
    char album[METADATA_FIELD_SIZE];
  };
  /// Stores new track details (RTSP task) for publish_metadata_() to pick up in the main loop
  void set_metadata_(const Metadata &metadata);
  void publish_metadata_();

  text_sensor::TextSensor *title_text_sensor_{nullptr};
  text_sensor::TextSensor *artist_text_sensor_{nullptr};
  text_sensor::TextSensor *album_text_sensor_{nullptr};
  Metadata metadata_{};  // Guarded by remote_lock_
  std::atomic<bool> metadata_changed_{false};
#endif

  void set_session_state_(SessionState state);
  void register_mdns_service_();
  void set_wifi_streaming_(bool streaming);

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
  StaticTask remote_task_;
  EventGroupHandle_t audio_events_{nullptr};
  SemaphoreHandle_t remote_lock_{nullptr};

  // Cross-thread state
  std::atomic<uint8_t> session_state_{SESSION_STATE_NONE};
  std::atomic<bool> disconnect_requested_{false};
  std::atomic<uint8_t> remote_requests_{0};
  std::atomic<float> remote_volume_db_{0.0f};
  // Volume last set by the sender, 0 to 1, to avoid sending it back
  std::atomic<float> sender_volume_{-1.0f};
  std::atomic<uint32_t> frames_played_{0};
  std::atomic<int64_t> played_timestamp_us_{0};
  // Set when this source becomes active, so the audio task restarts its count of written frames together with
  // the orchestrator's count of played frames
  std::atomic<bool> reset_frame_counts_{false};
  std::atomic<uint8_t> flush_request_{FLUSH_REQUEST_NONE};
  std::atomic<uint16_t> flush_seq_{0};

  // Main loop state
  bool play_requested_{false};
  bool wifi_streaming_{false};

  // RTSP task state
  RTSPConnection connections_[MAX_RTSP_CONNECTIONS];
  RAMUniquePtr<char[]> rtsp_buffers_;
  RTSPConnection *session_owner_{nullptr};
  int listen_fd_{-1};
  bool airplay_muted_{false};
  bool audio_running_{false};
  float airplay_volume_{-15.0f};

  // Guarded by remote_lock_
  RemoteTarget remote_target_{};

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
  uint16_t client_timing_port_{0};
  uint16_t initial_seq_{0};
  bool have_initial_seq_{false};
  uint16_t sender_version_{0};  // From "User-Agent: AirPlay/<version>"

  // Audio task state
  mbedtls_aes_context aes_;
  RAMUniquePtr<PacketSlot[]> slots_;
  RAMUniquePtr<uint8_t[]> packet_buffer_;
  RAMUniquePtr<uint8_t[]> decrypt_buffer_;
  RAMUniquePtr<int16_t[]> output_buffer_;
  uint32_t slot_count_{0};
  uint32_t fill_target_frames_{0};
  uint32_t head_slot_{0};    // slot of next_seq_
  uint16_t next_seq_{0};     // next sequence number to play
  uint16_t highest_seq_{0};  // highest sequence number received
  uint16_t accept_from_seq_{0};
  uint16_t resend_seq_{0};
  bool have_seq_{false};
  bool accept_any_seq_{false};
  uint32_t last_packet_ms_{0};
  uint32_t last_resend_scan_ms_{0};
  uint32_t frame_count_reset_ms_{0};
  bool frame_count_reset_pending_{false};
  bool playback_started_{false};
  bool synced_{false};
  bool sync_logged_{false};
  bool played_reports_missing_{false};
  size_t output_offset_{0};  // bytes of output_buffer_ already written
  size_t output_length_{0};  // bytes in output_buffer_
  uint32_t silence_pending_frames_{0};
  uint64_t bytes_written_{0};
  uint32_t first_write_ms_{0};
  int32_t drift_error_{0};
  int32_t servo_error_us_{0};
  bool servo_engaged_{false};
  uint32_t trim_counter_{0};
  uint32_t lost_packets_{0};
  uint32_t late_packets_{0};

  // Clock of the sender, from the timing exchange and the sync packets
  TimingSample timing_samples_[TIMING_SAMPLE_COUNT]{};
  uint32_t timing_sample_count_{0};
  uint32_t timing_sample_index_{0};
  uint32_t timing_requests_sent_{0};
  uint32_t last_timing_request_ms_{0};
  int64_t clock_offset_ns_{0};
  bool clock_locked_{false};
  bool anchor_valid_{false};
  uint32_t anchor_rtp_{0};
  int64_t anchor_remote_ns_{0};
  uint32_t sender_latency_frames_{0};
  uint32_t next_rtp_{0};
  bool have_next_rtp_{false};

  audio::AudioStreamInfo stream_info_{16, NUM_CHANNELS, SAMPLE_RATE};
};

}  // namespace esphome::airplay

#endif  // USE_ESP32
