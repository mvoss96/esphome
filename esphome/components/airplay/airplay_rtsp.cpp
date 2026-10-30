#include "airplay_media_source.h"

#ifdef USE_ESP32

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <lwip/sockets.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>

namespace esphome::airplay {

ESPHOME_LOG_TAG(TAG, "airplay.rtsp");

static constexpr uint32_t SELECT_TIMEOUT_MS = 100;
static constexpr uint32_t LISTEN_RETRY_MS = 1000;
static constexpr size_t RTSP_MAX_BODY_SIZE = 1024 * 1024;
// Detect senders that disappear without closing the connection
static constexpr int KEEPALIVE_IDLE_S = 30;
static constexpr int KEEPALIVE_INTERVAL_S = 5;
static constexpr int KEEPALIVE_COUNT = 3;
static constexpr int LISTEN_BACKLOG = 2;

static constexpr size_t RESPONSE_SIZE = 1024;
static constexpr size_t EXTRA_HEADERS_SIZE = 512;
static constexpr size_t RESPONSE_BODY_SIZE = 64;

/// AirPlay volume is -30 (quiet) to 0 (full) dB, with -144 meaning mute
static constexpr float AIRPLAY_VOLUME_MIN = -30.0f;
static constexpr float AIRPLAY_VOLUME_MUTE_LIMIT = -100.0f;

static constexpr char PUBLIC_METHODS[] =
    "Public: ANNOUNCE, SETUP, RECORD, PAUSE, FLUSH, TEARDOWN, OPTIONS, GET_PARAMETER, SET_PARAMETER\r\n";

/// Copies the value of a header from a raw header block ("Name: value" lines separated by CRLF).
/// @return true if the header was found
static bool get_header(const char *block, size_t block_length, const char *name, char *value, size_t value_size) {
  const size_t name_length = strlen(name);
  const char *end = block + block_length;
  const char *line = block;
  while (line < end) {
    const char *line_end = line;
    while (line_end < end && *line_end != '\r' && *line_end != '\n')
      line_end++;
    if (static_cast<size_t>(line_end - line) > name_length && strncasecmp(line, name, name_length) == 0 &&
        line[name_length] == ':') {
      const char *start = line + name_length + 1;
      while (start < line_end && (*start == ' ' || *start == '\t'))
        start++;
      const char *stop = line_end;
      while (stop > start && (stop[-1] == ' ' || stop[-1] == '\t'))
        stop--;
      const size_t length = std::min(static_cast<size_t>(stop - start), value_size - 1);
      std::memcpy(value, start, length);
      value[length] = '\0';
      return true;
    }
    line = line_end;
    while (line < end && (*line == '\r' || *line == '\n'))
      line++;
  }
  return false;
}

/// Finds "key=" in a parameter list such as "control_port=6001;timing_port=6002" and parses its number.
static bool get_numeric_parameter(const char *text, const char *key, uint32_t *value) {
  const size_t key_length = strlen(key);
  for (const char *p = text; (p = strstr(p, key)) != nullptr; p += key_length) {
    if ((p == text || p[-1] == ';' || p[-1] == ' ') && p[key_length] == '=') {
      *value = strtoul(p + key_length + 1, nullptr, 10);
      return true;
    }
  }
  return false;
}

static bool send_all(int fd, const char *data, size_t length) {
  while (length > 0) {
    const ssize_t sent = send(fd, data, length, 0);
    if (sent <= 0)
      return false;
    data += sent;
    length -= static_cast<size_t>(sent);
  }
  return true;
}

static int open_udp_socket(uint16_t *port) {
  const int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0)
    return -1;
  struct sockaddr_in address {};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = 0;
  socklen_t length = sizeof(address);
  if (bind(fd, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) != 0 ||
      getsockname(fd, reinterpret_cast<struct sockaddr *>(&address), &length) != 0) {
    close(fd);
    return -1;
  }
  *port = ntohs(address.sin_port);
  return fd;
}

void AirPlayMediaSource::rtsp_task(void *params) { static_cast<AirPlayMediaSource *>(params)->rtsp_task_loop_(); }

// THREAD CONTEXT: RTSP task. Everything below this point runs in this task unless stated otherwise.
void AirPlayMediaSource::rtsp_task_loop_() {
  while (!this->open_listen_socket_())
    vTaskDelay(pdMS_TO_TICKS(LISTEN_RETRY_MS));

  while (true) {
    if (this->disconnect_requested_.exchange(false, std::memory_order_acq_rel)) {
      if (this->session_owner_ != nullptr) {
        ESP_LOGI(TAG, "Disconnecting the sender");
        // Pause the sender too, so it does not switch to its own speaker and keep playing there
        this->send_remote_request_(REMOTE_REQUEST_PAUSE);
        this->close_connection_(*this->session_owner_);
      } else {
        this->set_session_state_(SESSION_STATE_NONE);
      }
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(this->listen_fd_, &read_fds);
    int max_fd = this->listen_fd_;
    for (auto &connection : this->connections_) {
      if (connection.fd >= 0) {
        FD_SET(connection.fd, &read_fds);
        max_fd = std::max(max_fd, connection.fd);
      }
    }

    struct timeval timeout {};
    timeout.tv_usec = SELECT_TIMEOUT_MS * 1000;
    const int ready = select(max_fd + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready < 0) {
      ESP_LOGW(TAG, "select() failed: %d", errno);
      vTaskDelay(pdMS_TO_TICKS(SELECT_TIMEOUT_MS));
      continue;
    }
    if (ready == 0)
      continue;

    // Read from existing connections before accepting, so a new connection cannot reuse a descriptor
    // number that is still marked as readable in read_fds
    for (auto &connection : this->connections_) {
      if (connection.fd >= 0 && FD_ISSET(connection.fd, &read_fds))
        this->receive_from_connection_(connection);
    }
    if (FD_ISSET(this->listen_fd_, &read_fds))
      this->accept_connection_();
  }
}

bool AirPlayMediaSource::open_listen_socket_() {
  const int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0) {
    ESP_LOGW(TAG, "Failed to create socket: %d", errno);
    return false;
  }
  int enable = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

  struct sockaddr_in address {};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons(this->port_);
  if (bind(fd, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) != 0 ||
      listen(fd, LISTEN_BACKLOG) != 0) {
    ESP_LOGW(TAG, "Failed to listen on port %u: %d", this->port_, errno);
    close(fd);
    return false;
  }
  this->listen_fd_ = fd;
  return true;
}

void AirPlayMediaSource::accept_connection_() {
  struct sockaddr_in address {};
  socklen_t address_length = sizeof(address);
  const int fd = accept(this->listen_fd_, reinterpret_cast<struct sockaddr *>(&address), &address_length);
  if (fd < 0)
    return;

  RTSPConnection *slot = nullptr;
  for (auto &connection : this->connections_) {
    if (connection.fd < 0) {
      slot = &connection;
      break;
    }
  }
  if (slot == nullptr) {
    // All slots are taken: drop the connection that is not playing
    for (auto &connection : this->connections_) {
      if (&connection != this->session_owner_) {
        this->close_connection_(connection);
        slot = &connection;
        break;
      }
    }
  }

  int enable = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable));
  setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &enable, sizeof(enable));
  int keepalive = KEEPALIVE_IDLE_S;
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &keepalive, sizeof(keepalive));
  keepalive = KEEPALIVE_INTERVAL_S;
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepalive, sizeof(keepalive));
  keepalive = KEEPALIVE_COUNT;
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &keepalive, sizeof(keepalive));

  slot->fd = fd;
  slot->rx_length = 0;
  slot->discard_remaining = 0;
  slot->peer_address = address.sin_addr.s_addr;
  const auto *ip = reinterpret_cast<const uint8_t *>(&slot->peer_address);
  ESP_LOGD(TAG, "Connection from %u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}

void AirPlayMediaSource::close_connection_(RTSPConnection &connection) {
  if (connection.fd < 0)
    return;
  if (&connection == this->session_owner_)
    this->end_session_();
  close(connection.fd);
  connection.fd = -1;
  connection.rx_length = 0;
  connection.discard_remaining = 0;
}

void AirPlayMediaSource::receive_from_connection_(RTSPConnection &connection) {
  const size_t space = RTSP_BUFFER_SIZE - connection.rx_length;
  const ssize_t received = recv(connection.fd, connection.buffer + connection.rx_length, space, 0);
  if (received <= 0) {
    ESP_LOGD(TAG, "Connection closed");
    this->close_connection_(connection);
    return;
  }
  connection.rx_length += static_cast<size_t>(received);

  while (connection.fd >= 0 && connection.rx_length > 0) {
    char *buffer = connection.buffer;

    if (connection.discard_remaining > 0) {
      // Skipping the body of a request that did not fit in the buffer
      const size_t drop = std::min(connection.discard_remaining, connection.rx_length);
      std::memmove(buffer, buffer + drop, connection.rx_length - drop);
      connection.rx_length -= drop;
      connection.discard_remaining -= drop;
      continue;
    }

    buffer[connection.rx_length] = '\0';
    const char *header_end = strstr(buffer, "\r\n\r\n");
    if (header_end == nullptr) {
      if (connection.rx_length == RTSP_BUFFER_SIZE) {
        ESP_LOGW(TAG, "Request headers too large");
        this->close_connection_(connection);
      }
      return;
    }
    const size_t header_length = static_cast<size_t>(header_end - buffer) + 4;

    char value[16];
    size_t content_length = 0;
    if (get_header(buffer, header_length, "Content-Length", value, sizeof(value)))
      content_length = strtoul(value, nullptr, 10);
    if (content_length > RTSP_MAX_BODY_SIZE) {
      ESP_LOGW(TAG, "Request body too large");
      this->close_connection_(connection);
      return;
    }

    size_t consumed;
    bool keep_open;
    if (header_length + content_length > RTSP_BUFFER_SIZE) {
      // Too large to buffer (e.g. cover art): answer without the body and skip it
      keep_open = this->process_request_(connection, buffer, header_length, nullptr, 0);
      consumed = std::min(connection.rx_length, header_length + content_length);
      connection.discard_remaining = header_length + content_length - consumed;
    } else {
      if (connection.rx_length < header_length + content_length)
        return;  // Wait for the rest of the body
      consumed = header_length + content_length;
      // Null terminate the body in place, restoring the byte that follows it afterwards
      char *body = buffer + header_length;
      const char saved = body[content_length];
      body[content_length] = '\0';
      keep_open = this->process_request_(connection, buffer, header_length, body, content_length);
      body[content_length] = saved;
    }

    if (!keep_open) {
      this->close_connection_(connection);
      return;
    }
    std::memmove(buffer, buffer + consumed, connection.rx_length - consumed);
    connection.rx_length -= consumed;
  }
}

bool AirPlayMediaSource::process_request_(RTSPConnection &connection, char *request, size_t header_length, char *body,
                                          size_t body_length) {
  char method[20];
  size_t method_length = 0;
  while (method_length < header_length && method_length < sizeof(method) - 1 && request[method_length] != ' ')
    method_length++;
  std::memcpy(method, request, method_length);
  method[method_length] = '\0';

  char cseq[16] = "0";
  get_header(request, header_length, "CSeq", cseq, sizeof(cseq));
  ESP_LOGV(TAG, "%s (CSeq %s)", method, cseq);

  int status = 200;
  const char *status_text = "OK";
  bool keep_open = true;
  char extra_headers[EXTRA_HEADERS_SIZE];
  extra_headers[0] = '\0';
  char response_body[RESPONSE_BODY_SIZE];
  response_body[0] = '\0';

  const bool is_owner = (&connection == this->session_owner_);

  if (strcmp(method, "OPTIONS") == 0) {
    snprintf(extra_headers, sizeof(extra_headers), "%s", PUBLIC_METHODS);
  } else if (strcmp(method, "ANNOUNCE") == 0) {
    if (!is_owner && this->session_owner_ != nullptr) {
      ESP_LOGI(TAG, "Another sender took over");
      this->close_connection_(*this->session_owner_);
    }
    this->stop_audio_();
    this->session_owner_ = &connection;
    // Senders identify as e.g. "AirPlay/381.13"; the version decides how the latency is computed
    char user_agent[48] = "";
    this->sender_version_ = 0;
    if (get_header(request, header_length, "User-Agent", user_agent, sizeof(user_agent)) &&
        strncasecmp(user_agent, "AirPlay/", 8) == 0) {
      this->sender_version_ = static_cast<uint16_t>(strtoul(user_agent + 8, nullptr, 10));
    }
    if (!this->handle_announce_(body != nullptr ? body : "", body_length)) {
      status = 415;
      status_text = "Unsupported Media Type";
    }
  } else if (strcmp(method, "SETUP") == 0) {
    char transport[192] = "";
    get_header(request, header_length, "Transport", transport, sizeof(transport));
    if (!is_owner || !this->handle_setup_(connection, transport, extra_headers, sizeof(extra_headers))) {
      status = 455;
      status_text = "Method Not Valid in This State";
    }
  } else if (strcmp(method, "RECORD") == 0) {
    char rtp_info[96] = "";
    uint32_t seq = 0;
    this->have_initial_seq_ = get_header(request, header_length, "RTP-Info", rtp_info, sizeof(rtp_info)) &&
                              get_numeric_parameter(rtp_info, "seq", &seq);
    this->initial_seq_ = static_cast<uint16_t>(seq);
    if (is_owner && this->start_audio_()) {
      ESP_LOGI(TAG, "Playback started");
      this->set_session_state_(SESSION_STATE_PLAYING);
      // The same value as shairport-sync and the AirPort Express
      snprintf(extra_headers, sizeof(extra_headers), "Audio-Latency: 11025\r\n");
    } else {
      status = 455;
      status_text = "Method Not Valid in This State";
    }
  } else if (strcmp(method, "FLUSH") == 0) {
    if (is_owner) {
      // The audio task drops the buffered audio and shows the session as paused
      char rtp_info[96] = "";
      uint32_t seq = 0;
      if (get_header(request, header_length, "RTP-Info", rtp_info, sizeof(rtp_info)) &&
          get_numeric_parameter(rtp_info, "seq", &seq)) {
        this->flush_seq_.store(static_cast<uint16_t>(seq), std::memory_order_relaxed);
        this->flush_request_.store(FLUSH_REQUEST_FROM_SEQ, std::memory_order_release);
      } else {
        // Without a sequence number accept whatever arrives next
        this->flush_request_.store(FLUSH_REQUEST_ALL, std::memory_order_release);
      }
    }
  } else if (strcmp(method, "TEARDOWN") == 0) {
    if (is_owner) {
      ESP_LOGI(TAG, "Playback stopped");
      this->end_session_();
    }
    snprintf(extra_headers, sizeof(extra_headers), "Connection: close\r\n");
    keep_open = false;
  } else if (strcmp(method, "SET_PARAMETER") == 0) {
    char content_type[48] = "";
    get_header(request, header_length, "Content-Type", content_type, sizeof(content_type));
    if (is_owner && body != nullptr)
      this->handle_set_parameter_(content_type, body, body_length);
  } else if (strcmp(method, "GET_PARAMETER") == 0) {
    if (body != nullptr && strstr(body, "volume") != nullptr)
      snprintf(response_body, sizeof(response_body), "volume: %.6f\r\n", this->airplay_volume_);
  } else {
    status = 501;
    status_text = "Not Implemented";
  }

  // Remote control details of the sender that owns the session
  if (&connection == this->session_owner_) {
    char dacp_id[DACP_ID_SIZE];
    char active_remote[ACTIVE_REMOTE_SIZE] = "";
    if (get_header(request, header_length, "DACP-ID", dacp_id, sizeof(dacp_id))) {
      get_header(request, header_length, "Active-Remote", active_remote, sizeof(active_remote));
      this->update_remote_target_(dacp_id, active_remote, connection.peer_address);
    }
  }

  char challenge[64];
  const size_t extra_length = strlen(extra_headers);
  if (get_header(request, header_length, "Apple-Challenge", challenge, sizeof(challenge))) {
    if (!this->add_apple_response_(connection, challenge, extra_headers + extra_length,
                                   sizeof(extra_headers) - extra_length)) {
      ESP_LOGW(TAG, "Failed to answer the Apple-Challenge");
    }
  }

  char response[RESPONSE_SIZE];
  const size_t body_size = strlen(response_body);
  int length = snprintf(response, sizeof(response),
                        "RTSP/1.0 %d %s\r\n"
                        "CSeq: %s\r\n"
                        "Server: AirTunes/105.1\r\n"
                        "Audio-Jack-Status: connected; type=analog\r\n"
                        "%s",
                        status, status_text, cseq, extra_headers);
  if (body_size > 0 && length > 0 && static_cast<size_t>(length) < sizeof(response)) {
    length += snprintf(response + length, sizeof(response) - length,
                       "Content-Type: text/parameters\r\nContent-Length: %u\r\n", static_cast<unsigned>(body_size));
  }
  if (length > 0 && static_cast<size_t>(length) < sizeof(response))
    length += snprintf(response + length, sizeof(response) - length, "\r\n%s", response_body);
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(response)) {
    ESP_LOGW(TAG, "Response too large");
    return false;
  }
  if (!send_all(connection.fd, response, static_cast<size_t>(length)))
    return false;
  return keep_open;
}

bool AirPlayMediaSource::handle_announce_(const char *body, size_t body_length) {
  this->codec_ = Codec::NONE;
  this->encrypted_ = false;
  bool have_key = false;
  bool have_iv = false;
  bool key_error = false;
  ALACConfig alac;
  bool have_fmtp = false;

  const char *end = body + body_length;
  const char *line = body;
  while (line < end) {
    const char *line_end = line;
    while (line_end < end && *line_end != '\r' && *line_end != '\n')
      line_end++;
    const size_t length = static_cast<size_t>(line_end - line);

    if (length > 9 && strncmp(line, "a=rtpmap:", 9) == 0) {
      if (strstr(line, "AppleLossless") != nullptr && strstr(line, "AppleLossless") < line_end) {
        this->codec_ = Codec::ALAC;
      } else if (strstr(line, "L16/44100/2") != nullptr && strstr(line, "L16/44100/2") < line_end) {
        this->codec_ = Codec::PCM;
      }
    } else if (length > 7 && strncmp(line, "a=fmtp:", 7) == 0) {
      unsigned values[12];
      // payload_type frame_length version bit_depth pb mb kb channels max_run max_frame_bytes bit_rate sample_rate
      size_t count = 0;
      const char *p = line + 7;
      while (count < 12) {
        char *next = nullptr;
        const auto value = strtoul(p, &next, 10);
        if (next == p || next > line_end)
          break;
        values[count++] = static_cast<unsigned>(value);
        p = next;
      }
      if (count == 12) {
        alac.frame_length = values[1];
        alac.bit_depth = static_cast<uint8_t>(values[3]);
        alac.pb = static_cast<uint8_t>(values[4]);
        alac.mb = static_cast<uint8_t>(values[5]);
        alac.kb = static_cast<uint8_t>(values[6]);
        alac.num_channels = static_cast<uint8_t>(values[7]);
        alac.max_run = static_cast<uint16_t>(values[8]);
        alac.sample_rate = values[11];
        have_fmtp = true;
      }
    } else if (length > 12 && strncmp(line, "a=rsaaeskey:", 12) == 0) {
      uint8_t encrypted_key[RSA_SIGNATURE_SIZE];
      const size_t key_length = base64_decode_loose(line + 12, length - 12, encrypted_key, sizeof(encrypted_key));
      have_key = this->crypto_.decrypt_aes_key(encrypted_key, key_length, this->aes_key_);
      key_error = !have_key;
    } else if (length > 8 && strncmp(line, "a=aesiv:", 8) == 0) {
      have_iv =
          base64_decode_loose(line + 8, length - 8, this->aes_iv_, sizeof(this->aes_iv_)) == sizeof(this->aes_iv_);
    }

    line = line_end;
    while (line < end && (*line == '\r' || *line == '\n'))
      line++;
  }

  if (key_error) {
    ESP_LOGW(TAG, "Failed to decrypt the session key");
    this->codec_ = Codec::NONE;
    return false;
  }
  if (have_key != have_iv) {
    ESP_LOGW(TAG, "Incomplete encryption parameters");
    this->codec_ = Codec::NONE;
    return false;
  }
  this->encrypted_ = have_key;

  if (this->codec_ == Codec::ALAC) {
    if (!have_fmtp || alac.sample_rate != SAMPLE_RATE || alac.num_channels != NUM_CHANNELS ||
        !this->decoder_.configure(alac)) {
      ESP_LOGW(TAG, "Unsupported ALAC parameters");
      this->codec_ = Codec::NONE;
    }
  }
  if (this->codec_ == Codec::NONE) {
    ESP_LOGW(TAG, "Unsupported audio format");
    return false;
  }
  ESP_LOGD(TAG, "Stream: %s, %s", this->codec_ == Codec::ALAC ? LOG_STR_LITERAL("ALAC") : LOG_STR_LITERAL("PCM"),
           this->encrypted_ ? LOG_STR_LITERAL("encrypted") : LOG_STR_LITERAL("unencrypted"));
  return true;
}

bool AirPlayMediaSource::handle_setup_(RTSPConnection &connection, const char *transport, char *response_headers,
                                       size_t size) {
  uint32_t control_port = 0;
  uint32_t timing_port = 0;
  get_numeric_parameter(transport, "control_port", &control_port);
  get_numeric_parameter(transport, "timing_port", &timing_port);

  this->stop_audio_();
  this->close_udp_sockets_();

  uint16_t audio_port = 0;
  uint16_t local_control_port = 0;
  uint16_t local_timing_port = 0;
  this->audio_fd_ = open_udp_socket(&audio_port);
  this->control_fd_ = open_udp_socket(&local_control_port);
  this->timing_fd_ = open_udp_socket(&local_timing_port);
  if (this->audio_fd_ < 0 || this->control_fd_ < 0 || this->timing_fd_ < 0) {
    ESP_LOGW(TAG, "Failed to open UDP sockets");
    this->close_udp_sockets_();
    return false;
  }

  this->client_address_ = connection.peer_address;
  this->client_control_port_ = static_cast<uint16_t>(control_port);
  this->client_timing_port_ = static_cast<uint16_t>(timing_port);

  snprintf(response_headers, size,
           "Transport: RTP/AVP/UDP;unicast;mode=record;server_port=%u;control_port=%u;timing_port=%u\r\n"
           "Session: 1\r\n",
           audio_port, local_control_port, local_timing_port);
  return true;
}

#ifdef USE_AIRPLAY_METADATA
/// Copies a UTF-8 string of a DMAP item, shortened without cutting a character in half
static void copy_utf8(char *destination, size_t size, const uint8_t *source, size_t length) {
  size_t count = std::min(length, size - 1);
  if (count < length) {
    while (count > 0 && (source[count] & 0xc0) == 0x80)
      count--;
  }
  std::memcpy(destination, source, count);
  destination[count] = '\0';
}

/// Picks the track details out of DMAP data: items of a 4 character tag, a 32-bit length and the value
static void parse_dmap(const uint8_t *data, size_t length, char *title, char *artist, char *album, size_t size) {
  size_t position = 0;
  while (position + 8 <= length) {
    const uint8_t *tag = data + position;
    const uint32_t item_length = (static_cast<uint32_t>(tag[4]) << 24) | (static_cast<uint32_t>(tag[5]) << 16) |
                                 (static_cast<uint32_t>(tag[6]) << 8) | tag[7];
    position += 8;
    if (item_length > length - position)
      return;
    const uint8_t *value = data + position;
    if (std::memcmp(tag, "mlit", 4) == 0) {
      parse_dmap(value, item_length, title, artist, album, size);  // Container of the items below
    } else if (std::memcmp(tag, "minm", 4) == 0) {
      copy_utf8(title, size, value, item_length);
    } else if (std::memcmp(tag, "asar", 4) == 0) {
      copy_utf8(artist, size, value, item_length);
    } else if (std::memcmp(tag, "asal", 4) == 0) {
      copy_utf8(album, size, value, item_length);
    }
    position += item_length;
  }
}
#endif

void AirPlayMediaSource::handle_set_parameter_(const char *content_type, const char *body, size_t body_length) {
#ifdef USE_AIRPLAY_METADATA
  if (strncasecmp(content_type, "application/x-dmap-tagged", 25) == 0) {
    Metadata metadata{};
    parse_dmap(reinterpret_cast<const uint8_t *>(body), body_length, metadata.title, metadata.artist, metadata.album,
               METADATA_FIELD_SIZE);
    ESP_LOGD(TAG, "Now playing: %s - %s", metadata.artist, metadata.title);
    this->set_metadata_(metadata);
    return;
  }
#endif
  if (strncasecmp(content_type, "text/parameters", 15) != 0)
    return;  // Cover art and progress are not used

  const char *volume = strstr(body, "volume:");
  if (volume == nullptr)
    return;
  const float airplay_volume = strtof(volume + 7, nullptr);
  this->airplay_volume_ = airplay_volume;
  ESP_LOGD(TAG, "Volume: %.2f dB", airplay_volume);

  if (airplay_volume <= AIRPLAY_VOLUME_MUTE_LIMIT) {
    if (!this->airplay_muted_) {
      this->airplay_muted_ = true;
      this->request_mute_(true);
    }
    return;
  }
  const float level = std::clamp((airplay_volume - AIRPLAY_VOLUME_MIN) / -AIRPLAY_VOLUME_MIN, 0.0f, 1.0f);
  this->sender_volume_.store(level, std::memory_order_relaxed);
  this->request_volume_(level);
  if (this->airplay_muted_) {
    this->airplay_muted_ = false;
    this->request_mute_(false);
  }
}

bool AirPlayMediaSource::add_apple_response_(const RTSPConnection &connection, const char *challenge, char *output,
                                             size_t size) {
  static constexpr size_t CHALLENGE_SIZE = 16;
  uint8_t data[CHALLENGE_SIZE + 4 + sizeof(this->mac_)];
  size_t length = base64_decode_loose(challenge, strlen(challenge), data, CHALLENGE_SIZE);
  if (length != CHALLENGE_SIZE)
    return false;

  // The signed data is the challenge, the receiver's IP address on this connection and its MAC address
  struct sockaddr_in local {};
  socklen_t local_length = sizeof(local);
  if (getsockname(connection.fd, reinterpret_cast<struct sockaddr *>(&local), &local_length) != 0)
    return false;
  std::memcpy(data + length, &local.sin_addr.s_addr, 4);
  length += 4;
  std::memcpy(data + length, this->mac_, sizeof(this->mac_));
  length += sizeof(this->mac_);

  uint8_t signature[RSA_SIGNATURE_SIZE];
  if (!this->crypto_.sign_challenge(data, length, signature))
    return false;

  static constexpr char HEADER[] = "Apple-Response: ";
  static constexpr size_t HEADER_LENGTH = sizeof(HEADER) - 1;
  if (size < HEADER_LENGTH + 3)
    return false;
  std::memcpy(output, HEADER, HEADER_LENGTH);
  const size_t encoded =
      base64_encode_unpadded(signature, sizeof(signature), output + HEADER_LENGTH, size - HEADER_LENGTH - 2);
  if (encoded == 0)
    return false;
  std::memcpy(output + HEADER_LENGTH + encoded, "\r\n", 3);
  return true;
}

void AirPlayMediaSource::end_session_() {
  this->stop_audio_();
  this->close_udp_sockets_();
  this->session_owner_ = nullptr;
  this->codec_ = Codec::NONE;
  this->sender_volume_.store(-1.0f, std::memory_order_relaxed);
#ifdef USE_AIRPLAY_METADATA
  this->set_metadata_(Metadata{});
#endif
  if (this->airplay_muted_) {
    this->airplay_muted_ = false;
    this->request_mute_(false);
  }
  // Keep the remote control details for a while, so that a paused sender can be resumed from here
  xSemaphoreTake(this->remote_lock_, portMAX_DELAY);
  this->remote_target_.session_active = false;
  this->remote_target_.session_end_ms = millis();
  xSemaphoreGive(this->remote_lock_);
  this->set_session_state_(SESSION_STATE_NONE);
}

}  // namespace esphome::airplay

#endif  // USE_ESP32
