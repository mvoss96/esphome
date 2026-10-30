#include "airplay_media_source.h"

#ifdef USE_ESP32

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <fcntl.h>
#include <lwip/sockets.h>
#include <mdns.h>
#include <unistd.h>

#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace esphome::airplay {

ESPHOME_LOG_TAG(TAG, "airplay.remote");

static constexpr uint32_t DISCOVERY_TIMEOUT_MS = 2000;
static constexpr size_t DISCOVERY_MAX_RESULTS = 10;
static constexpr uint32_t COMMAND_TIMEOUT_MS = 1500;
/// Commands still reach a sender for this long after it disconnected, e.g. to resume after a pause
static constexpr uint32_t SESSION_GRACE_MS = 15 * 60 * 1000;

/// Case-insensitive substring search
static bool contains_ignore_case(const char *text, const char *needle) {
  const size_t needle_length = strlen(needle);
  for (; *text != '\0'; text++) {
    if (strncasecmp(text, needle, needle_length) == 0)
      return true;
  }
  return false;
}

void AirPlayMediaSource::remote_task(void *params) { static_cast<AirPlayMediaSource *>(params)->remote_task_loop_(); }

// THREAD CONTEXT: Any. Wakes the remote control task.
void AirPlayMediaSource::send_remote_request_(uint8_t request) {
  this->remote_requests_.fetch_or(request, std::memory_order_acq_rel);
  if (this->remote_task_.is_created())
    xTaskNotifyGive(this->remote_task_.get_handle());
}

// THREAD CONTEXT: RTSP task. Remote control details arrive as headers on every request of the session owner.
void AirPlayMediaSource::update_remote_target_(const char *dacp_id, const char *active_remote, uint32_t address) {
  bool changed = false;
  xSemaphoreTake(this->remote_lock_, portMAX_DELAY);
  RemoteTarget &target = this->remote_target_;
  if (strcmp(target.dacp_id, dacp_id) != 0 || target.address != address) {
    snprintf(target.dacp_id, sizeof(target.dacp_id), "%s", dacp_id);
    target.address = address;
    target.port = 0;
    changed = true;
  }
  if (active_remote[0] != '\0')
    snprintf(target.active_remote, sizeof(target.active_remote), "%s", active_remote);
  target.session_active = true;
  xSemaphoreGive(this->remote_lock_);
  // Look up the port now, so that the first command does not have to wait for it
  if (changed)
    this->send_remote_request_(REMOTE_REQUEST_DISCOVER);
}

// THREAD CONTEXT: Remote control task. Network calls here may block for a few seconds without holding up
// the RTSP server.
void AirPlayMediaSource::remote_task_loop_() {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    const uint8_t requests = this->remote_requests_.exchange(0, std::memory_order_acq_rel);
    if (requests == 0)
      continue;

    xSemaphoreTake(this->remote_lock_, portMAX_DELAY);
    RemoteTarget target = this->remote_target_;
    xSemaphoreGive(this->remote_lock_);

    const bool commands = (requests & ~REMOTE_REQUEST_DISCOVER) != 0;
    if (target.dacp_id[0] == '\0' || target.active_remote[0] == '\0' ||
        (!target.session_active && millis() - target.session_end_ms > SESSION_GRACE_MS)) {
      if (commands) {
        ESP_LOGW(TAG, "No AirPlay sender with remote control support");
      }
      continue;
    }

    if (target.port == 0) {
      if (!this->discover_remote_port_(target))
        continue;
      xSemaphoreTake(this->remote_lock_, portMAX_DELAY);
      if (strcmp(this->remote_target_.dacp_id, target.dacp_id) == 0)
        this->remote_target_.port = target.port;
      xSemaphoreGive(this->remote_lock_);
    }

    struct Command {
      uint8_t request;
      const char *path;
    };
    static constexpr Command COMMANDS[] = {
        {REMOTE_REQUEST_PLAY, "play"},
        {REMOTE_REQUEST_PAUSE, "pause"},
        {REMOTE_REQUEST_NEXT, "nextitem"},
        {REMOTE_REQUEST_PREVIOUS, "previtem"},
    };
    for (const auto &command : COMMANDS) {
      if ((requests & command.request) != 0)
        this->send_remote_command_(target, command.path);
    }
    if ((requests & REMOTE_REQUEST_VOLUME) != 0) {
      char path[64];
      snprintf(path, sizeof(path), "setproperty?dmcp.device-volume=%.6f",
               this->remote_volume_db_.load(std::memory_order_relaxed));
      this->send_remote_command_(target, path);
    }
  }
}

bool AirPlayMediaSource::discover_remote_port_(RemoteTarget &target) {
  // The sender announces its remote control service as "iTunes_Ctrl_<DACP-ID>", or only "<DACP-ID>"
  mdns_result_t *results = nullptr;
  if (mdns_query_ptr("_dacp", "_tcp", DISCOVERY_TIMEOUT_MS, DISCOVERY_MAX_RESULTS, &results) == ESP_OK) {
    for (const mdns_result_t *result = results; result != nullptr; result = result->next) {
      if (result->instance_name != nullptr && result->port != 0 &&
          contains_ignore_case(result->instance_name, target.dacp_id)) {
        target.port = result->port;
        break;
      }
    }
  }
  mdns_query_results_free(results);
  if (target.port == 0) {
    ESP_LOGW(TAG, "Remote control service of the sender not found");
    return false;
  }
  ESP_LOGD(TAG, "Remote control service found on port %u", target.port);
  return true;
}

bool AirPlayMediaSource::send_remote_command_(const RemoteTarget &target, const char *command) {
  const int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0)
    return false;
  struct timeval timeout {};
  timeout.tv_sec = COMMAND_TIMEOUT_MS / 1000;
  timeout.tv_usec = (COMMAND_TIMEOUT_MS % 1000) * 1000;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  // Connect without blocking for longer than the timeout
  const int flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  struct sockaddr_in address {};
  address.sin_family = AF_INET;
  address.sin_port = htons(target.port);
  address.sin_addr.s_addr = target.address;
  bool connected = connect(fd, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) == 0;
  if (!connected && errno == EINPROGRESS) {
    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(fd, &write_fds);
    int error = 0;
    socklen_t error_length = sizeof(error);
    connected = select(fd + 1, nullptr, &write_fds, nullptr, &timeout) > 0 &&
                getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &error_length) == 0 && error == 0;
  }
  fcntl(fd, F_SETFL, flags);

  int32_t status = 0;
  if (connected) {
    const auto *ip = reinterpret_cast<const uint8_t *>(&target.address);
    char request[256];
    const int length = snprintf(request, sizeof(request),
                                "GET /ctrl-int/1/%s HTTP/1.1\r\n"
                                "Host: %u.%u.%u.%u:%u\r\n"
                                "Active-Remote: %s\r\n"
                                "Connection: close\r\n\r\n",
                                command, ip[0], ip[1], ip[2], ip[3], target.port, target.active_remote);
    if (length > 0 && static_cast<size_t>(length) < sizeof(request) &&
        send(fd, request, static_cast<size_t>(length), 0) == length) {
      char reply[32];
      const ssize_t received = recv(fd, reply, sizeof(reply) - 1, 0);
      if (received > 12) {
        reply[received] = '\0';
        status = static_cast<int32_t>(strtol(reply + 9, nullptr, 10));
      }
    }
  }
  close(fd);

  if (status >= 200 && status < 300) {
    ESP_LOGD(TAG, "Sent '%s'", command);
    return true;
  }
  ESP_LOGW(TAG, "Remote command '%s' failed (%" PRId32 ")", command, status);
  if (status == 0) {
    // Not reachable: the port may have changed, so look it up again next time
    xSemaphoreTake(this->remote_lock_, portMAX_DELAY);
    if (strcmp(this->remote_target_.dacp_id, target.dacp_id) == 0)
      this->remote_target_.port = 0;
    xSemaphoreGive(this->remote_lock_);
  }
  return false;
}

}  // namespace esphome::airplay

#endif  // USE_ESP32
