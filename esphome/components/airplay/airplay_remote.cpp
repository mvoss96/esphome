#include "airplay_media_source.h"

#ifdef USE_ESP32

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <mdns.h>

#include <cstring>

namespace esphome::airplay {

ESPHOME_LOG_TAG(TAG, "airplay.remote");

static constexpr uint32_t DISCOVERY_TIMEOUT_MS = 2000;
static constexpr size_t DISCOVERY_MAX_RESULTS = 10;
static constexpr uint32_t COMMAND_TIMEOUT_MS = 1500;
/// Commands still reach a sender for this long after it disconnected, e.g. to resume after a pause
static constexpr uint32_t SESSION_GRACE_MS = 15 * 60 * 1000;

void AirPlayMediaSource::remote_task(void *params) { static_cast<AirPlayMediaSource *>(params)->remote_task_loop_(); }

// THREAD CONTEXT: Any. Wakes the remote control task.
void AirPlayMediaSource::send_remote_request_(uint8_t request) {
  this->remote_requests_.fetch_or(request, std::memory_order_acq_rel);
  if (this->remote_task_.is_created())
    xTaskNotifyGive(this->remote_task_.get_handle());
}

// THREAD CONTEXT: Remote control task. Network calls here may block for a few seconds without holding up
// playback.
void AirPlayMediaSource::remote_task_loop_() {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    const uint8_t requests = this->remote_requests_.exchange(0, std::memory_order_acq_rel);
    if (requests == 0)
      continue;

    xSemaphoreTake(this->lock_, portMAX_DELAY);
    RemoteState remote = this->remote_;
    xSemaphoreGive(this->lock_);

    const bool commands = (requests & ~REMOTE_REQUEST_DISCOVER) != 0;
    if (remote.target.dacp_id[0] == '\0' || remote.target.active_remote[0] == '\0' ||
        (!remote.session_active && millis() - remote.session_end_ms > SESSION_GRACE_MS)) {
      if (commands) {
        ESP_LOGW(TAG, "No AirPlay sender with remote control support");
      }
      continue;
    }

    if (remote.port == 0) {
      if (!this->discover_remote_port_(remote))
        continue;
      xSemaphoreTake(this->lock_, portMAX_DELAY);
      if (std::strcmp(this->remote_.target.dacp_id, remote.target.dacp_id) == 0)
        this->remote_.port = remote.port;
      xSemaphoreGive(this->lock_);
    }

    struct Command {
      uint8_t request;
      const char *path;
    };
    static constexpr Command COMMANDS[] = {
        {REMOTE_REQUEST_PLAY, ::airplay::DACP_PLAY},
        {REMOTE_REQUEST_PAUSE, ::airplay::DACP_PAUSE},
        {REMOTE_REQUEST_NEXT, ::airplay::DACP_NEXT},
        {REMOTE_REQUEST_PREVIOUS, ::airplay::DACP_PREVIOUS},
    };
    char volume_command[64];
    const char *volume = nullptr;
    if ((requests & REMOTE_REQUEST_VOLUME) != 0 &&
        ::airplay::dacp_volume_command(this->remote_volume_.load(std::memory_order_relaxed), volume_command,
                                       sizeof(volume_command)) > 0) {
      volume = volume_command;
    }
    bool unreachable = false;
    for (const auto &command : COMMANDS) {
      if ((requests & command.request) != 0)
        unreachable |= ::airplay::send_dacp_command(remote.target, remote.port, command.path, COMMAND_TIMEOUT_MS) == 0;
    }
    if (volume != nullptr)
      unreachable |= ::airplay::send_dacp_command(remote.target, remote.port, volume, COMMAND_TIMEOUT_MS) == 0;

    if (unreachable) {
      // The port may have changed, so look it up again next time
      xSemaphoreTake(this->lock_, portMAX_DELAY);
      if (std::strcmp(this->remote_.target.dacp_id, remote.target.dacp_id) == 0)
        this->remote_.port = 0;
      xSemaphoreGive(this->lock_);
    }
  }
}

bool AirPlayMediaSource::discover_remote_port_(RemoteState &remote) {
  // The sender announces its remote control service as "iTunes_Ctrl_<DACP-ID>", or only "<DACP-ID>"
  mdns_result_t *results = nullptr;
  if (mdns_query_ptr(::airplay::DACP_SERVICE_TYPE, ::airplay::DACP_SERVICE_PROTOCOL, DISCOVERY_TIMEOUT_MS,
                     DISCOVERY_MAX_RESULTS, &results) == ESP_OK) {
    for (const mdns_result_t *result = results; result != nullptr; result = result->next) {
      if (result->instance_name != nullptr && result->port != 0 &&
          ::airplay::dacp_instance_matches(result->instance_name, remote.target.dacp_id)) {
        remote.port = result->port;
        break;
      }
    }
  }
  mdns_query_results_free(results);
  if (remote.port == 0) {
    ESP_LOGW(TAG, "Remote control service of the sender not found");
    return false;
  }
  ESP_LOGD(TAG, "Remote control service found on port %u", remote.port);
  return true;
}

}  // namespace esphome::airplay

#endif  // USE_ESP32
