#pragma once

#include "esphome/core/component.h"
#include "esphome/components/canbus/canbus.h"

#include <array>
#include <queue>
#include <vector>
#include <mutex>

extern "C" {
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <fcntl.h>
  #include <unistd.h>
  #include <errno.h>
}

extern "C" {
  #include "esp_timer.h"   // ESP-IDF micros
}
static inline uint32_t gvret_micros() {
  return (uint32_t)(esp_timer_get_time() & 0xFFFFFFFFULL);
}

namespace esphome {
namespace gvret_tcp {

class GvretOnTransmitTrigger : public Trigger<canbus::CanFrame> {
 public:
  void fire(const canbus::CanFrame &frame) { this->trigger(frame); }
};

class GvretTcpServer : public Component {
 public:
  void set_port(uint16_t port) { port_ = port; }
  void set_bus_index(uint8_t idx) { bus_index_ = idx; }
  GvretOnTransmitTrigger *get_on_transmit_trigger() { return &on_transmit_; }

  // Called from canbus.on_frame lambda
  void forward_frame(const canbus::CanFrame &frame);

  void setup() override;
  void loop() override;
  void dump_config() override;

 private:
  // networking
  void start_server_();
  void accept_client_();
  void close_client_();
  void send_record_(const uint8_t *data, size_t len);
  bool recv_bytes_(uint8_t *buff, size_t maxlen, ssize_t &out_len);

  bool handle_control_command_(uint8_t cmd, std::vector<uint8_t> &buffer);
  size_t encode_frame_(const canbus::CanFrame &f, uint8_t *out);
  static constexpr size_t MAX_BIN_RECORD = 2 + 4 + 4 + 1 + 8; // [F1][00][TS4][ID4][DLC][DATA8]
  static constexpr size_t MAX_CSV_RECORD = 32; // conservative upper bound per line

  uint32_t uptime_us_() const { return gvret_micros(); }

  uint16_t port_{23};
  uint8_t bus_index_{0};

  int server_fd_{-1};
  int client_fd_{-1};

  static constexpr size_t MAX_TX_QUEUE = 256;
  std::mutex q_mutex_;
  std::queue<canbus::CanFrame> tx_queue_;
  uint32_t tx_dropped_{0};
  uint32_t tx_enqueued_{0};
  uint32_t tx_sent_{0};
  uint32_t last_stats_us_{0};
  std::vector<uint8_t> rx_buf_;

  GvretOnTransmitTrigger on_transmit_;

  bool binary_mode_{false};
};

}  // namespace gvret_tcp
}  // na
