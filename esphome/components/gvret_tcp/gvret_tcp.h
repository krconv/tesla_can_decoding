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

  // Forward CAN frame → SavvyCAN
  void forward_frame(const canbus::CanFrame &frame);

  void setup() override;
  void loop() override;
  void dump_config() override;

 protected:
  void start_server_();
  void accept_client_();
  void close_client_();

  void send_record_(const uint8_t *data, size_t len);
  bool recv_bytes_(uint8_t *buf, size_t maxlen, ssize_t &out_len);

  void encode_frame_(const canbus::CanFrame &f, std::array<uint8_t, 22> &out);
  void handle_incoming_(const uint8_t *buf, size_t len);

  void reply_heartbeat_();
  void reply_time_sync_();
  void reply_device_info_();
  void reply_bus_config_();

  uint32_t uptime_us_() const { return (uint32_t)(micros() & 0xFFFFFFFFUL); }

  uint16_t port_{23};
  uint8_t bus_index_{0};

  int server_fd_{-1};
  int client_fd_{-1};

  std::mutex q_mutex_;
  std::queue<std::array<uint8_t, 22>> tx_queue_;
  std::vector<uint8_t> rx_buf_;

  GvretOnTransmitTrigger on_transmit_;
};

}  // namespace gvret_tcp
}  // namespace esphome
