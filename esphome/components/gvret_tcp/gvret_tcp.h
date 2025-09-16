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
  bool recv_bytes_(uint8_t *buf, size_t maxlen, ssize_t &out_len);

  // gvret
  // Encode frame according to current mode.
  // Binary mode: [TS LE 4][ID LE 4 (bit31=ext)][DLC|(bus<<4)][DATA]
  // Text mode:   "millis,id,ext,bus,len[,data...]\r\n"
  void encode_frame_(const canbus::CanFrame &f, std::vector<uint8_t> &out);
  void reply_heartbeat_();    // F1 09 -> F1 09 DE AD
  void reply_time_sync_();    // F1 01 -> F1 01 <u32 micros>
  void reply_device_info_();  // F1 07 -> F1 07 <meta + "ESPHome-GVRET\0">
  void reply_bus_config_();   // F1 06 -> F1 06 <bitrate…>

  uint32_t uptime_us_() const { return gvret_micros(); }

  uint16_t port_{23};
  uint8_t bus_index_{0};

  int server_fd_{-1};
  int client_fd_{-1};

  std::mutex q_mutex_;
  std::queue<std::vector<uint8_t>> tx_queue_;
  std::vector<uint8_t> rx_buf_;

  GvretOnTransmitTrigger on_transmit_;

  bool binary_mode_{false};
};

}  // namespace gvret_tcp
}  // na
