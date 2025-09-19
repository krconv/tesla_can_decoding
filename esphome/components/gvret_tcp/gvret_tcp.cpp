#include "gvret_tcp.h"
#include "esphome/core/log.h"
#include "esphome/components/wifi/wifi_component.h"
#include <cstdio>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
}

using esphome::wifi::global_wifi_component;

namespace esphome {
namespace gvret_tcp {

static const char *const TAG = "gvret_tcp";
static constexpr uint8_t GVRET_HEADER = 0xF1;

// GVRET protocol commands (under 0xF1 header)
enum class Command : uint8_t {
  COMMAND_BUILD_CAN_FRAME = 0,
  COMMAND_TIME_SYNC = 1,
  COMMAND_DIGITAL_INPUTS = 2,
  COMMAND_ANALOG_INPUTS = 3,
  COMMAND_SET_DIGITAL_OUTPUT = 4,
  COMMAND_SETUP_CAN_BUS = 5,
  COMMAND_GET_CAN_BUS_PARAMETERS = 6,
  COMMAND_GET_DEVICE_INFO = 7,
  COMMAND_SET_SOFTWARE_MODE = 8,
  COMMAND_KEEP_ALIVE = 9,
  COMMAND_SET_SYSTEM_TYPE = 10,
  COMMAND_ECHO_CAN_FRAME = 11,
  COMMAND_GET_NUMBER_OF_BUSES = 12,
  COMMAND_GET_EXTERNAL_BUSES = 13,
  COMMAND_SET_EXTERNAL_BUSES = 14,
  COMMAND_ENABLE_BINARY_OUTPUT = 0xE7
};

static inline const char *command_name(uint8_t raw) {
  switch (static_cast<Command>(raw)) {
    case Command::COMMAND_BUILD_CAN_FRAME: return "COMMAND_BUILD_CAN_FRAME";
    case Command::COMMAND_TIME_SYNC: return "COMMAND_TIME_SYNC";
    case Command::COMMAND_DIGITAL_INPUTS: return "COMMAND_DIGITAL_INPUTS";
    case Command::COMMAND_ANALOG_INPUTS: return "COMMAND_ANALOG_INPUTS";
    case Command::COMMAND_SET_DIGITAL_OUTPUT: return "COMMAND_SET_DIGITAL_OUTPUT";
    case Command::COMMAND_SETUP_CAN_BUS: return "COMMAND_SETUP_CAN_BUS";
    case Command::COMMAND_GET_CAN_BUS_PARAMETERS: return "COMMAND_GET_CAN_BUS_PARAMETERS";
    case Command::COMMAND_GET_DEVICE_INFO: return "COMMAND_GET_DEVICE_INFO";
    case Command::COMMAND_SET_SOFTWARE_MODE: return "COMMAND_SET_SOFTWARE_MODE";
    case Command::COMMAND_KEEP_ALIVE: return "COMMAND_KEEP_ALIVE";
    case Command::COMMAND_SET_SYSTEM_TYPE: return "COMMAND_SET_SYSTEM_TYPE";
    case Command::COMMAND_ECHO_CAN_FRAME: return "COMMAND_ECHO_CAN_FRAME";
    case Command::COMMAND_GET_NUMBER_OF_BUSES: return "COMMAND_GET_NUMBER_OF_BUSES";
    case Command::COMMAND_GET_EXTERNAL_BUSES: return "COMMAND_GET_EXTERNAL_BUSES";
    case Command::COMMAND_SET_EXTERNAL_BUSES: return "COMMAND_SET_EXTERNAL_BUSES";
    default: return "UNKNOWN";
  }
}

void GvretTcpServer::setup() {
  rx_buf_.reserve(512);
}

void GvretTcpServer::loop() {
  if (server_fd_ < 0) {
    if (global_wifi_component == nullptr || !global_wifi_component->is_connected()) {
      ESP_LOGI(TAG, "Server not started: WiFi not connected");
      return;
    }
    ESP_LOGI(TAG, "Starting GVRET TCP server on port %u", port_);
    start_server_();
    return;
  }

  if (client_fd_ < 0) { 
    accept_client_(); 
  }

  while (client_fd_ >= 0) {
    uint8_t buffer[2048];
    size_t buffer_len = 0;
    size_t frames_in_batch = 0;

    for (;;) {
      canbus::CanFrame frame;
      {
        std::lock_guard<std::mutex> lk(q_mutex_);
        if (tx_queue_.empty()) {
          break;
        }
        frame = tx_queue_.front();
        tx_queue_.pop();
      }

      uint8_t rec[MAX_BIN_RECORD > MAX_CSV_RECORD ? MAX_BIN_RECORD : MAX_CSV_RECORD];
      size_t record_len = encode_frame_(frame, rec);
      if (buffer_len + record_len > sizeof(buffer)) {
        // Send current batch before adding more
        if (buffer_len > 0){
          send_record_(buffer, buffer_len);
        } 
        buffer_len = 0;
      }
      memcpy(&buffer[buffer_len], rec, record_len);
      buffer_len += record_len;
      frames_in_batch++;
    }

    if (buffer_len > 0) {
      send_record_(buffer, buffer_len);
      tx_sent_ += frames_in_batch;
    } else {
      break;
    }
  }

  if (client_fd_ >= 0) {
    uint8_t buffer[128];
    ssize_t len = 0;
    if (recv_bytes_(buffer, sizeof(buffer), len) && len > 0) {
      rx_buf_.insert(rx_buf_.end(), buffer, buffer + len);

      size_t parsed = 0;
      while (!rx_buf_.empty()) {
        if (rx_buf_[0] == static_cast<uint8_t>(Command::COMMAND_ENABLE_BINARY_OUTPUT)) {
          if (handle_control_command_(rx_buf_[0], rx_buf_)) {
            continue;
          }
        }

        if (rx_buf_[0] != GVRET_HEADER) {
          ESP_LOGW(TAG, "Malformed GVRET header 0x%02X dropped", rx_buf_[0]);
          rx_buf_.erase(rx_buf_.begin());
          continue;
        } 

        if (rx_buf_.size() < 2) {
          // partial command, wait for more data
          break;
        }

        uint8_t cmd = rx_buf_[1];

        if (handle_control_command_(cmd, rx_buf_)) {
          continue;
        }

        parsed++;
      }
    }
  }

  // Periodic TX stats logging
  uint32_t now = uptime_us_();
  if (now - last_stats_us_ >= 30000000U) { // ~30s
    size_t queue_size = 0;
    {
      std::lock_guard<std::mutex> lk(q_mutex_);
      queue_size = tx_queue_.size();
    }
    ESP_LOGI(TAG, "TX stats: sent=%u dropped=%u queued=%u", (unsigned) tx_sent_, (unsigned) tx_dropped_, (unsigned) queue_size);
    last_stats_us_ = now;
  }
}

void GvretTcpServer::dump_config() {
  ESP_LOGCONFIG(TAG, "GVRET TCP Server on port %u (bus index %u)", port_, bus_index_);
}

void GvretTcpServer::forward_frame(const canbus::CanFrame &f) {
  if (client_fd_ < 0) return;

  if (q_mutex_.try_lock()) {
    if (tx_queue_.size() >= MAX_TX_QUEUE) {
      tx_dropped_++;
    } else {
      tx_queue_.push(f);
      tx_enqueued_++;
    }
    q_mutex_.unlock();
  } else {
    tx_dropped_++;
  }
}

void GvretTcpServer::start_server_() {
  server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) { 
    ESP_LOGW(TAG, "socket() failed: errno=%d", errno); 
    return; 
  }

  int yes = 1; setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port_);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(server_fd_, (sockaddr*)&addr, sizeof(addr)) < 0) { 
    ESP_LOGW(TAG, "bind() failed: errno=%d", errno); 
    close(server_fd_); 
    server_fd_ = -1; 
    return; 
  }
  if (listen(server_fd_, 1) < 0) { 
    ESP_LOGW(TAG, "listen() failed: errno=%d", errno); 
    close(server_fd_); 
    server_fd_ = -1; 
    return; 
  }

  int flags = fcntl(server_fd_, F_GETFL, 0);
  fcntl(server_fd_, F_SETFL, flags | O_NONBLOCK);
}

void GvretTcpServer::accept_client_() {
  sockaddr_in caddr{}; socklen_t clen = sizeof(caddr);
  int client_fd = accept(server_fd_, (sockaddr*)&caddr, &clen);
  if (client_fd < 0) {
    return;
  }

  int flags = fcntl(client_fd, F_GETFL, 0);
  fcntl(client_fd, F_SETFL, flags | O_NONBLOCK);
  client_fd_ = client_fd;

  ESP_LOGI(TAG, "Client connected");
}

void GvretTcpServer::close_client_() {
  if (client_fd_ < 0) { 
    return;
  }

  close(client_fd_); 
  client_fd_ = -1; 
  binary_mode_ = false;

  {
    std::lock_guard<std::mutex> lk(q_mutex_);
    rx_buf_.clear();
    std::queue<canbus::CanFrame> empty;
    std::swap(tx_queue_, empty);
  }

  ESP_LOGI(TAG, "Client disconnected"); 
}

void GvretTcpServer::send_record_(const uint8_t *data, size_t len) {
  if (client_fd_ < 0) {
    return;
  } 
  send(client_fd_, data, len, 0);
}

bool GvretTcpServer::recv_bytes_(uint8_t *buf, size_t maxlen, ssize_t &out_len) {
  out_len = 0;
  if (client_fd_ < 0) {
    return false;
  } 

  ssize_t received = recv(client_fd_, buf, maxlen, 0);
  if (received > 0) { 
    out_len = received;
    return true; 
  } 
  if (received == 0) { 
    close_client_();
  }
  return false;
}

bool GvretTcpServer::handle_control_command_(uint8_t cmd, std::vector<uint8_t> &buffer) {
  switch (static_cast<Command>(cmd)) {
    case Command::COMMAND_BUILD_CAN_FRAME:
      constexpr size_t header_len = 8;  // 2 (command) + 4 (ID) + 1 (flags) + 1 (DLC)
      if (buffer.size() < header_len) {
        return true;
      }

      // Read ID (LE). bit31 may indicate extended frame.
      uint32_t raw = (uint32_t)buffer[2] |
                      ((uint32_t)buffer[3] << 8) |
                      ((uint32_t)buffer[4] << 16) |
                      ((uint32_t)buffer[5] << 24);
      bool ext = (raw & 0x80000000u) != 0;
      uint32_t id = raw & (ext ? 0x1FFFFFFFu : 0x7FFu);

      uint8_t data_len = buffer[7];
      size_t frame_len = header_len + (size_t) data_len + 1;
      if (buffer.size() < frame_len) {
        return true;
      }

      canbus::CanFrame f{};
      f.use_extended_id = ext;
      f.can_id = id;
      f.can_data_length_code = data_len;
      f.remote_transmission_request = false;
      for (uint8_t i = 0; i < data_len; i++) {
        f.data[i] = buffer[header_len + i];
      }

      on_transmit_.fire(f);
      buffer.erase(buffer.begin(), buffer.begin() + frame_len);
      return true;
    case Command::COMMAND_ENABLE_BINARY_OUTPUT:
      binary_mode_ = true;
      if (!buffer.empty()) buffer.erase(buffer.begin());
      return true; 
    case Command::COMMAND_KEEP_ALIVE: {
      const uint8_t msg[4] = {GVRET_HEADER, cmd, 0xDE, 0xAD};
      send_record_(msg, sizeof(msg));
      buffer.erase(buffer.begin(), buffer.begin() + 2);
      return true;
    }
    case Command::COMMAND_TIME_SYNC: {
      uint32_t ts = uptime_us_();
      const uint8_t msg[6] = {GVRET_HEADER, cmd, (uint8_t) ts, (uint8_t) (ts >> 8), (uint8_t) (ts >> 16), (uint8_t) (ts >> 24)};
      send_record_(msg, sizeof(msg));
      buffer.erase(buffer.begin(), buffer.begin() + 2);
      return true;
    }
    case Command::COMMAND_GET_DEVICE_INFO: {
      const uint8_t build_lo = 0x01, build_hi = 0x00;
      const uint8_t eeprom_ver = 0x01;
      const uint8_t file_output_type = binary_mode_ ? 0x01 : 0x00; // 0=GVRET CSV, 1=Binary
      const uint8_t auto_start_logging = 0x00;
      const uint8_t singlewire_enabled = 0x00;
      const uint8_t msg[8] = {GVRET_HEADER, cmd, build_lo, build_hi, eeprom_ver, file_output_type, auto_start_logging, singlewire_enabled};
      send_record_(msg, sizeof(msg));
      buffer.erase(buffer.begin(), buffer.begin() + 2);
      return true;
    }
    case Command::COMMAND_GET_CAN_BUS_PARAMETERS: {
      const uint8_t can0_enabled = 1, can0_listen_only = 0;
      const uint8_t can1_enabled = 0, can1_listen_only = 0, sw_enabled = 0;
      const uint32_t can0_speed = 500000; // TODO: adjust canbus config
      const uint32_t can1_speed = 0;
      const uint8_t msg[12] = {
        GVRET_HEADER, cmd,
        (uint8_t)(can0_enabled | (can0_listen_only << 4)),
        (uint8_t)(can0_speed & 0xFF), (uint8_t)(can0_speed >> 8), (uint8_t)(can0_speed >> 16), (uint8_t)(can0_speed >> 24),
        (uint8_t)(can1_enabled | (can1_listen_only << 4) | (sw_enabled << 6)),
        (uint8_t)(can1_speed & 0xFF), (uint8_t)(can1_speed >> 8), (uint8_t)(can1_speed >> 16), (uint8_t)(can1_speed >> 24)
      };
      send_record_(msg, sizeof(msg));
      buffer.erase(buffer.begin(), buffer.begin() + 2);
      return true;
    }
    case Command::COMMAND_GET_NUMBER_OF_BUSES: {
      const uint8_t msg[3] = {GVRET_HEADER, cmd, 0x01};
      send_record_(msg, sizeof(msg));
      buffer.erase(buffer.begin(), buffer.begin() + 2);
      return true;
    }
    default:
      ESP_LOGW(TAG, "Unknown command 0x%02X, skipping", cmd);
      if (buffer.size() >= 2)
        buffer.erase(buffer.begin(), buffer.begin() + 2);
      return false;
  }
}

size_t GvretTcpServer::encode_frame_(const canbus::CanFrame &f, uint8_t *out) {
  // Returns number of bytes written to out
  if (binary_mode_) {
    // [F1][00][TS LE 4][ID LE 4][DLC][DATA...]
    uint32_t ts = uptime_us_();
    uint32_t id = f.can_id;
    uint8_t dlc = f.can_data_length_code;
    size_t idx = 0;
    out[idx++] = GVRET_HEADER;
    out[idx++] = static_cast<uint8_t>(Command::COMMAND_BUILD_CAN_FRAME);
    out[idx++] = (uint8_t)(ts & 0xFF);
    out[idx++] = (uint8_t)((ts >> 8) & 0xFF);
    out[idx++] = (uint8_t)((ts >> 16) & 0xFF);
    out[idx++] = (uint8_t)((ts >> 24) & 0xFF);
    out[idx++] = (uint8_t)(id & 0xFF);
    out[idx++] = (uint8_t)((id >> 8) & 0xFF);
    out[idx++] = (uint8_t)((id >> 16) & 0xFF);
    out[idx++] = (uint8_t)((id >> 24) & 0xFF);
    out[idx++] = dlc;
    for (uint8_t i = 0; i < dlc && i < 8; i++) out[idx++] = f.data[i];
    return idx;
  } else {
    // CSV: millis,id,ext,bus,len[,data...]\r\n
    // To avoid snprintf on every byte, format into a small temp buffer
    char line[MAX_CSV_RECORD];
    uint32_t ms = uptime_us_() / 1000;
    int n = snprintf(line, sizeof(line), "%u,%x,%u,%u,%u",
                     ms, (unsigned) f.can_id, (unsigned) (f.use_extended_id ? 1 : 0),
                     (unsigned) bus_index_, (unsigned) f.can_data_length_code);
    if (n < 0) n = 0;
    memcpy(out, line, (size_t) n);
    size_t idx = (size_t) n;
    for (uint8_t i = 0; i < f.can_data_length_code && i < 8; i++) {
      int m = snprintf(line, sizeof(line), ",%x", (unsigned) f.data[i]);
      if (m < 0) m = 0;
      memcpy(out + idx, line, (size_t) m);
      idx += (size_t) m;
    }
    out[idx++] = '\r';
    out[idx++] = '\n';
    return idx;
  }
}

} // namespace gvret_tcp
} // namespace esphome
