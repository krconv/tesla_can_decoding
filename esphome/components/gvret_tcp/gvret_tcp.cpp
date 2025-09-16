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
static constexpr uint8_t COMMAND_ENABLE_BINARY_OUTPUT = 0xE7;  // top-level switch to binary output

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
static inline bool in_isr_context() {
#if defined(xPortInIsrContext)
  return xPortInIsrContext();
#else
  return false;
#endif
}

void GvretTcpServer::setup() {
  rx_buf_.reserve(512);
}

void GvretTcpServer::loop() {
  // Time budget per loop iteration to avoid starving the scheduler
  const uint32_t LOOP_BUDGET_US = 4000;  // ~4ms
  uint32_t loop_start = uptime_us_();

  if (server_fd_ < 0) {
    if (global_wifi_component == nullptr || !global_wifi_component->is_connected()) {
      ESP_LOGI(TAG, "Server not started: WiFi not connected");
      return;
    }
    ESP_LOGI(TAG, "Starting GVRET TCP server on port %u", port_);
    start_server_();
    return;
  }
  if (client_fd_ < 0) { accept_client_(); }

  // flush TX queue (bounded)
  size_t tx_sent = 0;
  while (client_fd_ >= 0) {
    canbus::CanFrame f;
    {
      std::lock_guard<std::mutex> lk(q_mutex_);
      if (tx_queue_.empty()) break;
      f = tx_queue_.front();
      tx_queue_.pop();
    }
    std::vector<uint8_t> rec;
    encode_frame_(f, rec);
    send_record_(rec.data(), rec.size());
    tx_sent++;
  }

  // read + parse
  if (client_fd_ >= 0) {
    uint8_t buf[128];
    ssize_t rlen = 0;
    if (recv_bytes_(buf, sizeof(buf), rlen) && rlen > 0) {
      rx_buf_.insert(rx_buf_.end(), buf, buf + rlen);

      size_t parsed = 0;
      while (!rx_buf_.empty()) {
        // Enable binary output mode
        if (rx_buf_[0] == COMMAND_ENABLE_BINARY_OUTPUT) { 
          ESP_LOGD(TAG, "CMD: COMMAND_ENABLE_BINARY_OUTPUT (enable binary output)");
          rx_buf_.erase(rx_buf_.begin());
          binary_mode_ = true;
          continue; 
        }

        if (rx_buf_[0] != GVRET_HEADER || rx_buf_.size() < 2) break;
        uint8_t cmd = rx_buf_[1];
        ESP_LOGI(TAG, "CMD: %s (0x%02X)", command_name(cmd), cmd);

        if (cmd == static_cast<uint8_t>(Command::COMMAND_BUILD_CAN_FRAME)) {  // frame record (19 bytes)
          if (rx_buf_.size() < 19) break;

          canbus::CanFrame f{};
          // 19B layout (current test): [F1][00][TS LE 4][ID LE 4][DLC][DATA 8]
          // Timestamp (bytes 2..5) is ignored here
          uint32_t can_id = (uint32_t)rx_buf_[6] |
                            ((uint32_t)rx_buf_[7] << 8)  |
                            ((uint32_t)rx_buf_[8] << 16) |
                            ((uint32_t)rx_buf_[9] << 24);
          uint8_t dlc = rx_buf_[10];
          bool ext = (can_id > 0x7FF);
          f.use_extended_id = ext;
          f.can_id = can_id & (ext ? 0x1FFFFFFF : 0x7FF);
          f.can_data_length_code = dlc;
          f.remote_transmission_request = false;

          for (uint8_t i = 0; i < dlc && i < 8; i++) f.data[i] = rx_buf_[11 + i];

          on_transmit_.fire(f);
          rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 19);
          continue;
        }

        // Control commands that require responses
        if (handle_control_command_(cmd)) { rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 2); continue; }

        // Unknown F1 command -> drop full command header (2 bytes)
        ESP_LOGW(TAG, "Unknown F1 command 0x%02X, skipping", cmd);
        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 2);
        parsed++;
        if (parsed >= 32) break;  // cap parsed items
        if (uptime_us_() - loop_start > LOOP_BUDGET_US) break;  // honor time budget
      }
    }
  }
}

void GvretTcpServer::dump_config() {
  ESP_LOGCONFIG(TAG, "GVRET TCP Server on port %u (bus index %u)", port_, bus_index_);
}

void GvretTcpServer::forward_frame(const canbus::CanFrame &f) {
  // Only buffer when a client is connected
  if (client_fd_ < 0) return;

  bool in_isr = in_isr_context();
  {
    std::lock_guard<std::mutex> lk(q_mutex_);
    tx_queue_.push(f);
  }
}

// ---- networking ----

void GvretTcpServer::start_server_() {
  server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) { ESP_LOGW(TAG, "socket() failed: errno=%d", errno); return; }

  int yes = 1; setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port_);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(server_fd_, (sockaddr*)&addr, sizeof(addr)) < 0) { ESP_LOGW(TAG, "bind() failed: errno=%d", errno); close(server_fd_); server_fd_ = -1; return; }
  if (listen(server_fd_, 1) < 0) { ESP_LOGW(TAG, "listen() failed: errno=%d", errno); close(server_fd_); server_fd_ = -1; return; }

  int flags = fcntl(server_fd_, F_GETFL, 0);
  fcntl(server_fd_, F_SETFL, flags | O_NONBLOCK);
}

void GvretTcpServer::accept_client_() {
  sockaddr_in caddr{}; socklen_t clen = sizeof(caddr);
  int cfd = accept(server_fd_, (sockaddr*)&caddr, &clen);
  if (cfd >= 0) {
    int flags = fcntl(cfd, F_GETFL, 0);
    fcntl(cfd, F_SETFL, flags | O_NONBLOCK);
    client_fd_ = cfd;
    rx_buf_.clear();
    ESP_LOGI(TAG, "Client connected");
  }
}

void GvretTcpServer::close_client_() {
  if (client_fd_ >= 0) { 
    ESP_LOGI(TAG, "Client disconnected"); 
    close(client_fd_); 
    client_fd_ = -1; 
    binary_mode_ = false;
    rx_buf_.clear();
    // Drop any buffered frames on disconnect
    {
      std::lock_guard<std::mutex> lk(q_mutex_);
      std::queue<canbus::CanFrame> empty;
      std::swap(tx_queue_, empty);
    }
  }
}

void GvretTcpServer::send_record_(const uint8_t *data, size_t len) {
  if (client_fd_ < 0) return;
  send(client_fd_, data, len, 0);
}

bool GvretTcpServer::recv_bytes_(uint8_t *buf, size_t maxlen, ssize_t &out_len) {
  out_len = 0;
  if (client_fd_ < 0) return false;
  ssize_t r = recv(client_fd_, buf, maxlen, 0);
  if (r > 0) { out_len = r; return true; }
  if (r == 0) { close_client_(); }
  return false;
}

bool GvretTcpServer::handle_control_command_(uint8_t cmd) {
  switch (static_cast<Command>(cmd)) {
    case Command::COMMAND_KEEP_ALIVE: {
      const uint8_t msg[4] = {GVRET_HEADER, 0x09, 0xDE, 0xAD};
      send_record_(msg, sizeof(msg));
      return true;
    }
    case Command::COMMAND_TIME_SYNC: {
      uint32_t ts = uptime_us_();
      const uint8_t msg[6] = {GVRET_HEADER, 0x01, (uint8_t) ts, (uint8_t) (ts >> 8), (uint8_t) (ts >> 16), (uint8_t) (ts >> 24)};
      send_record_(msg, sizeof(msg));
      return true;
    }
    case Command::COMMAND_GET_DEVICE_INFO: {
      // [F1][07][build_lo][build_hi][eeprom_ver][fileOutputType][autoStartLogging][singleWire_Enabled]
      const uint8_t build_lo = 0x01, build_hi = 0x00;
      const uint8_t eeprom_ver = 0x01;
      const uint8_t file_output_type = binary_mode_ ? 0x01 : 0x00; // 0=GVRET CSV, 1=Binary
      const uint8_t auto_start_logging = 0x00;
      const uint8_t singlewire_enabled = 0x00;
      const uint8_t msg[8] = {GVRET_HEADER, 0x07, build_lo, build_hi, eeprom_ver, file_output_type, auto_start_logging, singlewire_enabled};
      send_record_(msg, sizeof(msg));
      return true;
    }
    case Command::COMMAND_GET_CAN_BUS_PARAMETERS: {
      // 12-byte reply covering CAN0 and CAN1
      const uint8_t can0_enabled = 1, can0_listen_only = 0;
      const uint8_t can1_enabled = 0, can1_listen_only = 0, sw_enabled = 0;
      const uint32_t can0_speed = 500000; // adjust if available from config
      const uint32_t can1_speed = 0;
      const uint8_t msg[12] = {
        GVRET_HEADER, 0x06,
        (uint8_t)(can0_enabled | (can0_listen_only << 4)),
        (uint8_t)(can0_speed & 0xFF), (uint8_t)(can0_speed >> 8), (uint8_t)(can0_speed >> 16), (uint8_t)(can0_speed >> 24),
        (uint8_t)(can1_enabled | (can1_listen_only << 4) | (sw_enabled << 6)),
        (uint8_t)(can1_speed & 0xFF), (uint8_t)(can1_speed >> 8), (uint8_t)(can1_speed >> 16), (uint8_t)(can1_speed >> 24)
      };
      send_record_(msg, sizeof(msg));
      return true;
    }
    case Command::COMMAND_GET_NUMBER_OF_BUSES: {
      const uint8_t msg[3] = {GVRET_HEADER, 0x0C, 0x01};
      send_record_(msg, sizeof(msg));
      return true;
    }
    default:
      return false;
  }
}

void GvretTcpServer::encode_frame_(const canbus::CanFrame &f, std::vector<uint8_t>& out) {
  out.clear();
  if (binary_mode_) {
    // GVRET framed record with header:
    // [F1][00][TS LE 4][ID LE 4][DLC][DATA...]
    uint32_t ts = uptime_us_();
    uint32_t id = (f.use_extended_id ? (f.can_id & 0x1FFFFFFF) : (f.can_id & 0x7FF));
    uint8_t dlc = f.can_data_length_code; if (dlc > 8) dlc = 8;

    out.reserve(2 + 4 + 4 + 1 + dlc);
    out.push_back(GVRET_HEADER);
    out.push_back(static_cast<uint8_t>(Command::COMMAND_BUILD_CAN_FRAME));
    // Timestamp LE
    out.push_back(ts & 0xFF);
    out.push_back((ts >> 8) & 0xFF);
    out.push_back((ts >> 16) & 0xFF);
    out.push_back((ts >> 24) & 0xFF);
    // CAN ID LE
    out.push_back(id & 0xFF);
    out.push_back((id >> 8) & 0xFF);
    out.push_back((id >> 16) & 0xFF);
    out.push_back((id >> 24) & 0xFF);
    // DLC
    out.push_back(dlc);
    // Data
    for (uint8_t i = 0; i < dlc && i < 8; i++) out.push_back(f.data[i]);
  } else {
    // Text GVRET CSV: "millis,id,ext,bus,len[,data...]\r\n"
    char line[128];
    uint32_t ms = uptime_us_() / 1000;
    int n = snprintf(line, sizeof(line), "%u,%x,%u,%u,%u", ms, (unsigned) f.can_id, (unsigned) (f.use_extended_id ? 1 : 0), (unsigned) bus_index_, (unsigned) f.can_data_length_code);
    if (n < 0) n = 0; if (n > (int) sizeof(line)) n = sizeof(line);
    out.insert(out.end(), line, line + n);
    for (uint8_t i = 0; i < f.can_data_length_code && i < 8; i++) {
      int m = snprintf(line, sizeof(line), ",%x", (unsigned) f.data[i]);
      if (m < 0) m = 0; if (m > (int) sizeof(line)) m = sizeof(line);
      out.insert(out.end(), line, line + m);
    }
    out.push_back('\r');
    out.push_back('\n');
  }
}

} // namespace gvret_tcp
} // namespace esphome
