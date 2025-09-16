#include "gvret_tcp.h"
#include "esphome/core/log.h"
#include "esphome/components/wifi/wifi_component.h"
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
}

using esphome::wifi::global_wifi_component;

namespace esphome {
namespace gvret_tcp {

static const char *const TAG = "gvret_tcp";
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
    std::array<uint8_t, 19> rec;
    {
      std::lock_guard<std::mutex> lk(q_mutex_);
      if (tx_queue_.empty()) break;
      rec = tx_queue_.front();
      tx_queue_.pop();
    }
    ESP_LOGI(TAG, "TX: sending 19B frame (queued)");
    send_record_(rec.data(), rec.size());
    tx_sent++;
    if (tx_sent >= 16) break;  // cap per iteration
    if (uptime_us_() - loop_start > LOOP_BUDGET_US) break;  // honor time budget
  }

  // read + parse
  if (client_fd_ >= 0) {
    uint8_t buf[128];
    ssize_t rlen = 0;
    if (recv_bytes_(buf, sizeof(buf), rlen) && rlen > 0) {
      ESP_LOGI(TAG, "RX: received %d bytes", (int) rlen);
      rx_buf_.insert(rx_buf_.end(), buf, buf + rlen);

      size_t parsed = 0;
      while (!rx_buf_.empty()) {
        // 0xE7 -> enable binary (no-op)
        if (rx_buf_[0] == 0xE7) { ESP_LOGI(TAG, "CMD: E7 (enable binary)"); rx_buf_.erase(rx_buf_.begin()); continue; }

        if (rx_buf_[0] != 0xF1 || rx_buf_.size() < 2) break;
        uint8_t cmd = rx_buf_[1];
        ESP_LOGI(TAG, "CMD: F1 %02X", cmd);

        if (cmd == 0x00) {  // frame record (19 bytes)
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

          ESP_LOGI(TAG, "Frame RX->CAN (19B): id=0x%08X dlc=%u ext=%d rtr=%d", f.can_id, f.can_data_length_code, f.use_extended_id, f.remote_transmission_request);
          on_transmit_.fire(f);
          rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 19);
          continue;
        }

        if (cmd == 0x09) { reply_heartbeat_();  rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 2); continue; }
        if (cmd == 0x01) { reply_time_sync_();  rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 2); continue; }
        if (cmd == 0x07) { reply_device_info_();rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 2); continue; }
        if (cmd == 0x06) { reply_bus_config_(); rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 2); continue; }

        // Unknown F1 command -> drop full command header (2 bytes)
        ESP_LOGI(TAG, "Unknown F1 command 0x%02X, skipping", cmd);
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
  static uint32_t ff_cnt = 0;
  ff_cnt++;
  // Drop 99.99% of frames for testing (keep 1 out of 10000)
  if ((ff_cnt % 500) != 0) return;

  std::array<uint8_t, 19> rec;
  encode_frame_(f, rec);
  bool in_isr = in_isr_context();
  ESP_LOGI(TAG, "forward_frame(sampled): isr=%d id=0x%08X dlc=%u ext=%d rtr=%d (count=%u)",
           in_isr ? 1 : 0, f.can_id, f.can_data_length_code, f.use_extended_id, f.remote_transmission_request, ff_cnt);
  {
    std::lock_guard<std::mutex> lk(q_mutex_);
    size_t before = tx_queue_.size();
    tx_queue_.push(rec);
    size_t after = tx_queue_.size();
    static uint32_t qlog = 0; qlog++;
    if ((qlog & 0x3F) == 0) {
      ESP_LOGI(TAG, "queued GVRET record: %u -> %u", (unsigned) before, (unsigned) after);
    }
  }
}

// ---- networking ----

void GvretTcpServer::start_server_() {
  server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) { ESP_LOGI(TAG, "socket() failed: errno=%d", errno); return; }

  int yes = 1; setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port_);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(server_fd_, (sockaddr*)&addr, sizeof(addr)) < 0) { ESP_LOGI(TAG, "bind() failed: errno=%d", errno); close(server_fd_); server_fd_ = -1; return; }
  if (listen(server_fd_, 1) < 0) { ESP_LOGI(TAG, "listen() failed: errno=%d", errno); close(server_fd_); server_fd_ = -1; return; }

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


    // Send a single debug record with recognizable pattern bytes:
    // [F1][00][00][00][01][01][02][02]...[09][09] (continues up to 19 bytes)
    // This helps reverse engineer field positions in external parsers.
    std::array<uint8_t, 19> dbg{};
    dbg[0] = 0xF1; dbg[1] = 0x00;
    for (size_t i = 2; i < dbg.size(); i++) {
      uint8_t v = (uint8_t)((i - 2) & 0xFF);
      dbg[i] = v;
    }
    ESP_LOGI(TAG, "Sending debug pattern frame on connect");
    send_record_(dbg.data(), dbg.size());
  }
}

void GvretTcpServer::close_client_() {
  if (client_fd_ >= 0) { ESP_LOGI(TAG, "Client disconnected"); close(client_fd_); client_fd_ = -1; rx_buf_.clear(); }
}

void GvretTcpServer::send_record_(const uint8_t *data, size_t len) {
  if (client_fd_ < 0) return;
  static uint32_t txlog = 0; txlog++;
  if ((txlog & 0x0F) == 0) {  // log every 16th send to reduce spam
    if (len >= 2 && data[0] == 0xF1) {
      ESP_LOGI(TAG, "TX CMD: F1 %02X (%u bytes)", data[1], (unsigned) len);
    } else {
      ESP_LOGI(TAG, "TX: %u bytes", (unsigned) len);
    }
  }
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

// ---- Encode (F1 00 record, 19 bytes) ----

void GvretTcpServer::encode_frame_(const canbus::CanFrame &f, std::array<uint8_t,19>& out) {
  for (auto &b : out) b = 0;

  out[0] = 0xF1; out[1] = 0x00;

  // Timestamp little-endian (bytes 2..5)
  uint32_t ts = uptime_us_();
  out[2] = ts & 0xFF; out[3] = (ts >> 8) & 0xFF; out[4] = (ts >> 16) & 0xFF; out[5] = (ts >> 24) & 0xFF;

  // [11:04:06][I][gvret_tcp:243]: Frame CAN->TX (19B): id=0x00000118 dlc=8 ext=0 rtr=0 ts=646704316


  // CAN ID little-endian (bytes 6..9). Mask to the correct width.
  uint32_t id = (f.use_extended_id ? (f.can_id & 0x1FFFFFFF) : (f.can_id & 0x7FF));
  out[6] = id & 0xFF; out[7] = (id >> 8) & 0xFF; out[8] = (id >> 16) & 0xFF; out[9] = (id >> 24) & 0xFF;

  uint8_t dlc = f.can_data_length_code;
  if (dlc > 8) dlc = 8;
  out[10] = dlc;

  for (uint8_t i = 0; i < dlc && i < 8; i++) out[11 + i] = f.data[i];
  ESP_LOGI(TAG, "Frame CAN->TX (19B): id=0x%08X dlc=%u ext=%d rtr=%d ts=%u", id, dlc, f.use_extended_id, f.remote_transmission_request, ts);
}

// ---- Replies ----

void GvretTcpServer::reply_heartbeat_() {
  const uint8_t msg[4] = {0xF1, 0x09, 0xDE, 0xAD};
  send_record_(msg, sizeof(msg));
}

void GvretTcpServer::reply_time_sync_() {
  uint32_t ts = uptime_us_();
  const uint8_t msg[6] = {0xF1, 0x01, (uint8_t)ts, (uint8_t)(ts >> 8), (uint8_t)(ts >> 16), (uint8_t)(ts >> 24)};
  send_record_(msg, sizeof(msg));
}

void GvretTcpServer::reply_device_info_() {
  static const char name[] = "ESPHome-GVRET";
  std::vector<uint8_t> msg = {0xF1, 0x07, 0x02, /*version*/ 1, /*nbuses*/ 1, /*features*/ 0};
  msg.insert(msg.end(), name, name + sizeof(name)); // include NUL
  send_record_(msg.data(), msg.size());
}

void GvretTcpServer::reply_bus_config_() {
  const uint32_t bitrate = 500000;  // match your YAML bit_rate
  const uint8_t msg[1 + 1 + 1 + 4 + 1] = {
      0xF1, 0x06, 0x00,
      (uint8_t)bitrate, (uint8_t)(bitrate >> 8), (uint8_t)(bitrate >> 16), (uint8_t)(bitrate >> 24),
      0x00  // listen_only=false
  };
  send_record_(msg, sizeof(msg));
}

} // namespace gvret_tcp
} // namespace esphome
