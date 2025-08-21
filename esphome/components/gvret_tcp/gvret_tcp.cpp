#include "gvret_tcp.h"
#include "esphome/core/log.h"

namespace esphome {
namespace gvret_tcp {

static const char *const TAG = "gvret_tcp";

void GvretTcpServer::setup() {
  this->rx_buf_.reserve(512);
  start_server_();
}

void GvretTcpServer::loop() {
  if (server_fd_ < 0) { start_server_(); return; }
  if (client_fd_ < 0) { accept_client_(); }

  // send any queued CAN frames
  while (!tx_queue_.empty() && client_fd_ >= 0) {
    auto rec = tx_queue_.front();
    tx_queue_.pop();
    send_record_(rec.data(), rec.size());
  }

  // read incoming
  if (client_fd_ >= 0) {
    uint8_t buf[128];
    ssize_t rlen = 0;
    if (recv_bytes_(buf, sizeof(buf), rlen) && rlen > 0) {
      rx_buf_.insert(rx_buf_.end(), buf, buf + rlen);
      while (rx_buf_.size() >= 2) {
        if (rx_buf_[0] == 0xE7) { rx_buf_.erase(rx_buf_.begin()); continue; }  // binary mode
        if (rx_buf_[0] != 0xF1) { rx_buf_.erase(rx_buf_.begin()); continue; }

        uint8_t cmd = rx_buf_[1];
        if (cmd == 0x00 && rx_buf_.size() >= 22) {
          // frame from host
          canbus::CanFrame f{};
          f.can_id = (uint32_t)rx_buf_[3] | ((uint32_t)rx_buf_[4] << 8) |
                     ((uint32_t)rx_buf_[5] << 16) | ((uint32_t)rx_buf_[6] << 24);
          f.data_length = rx_buf_[7];
          uint16_t flags = (uint16_t)rx_buf_[8] | ((uint16_t)rx_buf_[9] << 8);
          f.extd = flags & 0x1;
          f.rtr  = flags & 0x2;
          for (uint8_t i=0; i<f.data_length && i<8; i++) f.data[i] = rx_buf_[14+i];
          on_transmit_.fire(f);
          rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin()+22);
        } else if (cmd == 0x09) { reply_heartbeat_(); rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin()+2); }
        else if (cmd == 0x01) { reply_time_sync_(); rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin()+2); }
        else if (cmd == 0x07) { reply_device_info_(); rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin()+2); }
        else if (cmd == 0x06) { reply_bus_config_(); rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin()+2); }
        else { rx_buf_.erase(rx_buf_.begin()); }
      }
    }
  }
}

void GvretTcpServer::dump_config() {
  ESP_LOGCONFIG(TAG, "GVRET TCP Server on port %u (bus index %u)", port_, bus_index_);
}

void GvretTcpServer::forward_frame(const canbus::CanFrame &frame) {
  std::array<uint8_t, 22> rec;
  encode_frame_(frame, rec);
  std::lock_guard<std::mutex> lk(q_mutex_);
  tx_queue_.push(rec);
}

// --- networking helpers ---

void GvretTcpServer::start_server_() {
  server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) return;
  int yes=1; setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  sockaddr_in addr{}; addr.sin_family=AF_INET; addr.sin_port=htons(port_); addr.sin_addr.s_addr=htonl(INADDR_ANY);
  if (bind(server_fd_, (sockaddr*)&addr, sizeof(addr))<0) { close(server_fd_); server_fd_=-1; return; }
  listen(server_fd_,1);
  fcntl(server_fd_,F_SETFL,fcntl(server_fd_,F_GETFL,0)|O_NONBLOCK);
}

void GvretTcpServer::accept_client_() {
  sockaddr_in caddr{}; socklen_t clen=sizeof(caddr);
  int cfd=accept(server_fd_,(sockaddr*)&caddr,&clen);
  if (cfd>=0) { fcntl(cfd,F_SETFL,fcntl(cfd,F_GETFL,0)|O_NONBLOCK); client_fd_=cfd; rx_buf_.clear(); }
}

void GvretTcpServer::close_client_() {
  if (client_fd_>=0) { close(client_fd_); client_fd_=-1; rx_buf_.clear(); }
}

void GvretTcpServer::send_record_(const uint8_t *data,size_t len) {
  if (client_fd_<0) return;
  send(client_fd_,data,len,0);
}

bool GvretTcpServer::recv_bytes_(uint8_t *buf,size_t maxlen,ssize_t &out_len) {
  if (client_fd_<0) return false;
  ssize_t r=recv(client_fd_,buf,maxlen,0);
  if (r>0){out_len=r; return true;}
  if (r==0){close_client_();}
  return false;
}

// --- GVRET records ---

void GvretTcpServer::encode_frame_(const canbus::CanFrame &f,std::array<uint8_t,22>&out) {
  for(auto &b:out) b=0;
  out[0]=0xF1; out[1]=0x00; out[2]=bus_index_;
  out[3]=f.can_id&0xFF; out[4]=(f.can_id>>8)&0xFF; out[5]=(f.can_id>>16)&0xFF; out[6]=(f.can_id>>24)&0xFF;
  out[7]=f.data_length;
  uint16_t flags=(f.extd?1:0)|(f.rtr?2:0);
  out[8]=flags&0xFF; out[9]=flags>>8;
  uint32_t ts=uptime_us_();
  out[10]=ts&0xFF; out[11]=(ts>>8)&0xFF; out[12]=(ts>>16)&0xFF; out[13]=(ts>>24)&0xFF;
  for(uint8_t i=0;i<f.data_length&&i<8;i++) out[14+i]=f.data[i];
}

// --- Replies ---

void GvretTcpServer::reply_heartbeat_() {
  uint8_t msg[4]={0xF1,0x09,0xDE,0xAD}; send_record_(msg,sizeof(msg));
}

void GvretTcpServer::reply_time_sync_() {
  uint32_t ts=uptime_us_();
  uint8_t msg[6]={0xF1,0x01,(uint8_t)ts,(uint8_t)(ts>>8),(uint8_t)(ts>>16),(uint8_t)(ts>>24)};
  send_record_(msg,sizeof(msg));
}

void GvretTcpServer::reply_device_info_() {
  const char name[]="ESPHome-GVRET";
  std::vector<uint8_t> msg={0xF1,0x07,0x02,1,0}; // version=2, nbuses=1, features=0
  msg.insert(msg.end(),name,name+sizeof(name));
  send_record_(msg.data(),msg.size());
}

void GvretTcpServer::reply_bus_config_() {
  uint32_t bitrate=500000; // adjust to your YAML
  uint8_t msg[1+1+1+4+1]={0xF1,0x06,0x00,
    (uint8_t)bitrate,(uint8_t)(bitrate>>8),(uint8_t)(bitrate>>16),(uint8_t)(bitrate>>24),
    0x00}; // listen_only=false
  send_record_(msg,sizeof(msg));
}

} // namespace gvret_tcp
} // namespace esphome
