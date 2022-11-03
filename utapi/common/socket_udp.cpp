/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#include "socket_udp.h"
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include "linuxcvl.h"
#include "print.h"

SocketUdp::SocketUdp(char *ip, int port, int rxque_max, SerialDecode *decode, int rxlen_max, int priority) {
  fp_ = LinuxCvl::socketudp_init((char *)" ", 0, 0);
  if (fp_ <= -1) {
    is_error_ = true;
    return;
  }

  addr_ = LinuxCvl::get_sockaddr(ip, port);

  is_error_ = false;
  rx_que_ = new BlockDeque<serial_stream_t>(rxque_max);
  decode_ = decode;
  rxlen_max_ = rxlen_max;
  if (decode_ == NULL)
    is_decode_ = false;
  else
    is_decode_ = true;

  flush();
  recv_task_ = new RtPeriodicMemberFun<SocketUdp>(0, "recv_task", 1024 * 1024, priority, &SocketUdp::recv_proc, this);
  recv_task_->start();
}

SocketUdp::~SocketUdp(void) {
  is_error_ = true;
  delete recv_task_;
  delete rx_que_;
}

bool SocketUdp::is_error(void) { return is_error_; }

void SocketUdp::close_port(void) {
  is_error_ = true;
  close(fp_);
}

void SocketUdp::flush(bool is_decode) {
  is_decode_ = is_decode;
  rx_que_->flush();
  if (decode_ != NULL && is_decode_) decode_->flush();
}

void SocketUdp::flush(int slave_id, int master_id, int rxlen_max) {
  is_decode_ = true;
  rx_que_->flush();
  if (decode_ != NULL) decode_->flush(slave_id, master_id, rxlen_max);
}

int SocketUdp::write_frame(serial_stream_t *data) {
  if (is_error_) return -1;
  int ret = LinuxCvl::socketudp_send_data(fp_, addr_, data->data, data->len);
  // Print::hex("[Sock UDP] write: ", data->data, data->len);
  return ret;
}

int SocketUdp::read_frame(serial_stream_t *data, float timeout_s) {
  if (is_error_) return -1;
  return rx_que_->pop(data, timeout_s);
}

void SocketUdp::recv_proc(void) {
  socklen_t addr_len = sizeof(addr_);
  while (is_error_ == false) {
    bzero(rx_stream_.data, rxlen_max_);
    rx_stream_.len = recvfrom(fp_, rx_stream_.data, rxlen_max_, 0, (struct sockaddr *)&addr_, &addr_len);
    if (rx_stream_.len <= 0) {
      close(fp_);
      printf("[SockeTcp] recv_proc exit\n");
      pthread_exit(0);
      return;
    }

    if (decode_ != NULL && is_decode_) {
      decode_->parse_put(rx_stream_.data, rx_stream_.len, rx_que_);
    } else {
      rx_que_->push_back(&rx_stream_);
    }
    // Print::hex("[Sock UDP] recv: ", rx_stream_.data, rx_stream_.len);
  }
}