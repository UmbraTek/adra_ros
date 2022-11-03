/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __COMMON_SOCKET_UDP_H__
#define __COMMON_SOCKET_UDP_H__

#include <arpa/inet.h>
#include <pthread.h>
#include "periodic_rt.h"
#include "socket.h"


class SocketUdp : public Socket {
 public:
  SocketUdp(char* ip, int port, int rxque_max, SerialDecode* decode, int rxlen_max, int priority);
  ~SocketUdp(void);
  bool is_error(void);
  void close_port(void);
  void flush(bool is_decode = true);
  void flush(int slave_id, int master_id, int rxlen_max);

  int write_frame(serial_stream_t* data);
  int read_frame(serial_stream_t* data, float timeout_s = 0);

 private:
  int fp_;
  int is_error_;
  int rxlen_max_;
  bool is_decode_ = true;
  struct sockaddr_in addr_;
  SerialDecode* decode_ = NULL;
  serial_stream_t rx_stream_;
  BlockDeque<serial_stream_t>* rx_que_ = NULL;
  RtPeriodicMemberFun<SocketUdp>* recv_task_ = NULL;

  void recv_proc(void);
};

#endif
