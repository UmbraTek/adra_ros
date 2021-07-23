/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#include "adra/adra_api_serial.h"

AdraApiSerial::AdraApiSerial(const char* com, int baud) {
  utrc_decode_ = new UtrcDecode(0xAA, 0x55, 128);
  socket_serial_ = new SocketSerial(com, baud, 16, utrc_decode_, 128, 45);
  if (socket_serial_->is_error()) {
    printf("[AdraApiS] Error: socket_file open failed, %s\n", com);
    return;
  }

  adrainit(BUS_TYPE::UTRC, socket_serial_, 1);
  sleep(1);
}

AdraApiSerial::~AdraApiSerial(void) {
  if (socket_serial_ != NULL) {
    socket_serial_->close_port();
    delete socket_serial_;
  }
  if (utrc_decode_ != NULL) delete utrc_decode_;
}