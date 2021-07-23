/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __ADRA_API_SERIAL_H__
#define __ADRA_API_SERIAL_H__

#include "adra/adra_api_base.h"
#include "common/socket_serial.h"

class AdraApiSerial : public AdraApiBase {
 public:
  AdraApiSerial(const char *com, int baud);
  ~AdraApiSerial(void);

 private:
  UtrcDecode *utrc_decode_ = NULL;
  SocketSerial *socket_serial_ = NULL;
};

#endif
