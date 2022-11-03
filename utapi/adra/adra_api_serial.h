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
  /**
   * AdraApiSerial is an interface class that controls the ADRA actuator through a serial port. USB-to-RS485 or USB-to-CAN
   * module hardware is required to connect the computer and the actuator.
   *
   * @param   char           com      USB serial port, The default port on Linux is "/dev/ttyUSB0".
   * @param   int            baud     Baud rate of serial communication.
   * @param   int            bus_type 0 indicates the actuator that uses the RS485 port.
   *							                    1 indicates the actuator that uses the CAN port.
   *							                    Defaults to 0.
   */
  AdraApiSerial(const char *com, int baud, int bus_type = BUS_TYPE::UTRC) {
    if (bus_type == BUS_TYPE::UTRC) {
      utrc_decode_ = new UtrcDecode(0xAA, 0x55, 128);
      socket_fp = new SocketSerial(com, baud, 16, utrc_decode_, 128, 45);
      if (socket_fp->is_error()) {
        printf("[AdraSeri] Error: socket_file open failed, %s\n", com);
        is_error_ = true;
        return;
      }
    } else if (bus_type == BUS_TYPE::UTCC) {
      utcc_decode_ = new UtccDecode(0xAA, 0x55, 128);
      socket_fp = new SocketSerial(com, baud, 16, utcc_decode_, 128, 45);
      if (socket_fp->is_error()) {
        printf("[AdraSeri] Error: socket_file open failed, %s\n", com);
        is_error_ = true;
        return;
      }
    }

    adrainit(bus_type, socket_fp, 1);
    sleep(1);
  }

  ~AdraApiSerial(void) {
    if (socket_fp != NULL) {
      socket_fp->close_port();
      delete socket_fp;
    }
    if (utrc_decode_ != NULL) delete utrc_decode_;
    if (utcc_decode_ != NULL) delete utcc_decode_;
  }

  bool is_error(void) { return is_error_; }

  /**
   * If use the USB of the EtherNet to RS485/CAN module to transmit RS485/CAN data,
   * need to use this function to put the EtherNet to RS485/CAN module into USB transmission mode.
   * After the EtherNet to RS485/CAN module is powered on, the transmission mode is TCP/UDP by default.
   * Therefore, only need to set the transmission mode once you are powered on.
   *
   */
  void into_usb_pm(void) {
    serial_stream_t stream;
    stream.len = 13;
    memcpy(stream.data, "# INTO-USB-PM\n", stream.len);
    socket_fp->write_frame(&stream);
    sleep(1);
  }

 private:
  UtrcDecode *utrc_decode_ = NULL;
  UtccDecode *utcc_decode_ = NULL;
  SocketSerial *socket_fp = NULL;
  bool is_error_ = false;
};

#endif
