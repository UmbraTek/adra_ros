/* Copyright 2022 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __ADRA_API_UDP_H__
#define __ADRA_API_UDP_H__

#include "adra/adra_api_base.h"
#include "common/socket_udp.h"

class AdraApiUdp : public AdraApiBase {
 public:
  /**
   * AdraApiUdp is an interface class that controls the ADRA actuator through a EtherNet UDP. EtherNet-to-RS485 or
   * EtherNet-to-CAN module hardware is required to connect the computer and the actuator.
   *
   * @param   char    ip        IP address of the EtherNet module.
   * @param   int     port      UDP port of EtherNet module. The default value is 5001.
   * @param   int     bus_type  0 indicates the actuator that uses the RS485 port.
   *                            1 indicates the actuator that uses the CAN port.
   *                            Defaults to 0.
   * @param   int     is_reset   Defaults to 1. Whether to reset can be reset in the following situations.
   *                    1. If connection type is UDP and DataLink is connected to TCP after being powered on,
   *                       reset is required.
   *                    2. If connection type is UDP and DataLink is not connected to TCP after being powered on,
   *                       you do not need to reset.
   *                    3. If connection type is TCP and DataLink is connected to TCP or UDP after being powered on,
   *                       reset is required.
   *                    4. If connection type is TCP and DataLink is not connected to TCP or UDP after being powered on,
   *                       you do not need to reset.
   *                    Note: In any case, it is good to use reset, but the initialization time is about 3 seconds
   *                          longer than that without reset.
   *                    Note: After DataLink is powered on and connected to USB, it needs to be powered on again
   *                          to connect to TCP or UDP.
   * @param   int     tcp_port  TCP port of EtherNet module. The default value is 6001.
   * @param   int     baud      Set the baud rate of the EtherNet to RS485/CAN module to be the same as that of the actuator.
   *                            If the baud rate is set to 0xFFFFFFFF, the baud rate of the EtherNet to RS485/CAN module is
   *                            not set. The default value is 0xFFFFFFFF.
   */
  AdraApiUdp(char *ip, int port = 5001, int bus_type = BUS_TYPE::UTRC, int is_reset = 1, int tcp_port = 6001,
             int baud = 0xFFFFFFFF) {
    if (bus_type == BUS_TYPE::UTRC) {
      if (is_reset) reset_net_rs485(ip, tcp_port, port);

      utrc_decode_ = new UtrcDecode(0xAA, 0x55, 128);
      socket_fp = new SocketUdp(ip, port, 16, utrc_decode_, 125, 45);
      if (socket_fp->is_error()) {
        printf("[Adra UDP] Error: SocketUdp failed, %s %d\n", ip, port);
        is_error_ = true;
        return;
      }
    } else if (bus_type == BUS_TYPE::UTCC) {
      if (is_reset) reset_net_can(ip, tcp_port, port);

      utcc_decode_ = new UtccDecode(0xAA, 0x55, 128);
      socket_fp = new SocketUdp(ip, port, 16, utcc_decode_, 125, 45);
      if (socket_fp->is_error()) {
        printf("[Adra UDP] Error: SocketUdp failed, %s %d\n", ip, port);
        is_error_ = true;
        return;
      }
    }

    adrainit(bus_type, socket_fp, 1);
    sleep(1);

    connect_net_module(baud);
  }

  ~AdraApiUdp(void) {
    if (socket_fp != NULL) {
      socket_fp->close_port();
      delete socket_fp;
    }
    if (utrc_decode_ != NULL) delete utrc_decode_;
    if (utcc_decode_ != NULL) delete utcc_decode_;
  }

  bool is_error(void) { return is_error_; }

  void into_usb_pm(void) {}

 private:
  UtrcDecode *utrc_decode_ = NULL;
  UtccDecode *utcc_decode_ = NULL;
  SocketUdp *socket_fp = NULL;
  bool is_error_ = false;

  void reset_net_can(char ip[], int tcp_port, int udp_port) {
    utcc_t tx_utcc;
    tx_utcc.head = 0xAA;
    tx_utcc.id = 0x0055;
    tx_utcc.state = 0;
    tx_utcc.len = 0x08;
    tx_utcc.rw = 0;
    tx_utcc.cmd = 0x7F;
    tx_utcc.data[0] = 0x7F;
    tx_utcc.data[1] = 0x7F;
    tx_utcc.data[2] = 0x7F;
    tx_utcc.data[3] = 0x7F;
    tx_utcc.data[4] = 0x7F;
    tx_utcc.data[5] = 0x7F;
    tx_utcc.data[6] = 0x7F;
    serial_stream_t tx_stream_;
    tx_utcc.pack(&tx_stream_);

    printf("[Adra UDP] Reset Net Step1: connect to tcp\n");
    int fp = LinuxCvl::socket_init((char *)" ", 0, 0);
    if (fp != -1) {
      struct sockaddr_in server_addr;
      server_addr.sin_family = AF_INET;
      server_addr.sin_port = htons(tcp_port);
      inet_aton(ip, &server_addr.sin_addr);
      int ret = connect(fp, (struct sockaddr *)&server_addr, sizeof(server_addr));
      if (ret == 0) {
        LinuxCvl::socket_send_data(fp, tx_stream_.data, tx_stream_.len);
      }
    }
    sleep(0.1);

    printf("[Adra UDP] Reset Net Step2: connect to udp\n");
    fp = LinuxCvl::socketudp_init((char *)" ", 0, 0);
    if (fp > -1) {
      struct sockaddr_in addr_ = LinuxCvl::get_sockaddr(ip, udp_port);
      int ret = LinuxCvl::socketudp_send_data(fp, addr_, tx_stream_.data, tx_stream_.len);
    }
    sleep(3);
    is_error_ = false;
  }

  void reset_net_rs485(char ip[], int tcp_port, int udp_port) {
    utrc_t tx_utrc;
    tx_utrc.master_id = 0xAA;
    tx_utrc.slave_id = 0x55;
    tx_utrc.state = 0;
    tx_utrc.len = 0x08;
    tx_utrc.rw = 0;
    tx_utrc.cmd = 0x7F;
    tx_utrc.data[0] = 0x7F;
    tx_utrc.data[1] = 0x7F;
    tx_utrc.data[2] = 0x7F;
    tx_utrc.data[3] = 0x7F;
    tx_utrc.data[4] = 0x7F;
    tx_utrc.data[5] = 0x7F;
    tx_utrc.data[6] = 0x7F;
    serial_stream_t tx_stream_;
    tx_utrc.pack(&tx_stream_);

    printf("[Adra UDP] Reset Net Step1: connect to tcp\n");
    int fp = LinuxCvl::socket_init((char *)" ", 0, 0);
    if (fp != -1) {
      struct sockaddr_in server_addr;
      server_addr.sin_family = AF_INET;
      server_addr.sin_port = htons(tcp_port);
      inet_aton(ip, &server_addr.sin_addr);
      int ret = connect(fp, (struct sockaddr *)&server_addr, sizeof(server_addr));
      if (ret == 0) {
        LinuxCvl::socket_send_data(fp, tx_stream_.data, tx_stream_.len);
      }
    }
    sleep(0.1);

    printf("[Adra UDP] Reset Net Step2: connect to udp\n");
    fp = LinuxCvl::socketudp_init((char *)" ", 0, 0);
    if (fp > -1) {
      struct sockaddr_in addr_ = LinuxCvl::get_sockaddr(ip, udp_port);
      int ret = LinuxCvl::socketudp_send_data(fp, addr_, tx_stream_.data, tx_stream_.len);
    }
    sleep(3);
    is_error_ = false;
  }
};

#endif
