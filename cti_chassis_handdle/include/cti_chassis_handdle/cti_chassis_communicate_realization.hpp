#ifndef CTI_CHASSIS_COMMUNICATE_REALIZATION_H
#define CTI_CHASSIS_COMMUNICATE_REALIZATION_H
#include "cti_chassis_handdle/cti_chassis_communicate_interface.hpp"
#include "serial_cmd.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <crc16/crc16.hpp>
#include "serial_cmd.hpp"
#include <thread>
#include <queue>

/*linux header*/
#include <termios.h>  //struct termios
#include <fcntl.h>    //open close read write
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/select.h>

// linux header
#include <inttypes.h>  //or stdint.h uint16_t uint32_t
#include <unistd.h>    //file operation open read write lseek close
#include <fcntl.h>
#include <linux/serial.h>
#include <iostream>

// for udp
#include <sys/socket.h>
#include <arpa/inet.h>
#include <iostream>



//
class ChassisCmncReal : public ChassisCmncBase {
 public:
  ChassisCmncReal();

  // std::string serial_port;

  void getDatas(unsigned char &cmd, unsigned int &length, unsigned char *data,
                bool &update_flag) override;

  void sendDatas() override;
  int queueSendDatas(unsigned char cmd, unsigned int length, void *data,
                     const bool &set_front) override;

  void send_serial_update_flow(const unsigned char *src,
                               unsigned int cnt) override;

  void runThread();
  void doWork();

  void module_serial_init();
  void init_frame_queue();
  void init_recv_buffer();

  int find_frame_header(const int &pos, char *found);
  void find_serial_frame(const int &fd);

  int queue_recv_frame();
  void parse_data_from_recv(unsigned char *dest, int pos_start, int cnt);

  int open_device_port();
  int do_open_serial();
  void close_serial_port(int fd);

  int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);
  int recv_device_flow(int fd, unsigned char *dest, int cnt);
  int query_serial_data(int fd);

  bool packSendData(unsigned char &cmd, unsigned int &length, void *data);

  void queueDatas(const bool &set_front);

  void send_device_flow(const int &fd);

  void copy_str_way1(unsigned char *myfrom, unsigned char *myto, int length);

  int set_socket(int fd, char *ip, int port);

  int set_dest_socket(char *ip, int port);

  int do_open_udp();

  int send_udp_flow(int fd, unsigned char *src, int cnt);

  int recv_udp_flow(int fd, unsigned char *dest, int cnt);

 private:
  int module_serial_fd{-1};

  SERIAL_RECE_BUF_TYPE serial_recv_buf_type = {0};
  SERIAL_SEND_BUF_TYPE serial_send_buf_type = {0};

  serial_candidate_frame_type candidate = {0};
  serial_frame_cnt_type send_frame_data = {0};
  struct timeval timeout = {0, 100};
  fd_set fds{0};

  bool update_write_flag{false};

  char UDP_IP[DEVICE_PORT_STR_LENGTH] = {0};
  int UDP_PORT = 0;
  struct sockaddr_in sockaddr = {0};

  char UDP_IP_DEST[DEVICE_PORT_STR_LENGTH] = {0};
  int UDP_PORT_DEST = 0;
  struct sockaddr_in sockaddr_dest = {0};
};

#endif
