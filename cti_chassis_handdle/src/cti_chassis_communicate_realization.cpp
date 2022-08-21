#include "cti_chassis_handdle/cti_chassis_communicate_realization.hpp"
#include <unistd.h>

ChassisCmncReal::ChassisCmncReal() { this->runThread(); }

void ChassisCmncReal::runThread() {
  std::thread dowork_thread([this]() { doWork(); });
  dowork_thread.detach();
}

void ChassisCmncReal::doWork() {
  module_serial_init();
  while (1) {
    usleep(2000);
    find_serial_frame(module_serial_fd);
  }
}

void ChassisCmncReal::sendDatas() {
  if (send_frame_deque_.size() == 0) {
    return;
  }

  send_device_flow(this->module_serial_fd);
  send_frame_deque_.pop_front();
}

int ChassisCmncReal::queueSendDatas(unsigned char cmd, unsigned int length,
                                    void *data, const bool &set_front) {
  if (!packSendData(cmd, length, data)) return -1;
  queueDatas(set_front);
  return 1;
}

void ChassisCmncReal::queueDatas(const bool &set_front) {
  if (send_frame_deque_.size() > DATABUF_NUM_MAX) {
    send_frame_deque_.pop_front();
  }
  //时间同步命令优先发送
  if (set_front) {
    send_frame_deque_.push_front(send_frame_data);
  } else {
    send_frame_deque_.push_back(send_frame_data);
  }
}

void ChassisCmncReal::module_serial_init() {
  init_recv_buffer();
  this->module_serial_fd = open_device_port();
}

void ChassisCmncReal::init_recv_buffer() {
  serial_recv_buf_type.start = 0;
  serial_recv_buf_type.cnt = 0;
  serial_recv_buf_type.state = RECV_STATE_FINDING_HEADER;
  init_crc();
}

void ChassisCmncReal::send_device_flow(const int &fd) {

  if(fd == -1)
  {
    return;
  }

  int ret = 0;
  int temp = send_frame_deque_.front().cnt;

  serial_frame_type serial_frame_type_ =
      send_frame_deque_.front().serial_frame_type_;


  if (this->getDeviceType() == "serial") {
    while (temp) {
      ret = write(fd, &serial_frame_type_, temp);
      temp -= ret;
    }
  } 
  else if (this->getDeviceType() == "udp") {
    while (temp) {
      ret = sendto(fd, &serial_frame_type_, temp, 0,
                   (struct sockaddr *)&sockaddr_dest, sizeof(sockaddr));
      temp -= ret;
    }
  }
}

void ChassisCmncReal::copy_str_way1(unsigned char *myfrom, unsigned char *myto,
                                    int length) {
  unsigned char *from = myfrom;
  unsigned char *to = myto;
  int i = 0;
  for (i = 0; length > i; i++) {
    *(to + i) = *(from + i);
  }
  // 由于最后一个字符'\0'没有赋给to，所以把'\0'赋值给to+i
  *(to + i) = '\0';
}

void ChassisCmncReal::getDatas(unsigned char &cmd, unsigned int &length,
                               unsigned char *data, bool &update_flag) {
  static unsigned char cmd_temp{0};
  static bool need_double_click{false};

  if (recv_frame_queue_.size() > 0) {

    cmd_temp = recv_frame_queue_.front().cmd;

    if (cmd_temp <= CMD_NORMAL_MAX) {
      update_flag = false;
      cmd = cmd_temp;
      length = recv_frame_queue_.front().cmd_length - SERIAL_FIXED_HEADER_SIZE;
      copy_str_way1((unsigned char *)&(recv_frame_queue_.front().data), data,
                    length);
    } else {
      if (cmd_temp == CMD_UPD_206 || cmd_temp == CMD_UPD_212 ||
          cmd_temp == CMD_UPD_210) {
        if (need_double_click == false) {
          update_flag = false;
          cmd = cmd_temp;
          length =
              recv_frame_queue_.front().cmd_length - SERIAL_FIXED_HEADER_SIZE;
          copy_str_way1((unsigned char *)&(recv_frame_queue_.front().data),
                        data, length);
          need_double_click = true;
          return;
        } else {
          need_double_click = false;
        }
      }
      //升级命令
      update_flag = true;
      cmd = cmd_temp;
      length = recv_frame_queue_.front().cmd_length + 2;

      copy_str_way1((unsigned char *)&recv_frame_queue_.front(), data,
                    recv_frame_queue_.front().cmd_length);
      copy_str_way1((uint8_t *)&recv_frame_queue_.front().crc16,
                    data + recv_frame_queue_.front().cmd_length, 2);
    }

    mutex_queue_.lock();
    recv_frame_queue_.pop();
    mutex_queue_.unlock();

  } else {
    cmd = 0;
    length = 0;
    data = NULL;
    update_flag = false;
  }
  return;
}

void ChassisCmncReal::send_serial_update_flow(const unsigned char *src,
                                              unsigned int cnt) {
  int ret = 0;
  int temp = cnt;
  int fd = this->module_serial_fd;

  if(fd == -1)
  {
    std::cout<<"module_serial_fd = -1"<<std::endl;
    return;
  }

  if (this->getDeviceType() == "serial") {
    while (temp) {
      ret = write(fd, src, temp);
      temp -= ret;
    }
  } 
  else if (this->getDeviceType() == "udp") {
    while (temp) {
      ret = sendto(fd, src, temp, 0,
                   (struct sockaddr *)&sockaddr_dest, sizeof(sockaddr));
      temp -= ret;
    }
  }
}



  

bool ChassisCmncReal::packSendData(unsigned char &cmd, unsigned int &length,
                                   void *data) {
  if ((NULL == data)) {
    std::cout << "NULL frame or data ptr when init serial frame!!!"
              << std::endl;
    return false;
  }

  if (length + sizeof(serial_fixed_header_type) > sizeof(serial_frame_type)) {
    std::cout << "***********Oops, we got a cmd_data_length[cmd] > "
                 "sizeof(serial_frame.data)!!! "
              << std::endl;
    return false;
  }

  unsigned char *pos = (unsigned char *)&(send_frame_data.serial_frame_type_);

  send_frame_data.cnt = length +
                        sizeof(send_frame_data.serial_frame_type_.crc16) +
                        sizeof(serial_fixed_header_type);

  memcpy((unsigned char *)&(send_frame_data.serial_frame_type_),
         serial_frame_header,
         sizeof(send_frame_data.serial_frame_type_.header));
  send_frame_data.serial_frame_type_.cmd = cmd;
  send_frame_data.serial_frame_type_.cmd_length =
      sizeof(serial_fixed_header_type) + length;
  memcpy((unsigned char *)&(send_frame_data.serial_frame_type_.data), data,
         length);

  // for (int i = 0; i < length; i++) {
  //     printf("%c\n", &send_frame_data.serial_frame_type_.data+i);
  // }
  init_crc();

  send_frame_data.serial_frame_type_.crc16 =
      calc_crc((unsigned char *)&(send_frame_data.serial_frame_type_),
               send_frame_data.serial_frame_type_.cmd_length);

  memcpy((unsigned char *)&(send_frame_data.serial_frame_type_) +
             send_frame_data.serial_frame_type_.cmd_length,
         (void *)&send_frame_data.serial_frame_type_.crc16,
         sizeof(send_frame_data.serial_frame_type_.crc16));


  return true;
}

/*****************************************************************************
 Prototype    : parse_recv_frame
 Description  : parse a frame from receive buffer ring to frame queue.
 Output       : None
 Return Value :
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/05/18
    Author       : Huangwei
    Modification : Created function

*****************************************************************************/
int ChassisCmncReal::queue_recv_frame() {
  serial_frame_type receive_serial_data{0};

  memcpy(receive_serial_data.header, serial_frame_header, FRAME_HEADER_SIZE);
  receive_serial_data.cmd = candidate.cmd;
  receive_serial_data.cmd_length = candidate.cmd_length;
  receive_serial_data.crc16 = candidate.crc16;

  parse_data_from_recv(
      (unsigned char *)&receive_serial_data.data,
      (serial_recv_buf_type.start + sizeof(serial_fixed_header_type)) %
          RECEIVE_BUFFER_SIZE,
      candidate.cmd_length - sizeof(serial_fixed_header_type));

  mutex_queue_.lock();

  if(this->recv_frame_queue_.size() >=25)
  {
    std::cout << "High process_received_serial_frames level detected!!!"
                << std::endl;
      std::cout << recv_frame_queue_.size() << std::endl;
    //recv_frame_queue_.pop();
  }

  recv_frame_queue_.push(receive_serial_data);
  mutex_queue_.unlock();

  return 1;
}

/*****************************************************************************
 Prototype    : parse_data_from_recv
 Description  : parse a frame data to dest from receive buffer ring.
 Input        : unsigned char *dest: destination buffer.
                unsigned char *src: source buffer ring.
                int pos_start: start position of valid data to parse.
 Output       : None
 Return Value :
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/05/18
    Author       : Huangwei
    Modification : Created function

*****************************************************************************/
void ChassisCmncReal::parse_data_from_recv(unsigned char *dest, int pos_start,
                                           int cnt) {
  int copy1, copy2;

  if (NULL == dest) {
    return;
  }

  if ((pos_start < 0) || (pos_start >= RECEIVE_BUFFER_SIZE) ||
      cnt > serial_recv_buf_type.cnt) {
    std::cout << "parse param range error." << std::endl;
    return;
  }

  /*parse data*/
  copy1 = ((RECEIVE_BUFFER_SIZE - pos_start) >= cnt)
              ? cnt
              : (RECEIVE_BUFFER_SIZE - pos_start);
  copy2 = cnt - copy1;

  memcpy(dest, serial_recv_buf_type.buf + pos_start, copy1);

  if (copy2 > 0) {
    memcpy(dest + copy1, serial_recv_buf_type.buf, copy2);
  }
}

/*****************************************************************************
 Prototype    : find_frame_header
 Description  : find a frame header from received serial ring buffer.
 Input        :
                pos     start position of received data for check.
 Output       :
                found   find a valid frame header.
 Return Value : the count size when finished finding.
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/05/18
    Author       : Huangwei
    Modification : Created function

*****************************************************************************/
int ChassisCmncReal::find_frame_header(const int &pos, char *found) {
  int i{0}, match_cnt{0};

  if ((pos < 0) || (pos >= RECEIVE_BUFFER_SIZE)) {
    // DEBUG_PRINTF("Test point 1\n");
    return 0;
  }

  while (i < (serial_recv_buf_type.cnt - FRAME_HEADER_SIZE)) {
    match_cnt = 0;
    while (match_cnt < FRAME_HEADER_SIZE) {
      if (serial_recv_buf_type.buf[RECEIVE_RING_ADD(pos, i)] !=
          serial_frame_header[match_cnt]) {
        i++;
        break;
        ;
      } else {
        match_cnt++;
        i++;
      }
    }

    if (FRAME_HEADER_SIZE == match_cnt) {
      *found = 1;
      break;
    }
  }

  return i;
}

/*****************************************************************************
 Prototype    : find_serial_frame
 Description  : find a valid frame in received buffer ring, copy to frame
                buffer.
 Input        : void
 Output       : None
 Return Value :
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/05/18
    Author       : Huangwei
    Modification : Created function

*****************************************************************************/
void ChassisCmncReal::find_serial_frame(const int &fd) {
  int ret{0}, i{0};
  static int candidate_pos = 0;
  char found_header = 0;
  int length = 0;

  if (fd < 0) {
    return;
  }

  ret = query_serial_data(fd);

  if (ret <= 0) {
    return;
  }

  while (serial_recv_buf_type.cnt) {
    switch (serial_recv_buf_type.state) {
      case RECV_STATE_FINDING_HEADER:
        if (serial_recv_buf_type.cnt <= FRAME_HEADER_SIZE) {
          return;
        }

        found_header = 0;
        ret = find_frame_header(serial_recv_buf_type.start, &found_header);

        if (found_header)  // find header bytes
        {
          /*hold to header*/
          serial_recv_buf_type.start = RECEIVE_RING_ADD(
              serial_recv_buf_type.start, ret - FRAME_HEADER_SIZE);
          serial_recv_buf_type.cnt -= (ret - FRAME_HEADER_SIZE);

          serial_recv_buf_type.state = RECV_STATE_CHECK_FIXED_HEADER;
        } else {
          // printf("Doesnt find header in recv buffer.\n");
          /*Not a valid header, abandon it.*/
          serial_recv_buf_type.start =
              RECEIVE_RING_ADD(serial_recv_buf_type.start, ret);
          serial_recv_buf_type.cnt -= ret;
        }
        break;

      case RECV_STATE_CHECK_FIXED_HEADER:
        if (serial_recv_buf_type.cnt < sizeof(serial_fixed_header_type)) {
          return;
        }

        memset(&candidate, 0, sizeof(candidate));

        parse_data_from_recv((unsigned char *)&candidate,
                             serial_recv_buf_type.start,
                             sizeof(serial_fixed_header_type));

        // for ros module seperate
        if (candidate.cmd_length >
            (sizeof(serial_fixed_header_type) + MAX_CMD_DATA_SIZE)) {
          /*not a valid header*/
          serial_recv_buf_type.start =
              RECEIVE_RING_ADD(serial_recv_buf_type.start, FRAME_HEADER_SIZE);
          serial_recv_buf_type.cnt -= FRAME_HEADER_SIZE;

          serial_recv_buf_type.state = RECV_STATE_FINDING_HEADER;
          return;
        }

        serial_recv_buf_type.state = RECV_STATE_RECEIVING_CRC;
        break;

      case RECV_STATE_RECEIVING_CRC:
        if (serial_recv_buf_type.cnt < candidate.cmd_length + 2)  // total frame
        {
          return;
        }

        /*indicate to crc*/
        candidate_pos =
            RECEIVE_RING_ADD(serial_recv_buf_type.start, candidate.cmd_length);
        parse_data_from_recv((unsigned char *)&candidate.crc16, candidate_pos,
                             sizeof(candidate.crc16));

        /*calculate candidate frame crc*/
        init_crc();
        for (i = 0; i < candidate.cmd_length; i++) {
          candidate_pos = RECEIVE_RING_ADD(serial_recv_buf_type.start, i);
          calc_crc(&serial_recv_buf_type.buf[candidate_pos], 1);
        }

        if (frame_crc == candidate.crc16) {
          if (!queue_recv_frame()) {
          }

          serial_recv_buf_type.start = RECEIVE_RING_ADD(
              serial_recv_buf_type.start, candidate.cmd_length + 2);
          serial_recv_buf_type.cnt -= (candidate.cmd_length + 2);
          serial_recv_buf_type.state = RECV_STATE_FINDING_HEADER;
        } else {

          // printf(
          //     "bad crc check(recv:0x%x, calc:0x%x). Find a fake or damaged "
          //     "frame.\n",
          //     candidate.crc16, frame_crc);
          serial_recv_buf_type.start =
              RECEIVE_RING_ADD(serial_recv_buf_type.start, FRAME_HEADER_SIZE);
          serial_recv_buf_type.cnt -= FRAME_HEADER_SIZE;
          serial_recv_buf_type.state = RECV_STATE_FINDING_HEADER;
        }
        break;

      default:
        /*wish nerver reach here.*/
        printf("Oops!***, wrong recv state:%d.\n", serial_recv_buf_type.state);
        serial_recv_buf_type.state = RECV_STATE_FINDING_HEADER;
        break;
    }
  }
}

/*****************************************************************************
 Prototype    : query_serial_data
 Description  : read serial data and put data into buf.
 Input        : None
 Output       : the receive data structure serial_recv_buf_type.
 Return Value : the size of read data.
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/05/18
    Author       : Huangwei
    Modification : Created function
*****************************************************************************/
int ChassisCmncReal::query_serial_data(int fd) {
  int pos{0};
  int ret{0};
  int remain_buffer_cnt{0};

  remain_buffer_cnt = RECEIVE_BUFFER_SIZE - serial_recv_buf_type.cnt;

  /*low available buffer level*/
  if (remain_buffer_cnt < SERIAL_FRAME_SIZE) {
    // DEBUG_PRINTF("Low recv buffer level!****cleaning recv buffer!\n\r");
    std::cout << "Low recv buffer level!****cleaning recv buffer!" << std::endl;
    init_recv_buffer();
    ret = 0;
    return ret;
  }

  /*if near buffer end, just read some data to fill buffer tail.*/
  pos = RECEIVE_RING_ADD(serial_recv_buf_type.start, serial_recv_buf_type.cnt);

  if (pos >= serial_recv_buf_type.start) {
    ret = recv_device_flow(fd, serial_recv_buf_type.buf + pos,
                           ((RECEIVE_BUFFER_SIZE - pos) > SERIAL_FRAME_SIZE)
                               ? SERIAL_FRAME_SIZE
                               : (RECEIVE_BUFFER_SIZE - pos));
  } else {
    ret = recv_device_flow(fd, serial_recv_buf_type.buf + pos,
                           (remain_buffer_cnt > SERIAL_FRAME_SIZE)
                               ? SERIAL_FRAME_SIZE
                               : remain_buffer_cnt);
  }

  if (ret > 0) {
    serial_recv_buf_type.cnt += ret;
  }

  return ret;
}

/*
    Prototype    : select()
    Description  : pretect form blocking ,if not serial buf no have content or
   break
*/
int ChassisCmncReal::recv_device_flow(int fd, unsigned char *dest, int cnt) {

  if(fd == -1)
  {
    return 0;
  }

  int ret = -1;
  FD_ZERO(&fds);
  FD_SET(fd, &fds);
  switch (select(fd + 1, &fds, NULL, NULL, &timeout)) {
    case -1:
      printf("serial read error!\n");
      break;
    case 0:
      break;
    default:
      if (FD_ISSET(fd, &fds)) {
        if (this->getDeviceType() == "serial") {
          ret = read(fd, dest, cnt);
        } else if (this->getDeviceType() == "udp") {
          socklen_t addrlen = sizeof(sockaddr);
          ret = recvfrom(fd, dest, cnt, 0, (struct sockaddr *)&sockaddr,
                         &addrlen);
        }
      }
  }
  return ret;
}

/*****************************************************************************
 Prototype    : open_device_port
 Description  : open a specific serial port until success.
 Input        : void
 Output       : void
 Return Value : the file handler fd.
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/05/18
    Author       : Huangwei
    Modification : Created function

*****************************************************************************/
int ChassisCmncReal::open_device_port() {

  //wait for ros param init
  sleep(1);

  int fd1{0};
  if (this->module_serial_fd != -1) {
    return this->module_serial_fd;
  }

  if (this->getDeviceType() == "udp") {
    while (-1 == (fd1 = do_open_udp())) {
      std::cout << "open udp port failed!" << std::endl;
      usleep(1000000UL);
    }
  } else if (this->getDeviceType() == "serial") {
    while (-1 == (fd1 = do_open_serial())) {
      std::cout << "open serial port failed! " << std::endl;
      usleep(1000000UL);
    }
  }

  std::cout << "open port successed!" << std::endl;
  return fd1;
}

int ChassisCmncReal::do_open_udp() {
  int fd1{0}, ret{0};

  strcpy((char *)UDP_IP, getDeviceName().c_str());
  strcpy((char *)UDP_IP_DEST, getDestName().c_str());
  UDP_PORT_DEST = getDevicePort();
  UDP_PORT = getDestPort();

  fd1 = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd1 == -1) {
    std::cout << "udp socket not ready" << std::endl;
    return -1;
  }
  std::cout << "open udp socket success" << std::endl;

  ret = set_socket(fd1, UDP_IP, UDP_PORT);
  set_dest_socket(UDP_IP_DEST, UDP_PORT_DEST);
  printf("sockaddr:<ip %s> <port %d> \n", inet_ntoa(sockaddr_dest.sin_addr),
         ntohs(sockaddr_dest.sin_port));
  if (ret == -1) {
    std::cout << "set udp socket udp_ip: " << UDP_IP
              << " udp_port: " << UDP_PORT << " failed!" << std::endl;
    return -1;
  }
  std::cout << "set udp socket udp_ip: " << UDP_IP << " udp_port: " << UDP_PORT
            << " successed!" << std::endl;
  return fd1;
}

/*****************************************************************************
 Prototype    : colse_serial_port
 Description  : close an opened port.
 Input        : int fd
 Output       : None
 Return Value :
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/05/18
    Author       : Huangwei
    Modification : Created function

*****************************************************************************/
void ChassisCmncReal::close_serial_port(int fd) {
  close(fd);
  this->module_serial_fd = 0;
}

/*****************************************************************************
 Prototype    : do_open_serial
 Description  : open a specific serial port and return result.
 Input        : void
 Output       : None
 Return Value : the open result
                -1    fail
                >=0   success, return the fd.
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/05/18
    Author       : Huangwei
    Modification : Created function

*****************************************************************************/
int ChassisCmncReal::do_open_serial() {
  int fd1{0}, ret{0};

  /*Open device port*/
  fd1 = open(this->getDeviceName().c_str(), O_RDWR | O_NOCTTY);  // | O_NDELAY);
  if (fd1 == -1) {
    std::cout << "Port " << this->getDeviceName() << " not ready" << std::endl;
    return -1;
  }
  std::cout << "Port " << this->getDeviceName() << " success" << std::endl;
  /*Set device*/
  // ret = set_opt(fd1, 115200, 8, 'N', 1);
  ret = set_opt(fd1, 115200, 8, 'N', 1);
  if (ret == -1) {
    std::cout << "SET " << this->getDeviceName() << " failed" << std::endl;
    return -1;
  }
  std::cout << "SET " << this->getDeviceName() << " success" << std::endl;
  return fd1;
}

/******************************************************************************
func:           set serial port param.
input param:
                fd: device file handle.
                nspeed: baudrate
                nBits: data bits
                nEvent: parity check
                sStop: stop bit
output:         none

return:
                -1      error
                0       success
author:         Huangwei
history:        2018/05/18      initial

******************************************************************************/
int ChassisCmncReal::set_opt(int fd, int nSpeed, int nBits, char nEvent,
                             int nStop) {
  struct termios newtio, oldtio;

  if (tcgetattr(fd, &oldtio) != 0) {
    perror("Get Serial attr failed.");
    return -1;
  }

  memset(&newtio, 0, sizeof(newtio));
  newtio.c_cflag |=
      CLOCAL |
      CREAD;  // CLOCAL:disable the modem control line; CREAD:open receiver.
  newtio.c_cflag &= ~CSIZE;  // data bit mask:S5,S6,S7 or CS8

  newtio.c_lflag &= ~ICANON;

  switch (nBits) {
    case 7:
      newtio.c_cflag |= CS7;
      break;

    case 8:
      newtio.c_cflag |= CS8;
      break;
  }

  switch (nEvent) {
    case 'O':
      newtio.c_cflag |= PARENB;  // enable parity check
      newtio.c_cflag |= PARODD;  // odd
      newtio.c_iflag |=
          (INPCK |
           ISTRIP);  // INPACK:enable input parity check; ISTRIP:discard 8th bit
      break;

    case 'E':
      newtio.c_iflag |= (INPCK | ISTRIP);
      newtio.c_cflag |= PARENB;
      newtio.c_cflag &= ~PARODD;
      break;

    case 'N':
      newtio.c_cflag &= ~PARENB;
      break;
  }

  switch (nSpeed) {
    case 2400:
      cfsetispeed(&newtio, B2400);
      cfsetospeed(&newtio, B2400);
      break;

    case 4800:
      cfsetispeed(&newtio, B4800);
      cfsetospeed(&newtio, B4800);
      break;

    case 9600:
      cfsetispeed(&newtio, B9600);
      cfsetospeed(&newtio, B9600);
      break;
    case 57600:
      cfsetispeed(&newtio, B57600);
      cfsetospeed(&newtio, B57600);
      break;

    case 115200:
      cfsetispeed(&newtio, B115200);
      cfsetospeed(&newtio, B115200);
      break;

    case 460800:
      cfsetispeed(&newtio, B460800);
      cfsetospeed(&newtio, B460800);
      break;

    default:
      cfsetispeed(&newtio, B9600);
      cfsetospeed(&newtio, B9600);
      break;
  }

  if (1 == nStop) {
    newtio.c_cflag &= ~CSTOPB;  // set 0 to enable 1 stop bit
  } else if (2 == nStop) {
    newtio.c_cflag |= CSTOPB;  // set 1 to enable 2 stop bit
  }

  // char count in noncanonical mode.
  newtio.c_cc[VTIME] =
      0;  // VTIME: ready delay in noncannoical mode, uint in 1/10 sec.
  newtio.c_cc[VMIN] =
      0;  // VMIN:minimum received char count in noncanonical mode.
  tcflush(fd,
          TCIFLUSH);  // adopt change after all fd file has transfered. discard
                      // all received but not read data before adopt change.

  if ((tcsetattr(fd, TCSANOW, &newtio)) !=
      0)  // TCSANOW:adopt change immediately.
  {
    perror("Serial attr set error");
    return -1;
  }

  return 0;
}

int ChassisCmncReal::set_socket(int fd, char *ip, int port) {
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_port = htons(port);
  sockaddr.sin_addr.s_addr = inet_addr(ip);
  if (bind(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
    std::cout << "server bind error!" << std::endl;
    return -1;
  }
  return 0;
}
int ChassisCmncReal::set_dest_socket(char *ip, int port) {
  printf("set_dest_socket: ip<%s>,port<%d>\n", ip, port);
  sockaddr_dest.sin_family = AF_INET;
  sockaddr_dest.sin_port = htons(port);
  sockaddr_dest.sin_addr.s_addr = inet_addr(ip);
  return 0;
}
