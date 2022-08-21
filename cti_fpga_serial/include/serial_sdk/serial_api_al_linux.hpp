#ifndef SERIAL_API_AL_H
#define SERIAL_API_AL_H

#include "serial_cmd.hpp"

#define DEVICE_PORT_STR_LENGTH          30
extern unsigned char SERIAL_DEVICE_PORT[];


extern const unsigned char serial_frame_header[];

extern int open_serial_port(void);
extern void colse_serial_port(int fd);
extern int send_serial_flow(int fd, unsigned char *src, int cnt);
extern int recv_serial_flow(int fd, unsigned char *dest, int cnt);

extern int process_cmd(serial_frame_type *frame);
extern int (*process_nomal_cmd_cb)(unsigned char cmd, unsigned char *data, unsigned int length);
extern int (*process_update_cmd_cb)(unsigned char cmd, unsigned char *data, unsigned int length);

#endif

