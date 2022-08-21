#ifndef SERIAL_API_H
#define SERIAL_API_H

#include "serial_sdk/serial_cmd.hpp"

/*----------------------------------------------*
 * macros & types                               *
 *----------------------------------------------*/
/*debug switch*/
#define DEBUG_PRINTF(format, args...)   //printf("%s %s():"format, __FILE__, __FUNCTION__, ##args)
#define DEBUG_PRINTF_L2(format, args...)   //printf("%s %s():"format, __FILE__, __FUNCTION__, ##args)

#define DEBUG_PRINTF2(format, args...)  //printf(format, ##args)
#define DEBUG_PRINTF2_L2(format, args...)  //printf(format, ##args)

//#define DEBUG_PRINTF(format, ...)  printf(format, ##__VA_ARGS__)

/*recv ring buffer size*/
#define RECEIVE_BUFFER_SIZE             2048

#define MAX_RECV_FRAME_QUEUE            16 
#define MAX_SEND_FRAME_QUEUE            8  //min 3 for resend no warning

#define SERIAL_FRAME_SIZE               sizeof(serial_frame_type)

/*serial receive ring buffer type*/
typedef struct
{
    int start;
    int cnt;
    int state;
    unsigned char buf[RECEIVE_BUFFER_SIZE];
}serial_recv_buf_type;


typedef struct
{
    int cnt;
    unsigned char buf[SERIAL_FRAME_SIZE];
}serial_send_buf_type;


/*serial received frame queue type*/
typedef struct
{
    unsigned char head;
    unsigned char cnt;
    serial_frame_type frame[MAX_RECV_FRAME_QUEUE];
}recv_frame_queue_type;


/*serial send frame queue type*/
typedef struct
{
    unsigned char state;
    unsigned char head;
    unsigned char cnt;
    unsigned short resend[MAX_SEND_FRAME_QUEUE];
    serial_send_buf_type frame[MAX_SEND_FRAME_QUEUE];
}send_frame_queue_type;


extern serial_frame_type serial_frame;
extern recv_frame_queue_type recv_frame_queue;

extern uint16_t recv_pthread_cnt;
extern uint16_t recv_pthread_crc_status;


extern int open_serial_port(void);
extern void colse_serial_port(int fd);

extern void init_frame_queue(void);
extern void init_recv_buffer(void);
extern void find_serial_frame(int fd);

extern int construct_serial_frame_ex(serial_frame_type * frame, unsigned char cmd, unsigned int length, void * data);


/**************for single serial recv&snd frame use***********************/

extern int send_single_serial_frame(serial_frame_type *frame, unsigned char need_setting);
extern int send_update_serial_frame(const unsigned char *data, unsigned int len);
/**************for testing demo**********************/

extern int module_serial_fd;
extern const unsigned char serial_frame_header[];

/********for serial recv&snd frames queue use**********************/
extern void process_received_serial_frames(void);

extern void clear_resend_param(unsigned char cmd);

extern int pack_send_frame_buf_in_queue(serial_frame_type *frame, unsigned short resend);

extern void send_serial_frames_if_necessary(void);


extern void module_serial_init(void);

extern void module_serial_process(void);


extern int recv_data_cnt;
extern int recv_ret;
extern int recv_crc_status;

#endif

