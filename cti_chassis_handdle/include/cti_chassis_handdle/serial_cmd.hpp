#ifndef SERIAL_CMD_H
#define SERIAL_CMD_H

//linux header
#include <inttypes.h>    //or stdint.h uint16_t uint32_t

/*recv ring buffer size*/
#define RECEIVE_BUFFER_SIZE 2048

#define MAX_RECV_FRAME_QUEUE 16
#define MAX_SEND_FRAME_QUEUE 8  // min 3 for resend no warning

#define DATABUF_NUM_MAX 5

// #define LOW_RECEIVE_BUFFER_THRESHOLD 9

#define SERIAL_FRAME_SIZE sizeof(serial_frame_type)

#define RECEIVE_RING_ADD(start, cnt) ((start + (cnt)) % RECEIVE_BUFFER_SIZE)
#define FRAME_HEADER_SIZE           4
/*the serial frame header definition.*/
const unsigned char serial_frame_header[FRAME_HEADER_SIZE] = {'C', 'T', 'R',
                                                              'L'};

#define DEVICE_PORT_STR_LENGTH          30                                                             

enum
{
    CMD_NORMAL = 0,

    /********USER CODE BEGIN********/
    /*add your command here*/

    
    /********USER CODE END*********/
    
    CMD_NORMAL_MAX = 199,
    CMD_MAX,
    CMD_UPD_210 = 210,
    CMD_UPD_206 = 206,
    CMD_UPD_212 = 212,
};
    


//add your normal cmd here, following the template of cmd_normal_type.
/********USER CODE BEGIN********/

/********USER CODE END*********/


#define MAX_CMD_DATA_SIZE       1024
struct cmd_normal_type
{
    unsigned char port_idx;           //!!!the port_idx is needed for every normal command in first data byte.
    unsigned char data[MAX_CMD_DATA_SIZE];
};


/*store a valid frame data in queue when finishing a candidate frame check.*/
//class serial_frame_type
typedef struct __attribute__((packed))
{
    unsigned char header[FRAME_HEADER_SIZE];  //"CTRL",�ڴ������м�����ʼ, 
                              //4�ֽ�ͷ���������ݷ����غϵĸ��ʡ�
    unsigned char cmd;    //���ݲ�ͬ�����������ֲ�ͬ��������
    unsigned short cmd_length; //��header��data���ֵ��ֽ���
    cmd_normal_type data;
    
    unsigned short crc16; //��header��data���ֵ�У��
}serial_frame_type;

//class serial_frame_cnt_type
typedef struct __attribute__((packed))
{
    unsigned int cnt;
    serial_frame_type serial_frame_type_;
}serial_frame_cnt_type;

typedef struct __attribute__((packed))
{

    unsigned char header[FRAME_HEADER_SIZE];  //"CTRL",�ڴ������м�����ʼ, 
                              //4�ֽ�ͷ���������ݷ����غϵĸ��ʡ�
    unsigned char cmd;    //���ݲ�ͬ�����������ֲ�ͬ��������
    unsigned short cmd_length; //��header��data���ֵ��ֽ���
}serial_fixed_header_type;

#define SERIAL_FIXED_HEADER_SIZE     sizeof(serial_fixed_header_type)


/*store a candidate frame data when pick up frome serial flow*/
typedef struct __attribute__((packed))
{
    unsigned char header[FRAME_HEADER_SIZE];  //"CTRL",�ڴ������м�����ʼ, 
                              //4�ֽ�ͷ���������ݷ����غϵĸ��ʡ�
    unsigned char cmd;    //���ݲ�ͬ�����������ֲ�ͬ��������
    unsigned short cmd_length; //��header��data���ֵ��ֽ���

    unsigned short crc16; //��header��data���ֵ�У��
}serial_candidate_frame_type;

//--all the arg *_1 below refer to the same name(without "_1") in the file <<user_cmd.hpp>>
#define MAX_VERSION_STR_1          10
#define MODULE_CHECK_UPD_1         81
typedef struct __attribute__((packed))
{
    unsigned char src;    //发起模块地址
    unsigned char dest;  //接收模块地址
}update_info_type_1;
typedef struct __attribute__((packed))
{
    update_info_type_1 upd_info;
    unsigned char run_area;          //0:boot, 1:app, 2:app2
    unsigned char update_status;       //update status in stage
    unsigned char boot_ver[MAX_VERSION_STR_1];
    char app_ver[MAX_VERSION_STR_1];
   // unsigned char app_ver[MAX_VERSION_STR];
    unsigned char update_lib_ver[MAX_VERSION_STR_1];
}recv_from_firmware_version_type_1;


enum RECV_STATE {
  RECV_STATE_FINDING_HEADER = 0,
  RECV_STATE_CHECK_FIXED_HEADER,
  RECV_STATE_RECEIVING_CRC
};

struct serial_send_buf_type {
  int cnt;
  unsigned char buf[SERIAL_FRAME_SIZE];
};

/*serial receive ring buffer type*/
struct SERIAL_RECE_BUF_TYPE {
  int start;
  int cnt;
  int state;
  unsigned char buf[RECEIVE_BUFFER_SIZE];
};

struct SERIAL_SEND_BUF_TYPE {
  int cnt;
  unsigned char buf[SERIAL_FRAME_SIZE];
};

/*serial received frame queue type*/
struct RECV_FRAME_QUEUE_TYPE {
  unsigned char head;
  unsigned char cnt;
  serial_frame_type frame[MAX_RECV_FRAME_QUEUE];
};

/*serial send frame queue type*/
struct SEND_FRAME_QUEUE_TYPE {
  unsigned char state;
  unsigned char head;
  unsigned char cnt;
  unsigned short resend[MAX_SEND_FRAME_QUEUE];
  SERIAL_SEND_BUF_TYPE frame[MAX_SEND_FRAME_QUEUE];
};

#endif

