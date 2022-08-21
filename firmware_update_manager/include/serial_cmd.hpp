#ifndef SERIAL_CMD_H
#define SERIAL_CMD_H


/*stm32 header*/
//#include "stm32f1xx_hal.h"    //uint16_t uint32_t
#if 0
#include <stdint.h>             //uint16_t uint32_t
#endif
//typedef unsigned char    uint8_t;    
//typedef unsigned short   uint16_t;    
//typedef unsigned long    uint32_t;   

//linux header
#include <inttypes.h>    //or stdint.h uint16_t uint32_t



/*add update cmds here, and: 

!!!define a updatecmd data type.
!!!add this updatecmd data type in union_data_type.
!!!add data length in data_length[] at the same index.

*/

enum
{
    CMD_NORMAL = 0,

    /********USER CODE BEGIN********/
    /*add your command here*/

    
    /********USER CODE END*********/
    
    CMD_NORMAL_MAX = 199,

    UPD_CMD_200 = 200,  //firmware update notify
    UPD_CMD_201,  //upgrade authentication
    UPD_CMD_202,  //upgrade permit
    UPD_CMD_203,  //upgrade data transfer
    UPD_CMD_204,  //upgrade data transfer respond
    UPD_CMD_205,  //reboot
    UPD_CMD_206,  //upgrade complete
    UPD_CMD_209 = 209,    //check version
    UPD_CMD_210,  //response version
    UPD_CMD_211,  //inform upgrade message
    UPD_CMD_212,  //send the main module id
    CMD_MAX,
};
    

typedef struct __attribute__((packed))
{
    unsigned char cmd_id;
    unsigned int cmd_length;
}cmd_length_table_type;


//add your normal cmd here, following the template of cmd_normal_type.
/********USER CODE BEGIN********/

/********USER CODE END*********/

typedef struct __attribute__((packed))
{
    unsigned char port_idx;           //!!!the port_idx is needed for every normal command in first data byte.
    unsigned char data[15];
}cmd_normal_type;



typedef struct __attribute__((packed))
{
    unsigned char src;
    unsigned char dest;
    //unsigned char update_cmd;
}update_info_type;


typedef struct __attribute__((packed))
{

    update_info_type upd_info;
    unsigned char status;
    unsigned char version[10];
    uint32_t firmware_size;
    unsigned char key[64];  //for athentication
}upd_cmd_200_type;


typedef struct __attribute__((packed))
{
    update_info_type upd_info;
    unsigned char status;
}upd_cmd_201_type;


typedef struct __attribute__((packed))
{
    update_info_type upd_info;
    unsigned char status;
}upd_cmd_202_type;


#define UPDATE_PACKET_DATA_SIZE     256
typedef struct __attribute__((packed))
{
    update_info_type upd_info;
    unsigned char status;
    uint16_t total_packets;
    uint16_t current_packet;
    uint16_t packet_length;
    uint16_t actual_length;
    unsigned char data[UPDATE_PACKET_DATA_SIZE];    //128, 256, 512 ,1024 СFLASH_SECTOR_BYTE_SIZE
}upd_cmd_203_type;


typedef struct __attribute__((packed))
{
    update_info_type upd_info;
    unsigned char status;
    uint16_t current_packet;
}upd_cmd_204_type;


typedef struct __attribute__((packed))
{
    update_info_type upd_info;
    unsigned char status;
}upd_cmd_205_type;


typedef struct __attribute__((packed))
{
    update_info_type upd_info;
    unsigned char status;
}upd_cmd_206_type;


typedef struct __attribute__((packed))
{
    update_info_type upd_info;
    unsigned char check;
}upd_cmd_209_type;



#define MAX_VERSION_STR           10
typedef struct __attribute__((packed))
{
    update_info_type upd_info;
    unsigned char run_area;          //0:boot, 1:app, 2:app2
    unsigned char update_status;       //update status in stage
    unsigned char boot_ver[MAX_VERSION_STR];
    unsigned char app_ver[MAX_VERSION_STR];
    unsigned char update_lib_ver[MAX_VERSION_STR];
}upd_cmd_210_type;


typedef struct __attribute__((packed))
{
    update_info_type upd_info;
    unsigned char status;  //infor the target module. 
                           //0:start upgrade 1:response 2:upgrade success 3:upgrade fail
    unsigned char detail;  //reserved
}upd_cmd_211_type;

typedef struct __attribute__((packed))
{
    update_info_type upd_info;
    uint8_t update_node;
}upd_cmd_212_type;


/*!!!add all the defined cmd type here.*/
typedef union __attribute__((packed))
{

    /********USER CODE BEGIN********/

    /********USER CODE END*********/
    cmd_normal_type  cmd_normal;
    upd_cmd_200_type upd_cmd_200;
    upd_cmd_201_type upd_cmd_201;
    upd_cmd_202_type upd_cmd_202;
    upd_cmd_203_type upd_cmd_203;
    upd_cmd_204_type upd_cmd_204;
    upd_cmd_205_type upd_cmd_205;
    upd_cmd_206_type upd_cmd_206;
    upd_cmd_209_type upd_cmd_209;
    upd_cmd_210_type upd_cmd_210;
    upd_cmd_211_type upd_cmd_211;
    upd_cmd_212_type upd_cmd_212;
}union_data_type;

#define FRAME_HEADER_SIZE           4


/*store a valid frame data in queue when finishing a candidate frame check.*/
typedef struct __attribute__((packed))
{

    unsigned char header[FRAME_HEADER_SIZE];  //"CTRL",�ڴ������м�����ʼ, 
                              //4�ֽ�ͷ���������ݷ����غϵĸ��ʡ�
    unsigned char cmd;    //���ݲ�ͬ�����������ֲ�ͬ��������
    unsigned short cmd_length; //��header��data���ֵ��ֽ���
    union_data_type data;
    
    unsigned short crc16; //��header��data���ֵ�У��
}serial_frame_type;

typedef struct __attribute__((packed))
{

    unsigned char header[FRAME_HEADER_SIZE];  //"CTRL",�ڴ������м�����ʼ, 
                              //4�ֽ�ͷ���������ݷ����غϵĸ��ʡ�
    unsigned char cmd;    //���ݲ�ͬ�����������ֲ�ͬ��������
    unsigned short cmd_length; //��header��data���ֵ��ֽ���
}serial_fixed_header_type;

#define SERIAL_FIXED_HEADER_SIZE     sizeof(serial_fixed_header_type)


typedef struct __attribute__((packed))
{

    unsigned char header[FRAME_HEADER_SIZE];  //"CTRL",�ڴ������м�����ʼ, 
                              //4�ֽ�ͷ���������ݷ����غϵĸ��ʡ�
    unsigned char cmd;    //���ݲ�ͬ�����������ֲ�ͬ��������
    unsigned short cmd_length; //��header��data���ֵ��ֽ���
    unsigned char port_idx;
}serial_get_port_idx_type;


/*store a candidate frame data when pick up frome serial flow*/
typedef struct __attribute__((packed))
{
    unsigned char header[FRAME_HEADER_SIZE];  //"CTRL",�ڴ������м�����ʼ, 
                              //4�ֽ�ͷ���������ݷ����غϵĸ��ʡ�
    unsigned char cmd;    //���ݲ�ͬ�����������ֲ�ͬ��������
    unsigned short cmd_length; //��header��data���ֵ��ֽ���

    unsigned short crc16; //��header��data���ֵ�У��
}serial_candidate_frame_type;


#endif

