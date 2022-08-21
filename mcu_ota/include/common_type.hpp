#ifndef COMMON_TYPE_HPP
#define COMMON_TYPE_HPP
#include <string>
#include <cmd_type.hpp>

//升级选项
typedef struct{
    int main_module_id;//升级的主板id号
    std::string main_module_name;//升级的主板名称
    int module_id;//升级的子板id
    std::string module_name;//升级的子版名称
    std::string bin_file_name;//升级文件
}update_option_t;

//参数列表结构体
#define MAX_BIN_FILE_NAME_SIZE          256
#define MAX_FILE_NAME_SIZE              MAX_BIN_FILE_NAME_SIZE
typedef struct
{
    uint8_t module_id;
    std::string module_name;
    std::string module_bin_file_name;
}module_param_table_type;

#define ROS_COMM_BUF_SIZE       2048
typedef struct
{
    unsigned char buf[ROS_COMM_BUF_SIZE];
    int start=0;
    int end=0;
}ros_comm_type;

/*serial receive ring buffer type*/ 
#define RECEIVE_BUFFER_SIZE     512 //接收区大小
typedef struct
{
    int start;
    int cnt;
    int state;
    unsigned char buf[RECEIVE_BUFFER_SIZE];
}serial_recv_buf_type;

#define SERIAL_FRAME_SIZE       sizeof(serial_frame_type)
typedef struct
{
    int cnt;
    unsigned char buf[SERIAL_FRAME_SIZE];
}serial_send_buf_type;

/*serial received frame queue type*/
#define MAX_RECV_FRAME_QUEUE    10  //接受队列的大小 
typedef struct
{
    unsigned char head;
    unsigned char cnt;
    serial_frame_type frame[MAX_RECV_FRAME_QUEUE];
}recv_frame_queue_type;

/*serial send frame queue type*/
#define MAX_SEND_FRAME_QUEUE    60  //发送对列的大小
typedef struct
{
    unsigned char state;
    unsigned char head;
    unsigned char cnt;
    unsigned int resend[MAX_SEND_FRAME_QUEUE];
    serial_send_buf_type frame[MAX_SEND_FRAME_QUEUE];
}send_frame_queue_type;

typedef struct 
{
  int prev_magic;
  bool enable_check;
  double start_time;
  double delay_time;
}cmd_overtime_clock_type;

//upgrade report type
typedef struct
{   
    int main_module_id;
    std::string main_module_name;
    int module_id;
    std::string module_name;
    std::string bin_file_name;
    std::string update_time;
    double update_dura_time;
    int result;
    int Estage;
    std::string EStateMessage;
    std::string Ecode;
    std::string app_version_before;
    std::string app_version_after;
    bool motor_driver_main_bus_comm_disable;
    bool motor_driver_aux_bus_comm_disable;
} upgrade_report_type;

typedef struct __attribute__((packed))
{
    uint32_t firmware_size;
    uint16_t packet_length;
    uint16_t total_packets;
    uint16_t current_packet;
    uint16_t actual_length;
    uint32_t read_offset;
}firmware_packet_type;

#endif
