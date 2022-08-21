#ifndef SERIAL_API_AL_H
#define SERIAL_API_AL_H

#include "serial_cmd.hpp"

//dest_main communication module definition
enum
{
    MAIN_MODULE_MOVE_CONTROL_BOARD = 1,
    MAIN_MODULE_SWEEPING_BOX_BOARD = 2,
    MAIN_MODULE_DUST_BOX_BOARD = 3,
    MAIN_MAX_MODULE,
};

//global communicating module definition
//**************************各模块地址
enum
{
    MODULE_NONE = 0,
    MODULE_DEBUG_BOARD = 1,  //10-10 belongs to normal moduls   
    MODULE_CONTROL_PC = 2,
    MODULE_MOVE_CONTROL_BOARD = 3,
    MODULE_POWER_INTEGRATE_BOARD = 4,
    MODULE_SWEEPING_BOX_BOARD =5,
    MODULE_DUST_BOX_BOARD =6,
    MODULE_GPS = 7,
    MODULE_POWER_BOARD = 8,
    
    MODULE_MOTOR_DRIVER_START = 10,  //10-29 belongs to motor drivers
    MODULE_MOTOR_DRIVER_FRONT_LEFT = MODULE_MOTOR_DRIVER_START,
    MODULE_MOTOR_DRIVER_FRONT_RIGHT = 11,
    MODULE_MOTOR_DRIVER_BACK_LEFT = 12,
    MODULE_MOTOR_DRIVER_BACK_RIGHT = 13,
    MODULE_MOTOR_TURN_FRONT = 14,
    MODULE_MOTOR_TURN_BACK = 15,
    MODULE_MOTOR_BRAKE_FRONT = 16,
    MODULE_MOTOR_BREAK_BACK = 17,
    MODULE_MOTOR_DRIVER_END = MODULE_MOTOR_BREAK_BACK,
    
    MODULE_LIGHT_START = 30,  //30-49 belongs to lights
    MODULE_LIGHT_FRONT_LEFT = MODULE_LIGHT_START,
    MODULE_LIGHT_FRONT_RIGHT = 31,
    MODULE_LIGHT_BACK_LEFT = 32,
    MODULE_LIGHT_BACK_RIGHT = 33,
    MODULE_LIGHT_BACK = 34,
    MODULE_LIGHT_HEAD = 35,
    MODULE_LIGHT_END = MODULE_LIGHT_HEAD,
    MODULE_LIGHT_TAPE = 39,

    MODULE_PUSH_ROD_START = 40,
    MODULE_PUSH_ROD_0 = MODULE_PUSH_ROD_START,
    MODULE_PUSH_ROD_1 = 41,
    MODULE_PUSH_ROD_2 = 42,
    MODULE_PUSH_ROD_3 = 43,
    MODULE_PUSH_ROD_4 = 44,
    MODULE_PUSH_ROD_5 = 45,
    MODULE_PUSH_ROD_6 = 46,
    MODULE_PUSH_ROD_7 = 47,
    MODULE_PUSH_ROD_8 = 48,
    MODULE_PUSH_ROD_9 = 49,
    MODULE_PUSH_ROD_10 = 50,
    MODULE_PUSH_ROD_11 = 51,
    MODULE_PUSH_ROD_12 = 52,
    MODULE_PUSH_ROD_13 = 53,
    MODULE_PUSH_ROD_14 = 54,
    MODULE_PUSH_ROD_15 = 55,
    MODULE_PUSH_ROD_END = MODULE_PUSH_ROD_15,

    MODULE_ULTRASONIC_START = 60,
    MODULE_ULTRASONIC_MASTER = MODULE_ULTRASONIC_START,
    MODULE_ULTRASONIC_1 = 61,
    MODULE_ULTRASONIC_2 = 62,
    MODULE_ULTRASONIC_3 = 63,
    MODULE_ULTRASONIC_4 = 64,
    MODULE_ULTRASONIC_5 = 65,
    MODULE_ULTRASONIC_6 = 66,
    MODULE_ULTRASONIC_7 = 67,
    MODULE_ULTRASONIC_8 = 68,
    MODULE_ULTRASONIC_9 = 69,
    MODULE_ULTRASONIC_10 = 70,
    MODULE_ULTRASONIC_11 = 71,
    MODULE_ULTRASONIC_12 = 72,
    MODULE_ULTRASONIC_13 = 73,
    MODULE_ULTRASONIC_14 = 74,
    MODULE_ULTRASONIC_15 = 75,
    MODULE_ULTRASONIC_END,

    MODULE_BELOW_START = 80,  //ohter modules start
    MODULE_CHECK_UPD = 81,
    MAX_MODULE,
};


/********USER CODE BEGIN********/

#define BOOTLOADER_VERSION      "BOOT_V0.02" " " __TIME__
#define APP_VERSION             "APP_V0.02" " " __TIME__

#define CONTROL_VERSION             "CTRL_V0.041" " " __DATE__ " " __TIME__

#define MODULE_SELF               MODULE_CONTROL_PC

#define MOVE_CONTROL_VERSIOM_FIRST_MIN 2
#define MOVE_CONTROL_VERSIOM_SECOND_MIN 0
#define MOVE_CONTROL_VERSIOM_THIRD_MIN 0


//upgrade cmd routing port 
//relation with upgrade cmd routing port router_map[].rout_port
//relation with raw_rx[].port_idx, so start with 0 and do not jump!!! do not define unneed port!!
enum
{
    PORT_USART1 = 0,  //MUST START WITH 0 AND DO NOT JUMP!!!
                      //idx 0 is main port to communicate with update cmd data
    PORT_UART2,
    PORT_USB,
    PORT_RJ45,
    PORT_COMM_MAX,
};


/********USER CODE END*********/


typedef struct
{
    unsigned char dest_module;
    unsigned char rout_port;
}router_map_type;


#define DEVICE_PORT_STR_LENGTH          256
#define UPDATE_FILE_STR_LENGTH          512

extern unsigned char SERIAL_DEVICE_PORT[];

extern unsigned char FIRMWARE_BIN_FILE[];

#define MAX_CMD_RESEND_TIME             10

#define FIRMWARE_PACKET_SIZE            256

/*the flag state of firmware upgrade*/
enum
{
    FIRMWARE_WAIT_CHECK_MOVE_CONTROL_VERSION = 0,
    FIRMWARE_WAIT_MAIN_MODULE_ID_RESPOND,
    FIRMWARE_NORMAL,
    FIRMWARE_WAIT_CHECK_BEFORE_VERSION,
    FIRMWARE_CHECK_VERSION_OK,
    FIRMWARE_PRE_UPDATE,     //send cmd_200 cmd_201 to shake hands
    FIRMWARE_ATHENTICATION,  //athentication  cmd_201
    FIRMWARE_WAIT_START_UPDATE,  //wait reboot, wait cmd_202
    FIRMWARE_UPDATING,           //cmd_203 and wait cmd_204 
    FIRMWARE_UPDATED_REBOOT,     // send cmd_205 and wait cmd_205
    FIRMWARE_WAIT_FINISH_UPDATE, //wait cmd_206
    FIRMWARE_WAIT_CHECK_AFTER_VERSION,
    FIRMWARE_DELETE_UPGRADE_FILE,
    FIRMWARE_ABORT_UPGRADE,
    FIRMWARE_UPGRADE_FINISHED,

};


#define GET_ARRAY_ELE_NUM(x)  (sizeof(x)/sizeof(x[0]))

typedef struct __attribute__((packed))
{
    uint32_t firmware_size;
    uint16_t packet_length;
    uint16_t total_packets;
    uint16_t current_packet;
    uint16_t actual_length;

    uint32_t read_offset;
}firmware_packet_type;

extern firmware_packet_type fw_packet;

extern uint16_t firmware_upgrade_flag;

extern const unsigned char serial_frame_header[];

/*!!!add all the defined cmd type data length here.*/
extern const cmd_length_table_type cmd_data_length[];

extern int firmware_bin_file_fd;

extern unsigned char update_target_id;



extern int get_cmd_data_length(unsigned char cmd_id);
extern void set_upd_info(update_info_type *info, unsigned char target_id);

extern int open_serial_port(void);
extern void colse_serial_port(int fd);
extern int send_serial_flow(int fd, unsigned char *src, int cnt);
extern int recv_serial_flow(int fd, unsigned char *dest, int cnt);

extern void write_upgrade_flag(uint16_t flag);


extern int check_firmware_file_exist(void);
extern int remove_firmware_file(void);
extern int init_firmware_upgrade(void);



extern int process_cmd(serial_frame_type *frame);


extern void send_update_status_cmd(unsigned char status, unsigned char target_id);
extern void send_update_status_cmd_once(unsigned char status, unsigned char target_id);


#endif

