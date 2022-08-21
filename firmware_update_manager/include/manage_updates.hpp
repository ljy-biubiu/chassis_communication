#ifndef MANAGE_UPDATES_H
#define MANAGE_UPDATES_H
#include <stdbool.h>
#include "serial_cmd.hpp"
#include "serial_api_al_linux.hpp"
#include <sys/time.h>
#include <time.h>

#define MAX_BIN_FILE_NAME_SIZE          256
#define MAX_FILE_NAME_SIZE              MAX_BIN_FILE_NAME_SIZE
#define MAX_ERROR_CODE_STR_SIZE         48
#define MAX_UPDATE_TIME_SIZE            32
#define MAX_CMD_OVERTIME_VALUE          20000    //5s


//manage updates state
enum
{
   CHECK_MOVE_CONTROL_VERSION = 0,
   WAIT_MOVE_CONTROL_VERSION_RESPOND,
   SEND_MAIN_MODULE_ID,
   WAIT_MAIN_MODULE_ID_RESPOND,
   INIT_UPDATE_LIST,
   INIT_SINGLE_UPDATE_PARAM,
   //CHECK_VERSION,
   DO_UPDATE,
   RECORD_STATUS,
   REPORT_STATUS,
};

//single update result
typedef struct
{   
    unsigned char main_module_id;
    unsigned char main_module_name[MAX_FILE_NAME_SIZE];
    unsigned char module_id;
    unsigned char module_name[MAX_FILE_NAME_SIZE];
    unsigned char bin_file_name[MAX_BIN_FILE_NAME_SIZE];
    unsigned char update_time[MAX_UPDATE_TIME_SIZE];
    int result;
    int Estage;
    unsigned char Ecode[MAX_ERROR_CODE_STR_SIZE];
    unsigned char app_version_before[MAX_VERSION_STR];
    unsigned char app_version_after[MAX_VERSION_STR];
}upgrade_result_type;


//set of update results
typedef struct
{
 unsigned char update_cnt;
 upgrade_result_type result[MAX_MODULE];
 unsigned char motor_driver_main_bus_comm_disable;
 unsigned char motor_driver_aux_bus_comm_disable;
}upgrade_report_type;


//input param table
typedef struct
{
    unsigned char module_id;
    unsigned char module_name[MAX_FILE_NAME_SIZE];
    unsigned char module_bin_file_name[MAX_BIN_FILE_NAME_SIZE];
}module_param_table_type;

const module_param_table_type module_param_table[] = 
{
    //module_id                         //update module name                //bin file name
    {MODULE_DEBUG_BOARD,                "MODULE_DEBUG_BOARD",               "MODULE_DEBUG_BOARD.BIN"},
    //no need to upgrade self
    //{MODULE_CONTROL_PC,                 "MODULE_CONTROL_PC",                "MODULE_CONTROL_PC.BIN"},
    {MODULE_MOVE_CONTROL_BOARD,         "MODULE_MOVE_CONTROL_BOARD",        "MODULE_MOVE_CONTROL_BOARD.BIN"},
    {MODULE_POWER_INTEGRATE_BOARD,      "MODULE_POWER_INTEGRATE_BOARD",     "MODULE_POWER_INTEGRATE_BOARD.BIN"},
    {MODULE_SWEEPING_BOX_BOARD,      "MODULE_SWEEPING_BOX_BOARD",     "MODULE_SWEEPING_BOX_BOARD.BIN"},
    {MODULE_DUST_BOX_BOARD,      "MODULE_DUST_BOX_BOARD",     "MODULE_DUST_BOX_BOARD.BIN"},
    {MODULE_GPS,      "MODULE_GPS",     "MODULE_GPS.BIN"},
    {MODULE_POWER_BOARD,      "MODULE_POWER_BOARD",     "MODULE_POWER_BOARD.BIN"},

    {MODULE_MOTOR_DRIVER_FRONT_LEFT,    "MODULE_MOTOR_DRIVER_FRONT_LEFT",   "MODULE_MOTOR_DRIVER_FRONT_LEFT.BIN"},
    {MODULE_MOTOR_DRIVER_FRONT_RIGHT,   "MODULE_MOTOR_DRIVER_FRONT_RIGHT",  "MODULE_MOTOR_DRIVER_FRONT_RIGHT.BIN"},
    {MODULE_MOTOR_DRIVER_BACK_LEFT,     "MODULE_MOTOR_DRIVER_BACK_LEFT",    "MODULE_MOTOR_DRIVER_BACK_LEFT.BIN"},
    {MODULE_MOTOR_DRIVER_BACK_RIGHT,    "MODULE_MOTOR_DRIVER_BACK_RIGHT",   "MODULE_MOTOR_DRIVER_BACK_RIGHT.BIN"},
    {MODULE_MOTOR_TURN_FRONT,           "MODULE_MOTOR_TURN_FRONT",          "MODULE_MOTOR_TURN_FRONT.BIN"},
    {MODULE_MOTOR_TURN_BACK,            "MODULE_MOTOR_TURN_BACK",           "MODULE_MOTOR_TURN_BACK.BIN"},
    {MODULE_MOTOR_BRAKE_FRONT,          "MODULE_MOTOR_BRAKE_FRONT",         "MODULE_MOTOR_BRAKE_FRONT.BIN"},
    {MODULE_MOTOR_BREAK_BACK,           "MODULE_MOTOR_BREAK_BACK",          "MODULE_MOTOR_BREAK_BACK.BIN"},

    {MODULE_LIGHT_FRONT_LEFT,           "MODULE_LIGHT_FRONT_LEFT",          "MODULE_LIGHT_FRONT_LEFT.BIN"},
    {MODULE_LIGHT_FRONT_RIGHT,          "MODULE_LIGHT_FRONT_RIGHT",         "MODULE_LIGHT_FRONT_RIGHT.BIN"},
    {MODULE_LIGHT_BACK_LEFT,            "MODULE_LIGHT_BACK_LEFT",           "MODULE_LIGHT_BACK_LEFT.BIN"},
    {MODULE_LIGHT_BACK_RIGHT,           "MODULE_LIGHT_BACK_RIGHT",          "MODULE_LIGHT_BACK_RIGHT.BIN"},
    {MODULE_LIGHT_BACK,                 "MODULE_LIGHT_BACK",                "MODULE_LIGHT_BACK.BIN"},
    {MODULE_LIGHT_HEAD,                 "MODULE_LIGHT_HEAD",                "MODULE_LIGHT_HEAD.BIN"},
    {MODULE_LIGHT_TAPE,                 "MODULE_LIGHT_TAPE",                "MODULE_LIGHT_TAPE.BIN"},	

    {MODULE_PUSH_ROD_0,          "MODULE_PUSH_ROD_0",         "MODULE_PUSH_ROD_0.BIN"},
    {MODULE_PUSH_ROD_1,          "MODULE_PUSH_ROD_1",         "MODULE_PUSH_ROD_1.BIN"},
    {MODULE_PUSH_ROD_2,          "MODULE_PUSH_ROD_2",         "MODULE_PUSH_ROD_2.BIN"},
    {MODULE_PUSH_ROD_3,          "MODULE_PUSH_ROD_3",         "MODULE_PUSH_ROD_3.BIN"},
    {MODULE_PUSH_ROD_4,          "MODULE_PUSH_ROD_4",         "MODULE_PUSH_ROD_4.BIN"},
    {MODULE_PUSH_ROD_5,          "MODULE_PUSH_ROD_5",         "MODULE_PUSH_ROD_5.BIN"},
    {MODULE_PUSH_ROD_6,          "MODULE_PUSH_ROD_6",         "MODULE_PUSH_ROD_6.BIN"},
    {MODULE_PUSH_ROD_7,          "MODULE_PUSH_ROD_7",         "MODULE_PUSH_ROD_7.BIN"},
    {MODULE_PUSH_ROD_8,          "MODULE_PUSH_ROD_8",         "MODULE_PUSH_ROD_8.BIN"},
    {MODULE_PUSH_ROD_9,          "MODULE_PUSH_ROD_9",         "MODULE_PUSH_ROD_9.BIN"},
    {MODULE_PUSH_ROD_10,          "MODULE_PUSH_ROD_10",         "MODULE_PUSH_ROD_10.BIN"},
    {MODULE_PUSH_ROD_11,          "MODULE_PUSH_ROD_11",         "MODULE_PUSH_ROD_11.BIN"},
    {MODULE_PUSH_ROD_12,          "MODULE_PUSH_ROD_12",         "MODULE_PUSH_ROD_12.BIN"},
    {MODULE_PUSH_ROD_13,          "MODULE_PUSH_ROD_13",         "MODULE_PUSH_ROD_13.BIN"},
    {MODULE_PUSH_ROD_14,          "MODULE_PUSH_ROD_14",         "MODULE_PUSH_ROD_14.BIN"},
    {MODULE_PUSH_ROD_15,          "MODULE_PUSH_ROD_15",         "MODULE_PUSH_ROD_15.BIN"},

    {MODULE_ULTRASONIC_MASTER,          "MODULE_ULTRASONIC_MASTER",         "MODULE_ULTRASONIC_MASTER.BIN"},
    {MODULE_ULTRASONIC_1,               "MODULE_ULTRASONIC_1",              "MODULE_ULTRASONIC_1.BIN"},
    {MODULE_ULTRASONIC_2,               "MODULE_ULTRASONIC_2",              "MODULE_ULTRASONIC_2.BIN"},
    {MODULE_ULTRASONIC_3,               "MODULE_ULTRASONIC_3",              "MODULE_ULTRASONIC_3.BIN"},
    {MODULE_ULTRASONIC_4,               "MODULE_ULTRASONIC_4",              "MODULE_ULTRASONIC_4.BIN"},
    {MODULE_ULTRASONIC_5,               "MODULE_ULTRASONIC_5",              "MODULE_ULTRASONIC_5.BIN"},
    {MODULE_ULTRASONIC_6,               "MODULE_ULTRASONIC_6",              "MODULE_ULTRASONIC_6.BIN"},
    {MODULE_ULTRASONIC_7,               "MODULE_ULTRASONIC_7",              "MODULE_ULTRASONIC_7.BIN"},
    {MODULE_ULTRASONIC_8,               "MODULE_ULTRASONIC_8",              "MODULE_ULTRASONIC_8.BIN"},
    {MODULE_ULTRASONIC_9,               "MODULE_ULTRASONIC_9",              "MODULE_ULTRASONIC_9.BIN"},
    {MODULE_ULTRASONIC_10,              "MODULE_ULTRASONIC_10",             "MODULE_ULTRASONIC_10.BIN"},
    {MODULE_ULTRASONIC_11,              "MODULE_ULTRASONIC_11",             "MODULE_ULTRASONIC_11.BIN"},
    {MODULE_ULTRASONIC_12,              "MODULE_ULTRASONIC_12",             "MODULE_ULTRASONIC_12.BIN"},
    {MODULE_ULTRASONIC_13,              "MODULE_ULTRASONIC_13",             "MODULE_ULTRASONIC_13.BIN"},
    {MODULE_ULTRASONIC_14,              "MODULE_ULTRASONIC_14",             "MODULE_ULTRASONIC_14.BIN"},
    {MODULE_ULTRASONIC_15,              "MODULE_ULTRASONIC_15",             "MODULE_ULTRASONIC_15.BIN"},
};
#define MODULE_PARAM_TABLE_ELE_NUM  (sizeof(module_param_table)/sizeof(module_param_table[0]))
typedef struct
{
    unsigned char main_module_id;
    unsigned char main_module_name[MAX_FILE_NAME_SIZE];   
}main_module_param_table_type;

const main_module_param_table_type main_module_param_table[] = 
{
    //main_module_id                         //update main module name                
    {MAIN_MODULE_MOVE_CONTROL_BOARD,                  "MAIN_MODULE_MOVE_CONTROL_BOARD"},
    {MAIN_MODULE_SWEEPING_BOX_BOARD,                  "MAIN_MODULE_SWEEPING_BOX_BOARD"},
    {MAIN_MODULE_DUST_BOX_BOARD,                          "MAIN_MODULE_DUST_BOX_BOARD"},
};
#define MAIN_MODULE_PARAM_TABLE_ELE_NUM  (sizeof(main_module_param_table)/sizeof(main_module_param_table[0]))
typedef struct 
{

  int prev_magic;
  bool enable_check;
  struct timeval start_time;
  int unsigned long delay_time;
}cmd_overtime_clock_type;

extern cmd_overtime_clock_type  cmd_overtime;
extern unsigned char manage_update_state;
extern signed char update_option;
extern signed char main_update_option;
extern unsigned char *specified_bin_file_name;

extern void clear_cmd_overtime_clock(int magic);
extern void set_cmd_overtime_clock(int magic, unsigned long delay_time);
extern void check_cmd_overtime_clock(void);
extern void clear_resend_cmd(unsigned int cmd);
extern void set_report_infomation(int result, int Estage, const char *Ecode);
extern void set_report_infomation_ex(char *version_before, char *version_after);
extern char *strupr(char *str);
extern int check_user_update_option(unsigned char *input_module);
extern int check_user_main_update_option(unsigned char *input_module);
extern int manage_module_updates(unsigned char *state, unsigned char option , unsigned char main_option);




#endif

