

#include "manage_updates.hpp"
#include "serial_cmd.hpp"
#include "serial_api.hpp"
#include "serial_api_al_linux.hpp"
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include<unistd.h>  
#include<fcntl.h>   //open read write
#include <ctype.h>

//全局变量
signed char update_option = 0;
signed char main_update_option = 0;
unsigned char *specified_bin_file_name = NULL;
upgrade_report_type report = {0};
//modoule index in array in a single update.
int update_module_idx = 0;
//初始化升级状态
unsigned char manage_update_state = CHECK_MOVE_CONTROL_VERSION;
//初始化升级升级超时检测
cmd_overtime_clock_type  cmd_overtime = {0};
//清空超时计时器
void clear_cmd_overtime_clock(int magic)
{
    if(magic == cmd_overtime.prev_magic)
    {
        memset(&cmd_overtime, 0, sizeof(cmd_overtime_clock_type));
    }
}
//设置超时计时器
void set_cmd_overtime_clock(int magic, unsigned long delay_time)
{
    if((cmd_overtime.prev_magic != 0) && (cmd_overtime.prev_magic != magic) )
    {
        clear_cmd_overtime_clock(cmd_overtime.prev_magic);
    }
    cmd_overtime.prev_magic = magic;
    cmd_overtime.delay_time = delay_time;
    gettimeofday(&cmd_overtime.start_time, NULL);
    cmd_overtime.enable_check = true;
}
//检查超时计时器
void check_cmd_overtime_clock(void)
{
    struct timeval tv;
    if(false == cmd_overtime.enable_check)
    {
        return;
    }
    gettimeofday(&tv, NULL);
    if(( (tv.tv_sec*1000 + tv.tv_usec/1000) - (cmd_overtime.start_time.tv_sec*1000 + cmd_overtime.start_time.tv_usec/1000) ) >= cmd_overtime.delay_time)
    {
        set_report_infomation(0, firmware_upgrade_flag, "Wait cmd response overtime.");
        write_upgrade_flag(FIRMWARE_ABORT_UPGRADE);
        //stop overtime _clock checking
        cmd_overtime.enable_check = false;
        
    }
}
//小写全部转大写
char *strupr(char *str)
{
   char *ptr = str;
   while (*ptr != '\0') {
       if (islower(*ptr)) 
            *ptr = toupper(*ptr);
       ptr++;
   }
   return str;
}
//清空重发命令
void clear_resend_cmd(unsigned int cmd)
{
    if( (snd_frame_queue.resend[snd_frame_queue.head] >> 8) == cmd)
    {
        snd_frame_queue.resend[snd_frame_queue.head] = 0;
    }
}
//单板升级控制
int do_single_firmware_update(unsigned int state, upgrade_result_type *result)
{
    //cmd overtime check
    check_cmd_overtime_clock();
    switch(state)
    {
        //roll query if a firmware_upgrade packet exists, send cmd_200, turns to FIRMWARE_PRE_UPDATE
        case FIRMWARE_NORMAL:
        {
            //manually set update tartget module ID. for test.
            //update_target_id = MODULE_MOVE_CONTROL_BOARD;
            update_target_id = result->module_id;
            upd_cmd_209_type upd_cmd_209 = {0};
            set_upd_info(&upd_cmd_209.upd_info, update_target_id);
            upd_cmd_209.check = 1;  //notice to check.
            construct_serial_frame(&serial_frame, UPD_CMD_209, (void *)&upd_cmd_209);
            pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_209 << 8) + MAX_CMD_RESEND_TIME);
            set_cmd_overtime_clock(UPD_CMD_209, MAX_CMD_OVERTIME_VALUE);
            write_upgrade_flag(FIRMWARE_WAIT_CHECK_BEFORE_VERSION);
        }
            break;

        case FIRMWARE_CHECK_VERSION_OK:
            if(-1 == check_firmware_file_exist())
            {
                DEBUG_PRINTF("Update file does not detected!!!\n");
                write_upgrade_flag(FIRMWARE_ABORT_UPGRADE);
                set_report_infomation(0, FIRMWARE_CHECK_VERSION_OK, "Update file not detected.");
                break;
            }     
            DEBUG_PRINTF("*******************Detect upgrade firmware file, begin to upgrade firmware.\n");     
            if(-1 != init_firmware_upgrade())
            {    
                upd_cmd_200_type upd_cmd_200 = {0};
                set_upd_info(&upd_cmd_200.upd_info, update_target_id);          
                upd_cmd_200.status = 0;  //notice to upgrade.
                upd_cmd_200.firmware_size = fw_packet.firmware_size;    
                construct_serial_frame(&serial_frame, UPD_CMD_200, (void *)&upd_cmd_200);
                pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_200 << 8) + MAX_CMD_RESEND_TIME);
                set_cmd_overtime_clock(UPD_CMD_200, MAX_CMD_OVERTIME_VALUE);
                write_upgrade_flag(FIRMWARE_PRE_UPDATE);
                DEBUG_PRINTF("In state FIRMWARE_NORMAL send cmd_200.\n");
            }
            else
            {
                write_upgrade_flag(FIRMWARE_ABORT_UPGRADE);
                set_report_infomation(0, FIRMWARE_CHECK_VERSION_OK, "Open upgrade file error.");
            }
            break;
        //wait respond cmd_200, turns to FIRMWARE_WAIT_START_UPDATE
        case FIRMWARE_PRE_UPDATE:  
            break;
        //wait receive cmd_202, turns to FIRMWARE_UPDATING
        case FIRMWARE_WAIT_START_UPDATE:
            break;
        //transfer cmd_203 firmware upgrade data
        case FIRMWARE_UPDATING:        
            break;
        //finish transfer
        case FIRMWARE_UPDATED_REBOOT:
            break;
        case FIRMWARE_WAIT_FINISH_UPDATE:
            break;
        case FIRMWARE_DELETE_UPGRADE_FILE:
            if(firmware_bin_file_fd != -1)
            {
                close(firmware_bin_file_fd);
            }
            //remove_firmware_file();
            write_upgrade_flag(FIRMWARE_UPGRADE_FINISHED);  
            set_report_infomation(1, FIRMWARE_UPGRADE_FINISHED, "Update file finished.");
            //exit(0);
            break;

        case FIRMWARE_ABORT_UPGRADE:
            if(firmware_bin_file_fd != -1)
            {
                close(firmware_bin_file_fd);
                firmware_bin_file_fd = -1;
            }
            write_upgrade_flag(FIRMWARE_UPGRADE_FINISHED);
            //remove_firmware_file();
            DEBUG_PRINTF2("Firmware upgrade failed, abort the module upgrade.\n");
            //exit(0);
            break;
        default:
            break;
              
    }
    return 0;
}
//检查升级的单板输入是否有效
int check_user_update_option(unsigned char *input_module)
{
    char name[MAX_FILE_NAME_SIZE] = {0};
    if(NULL == input_module)
    {
        return -1;
    }
    strncpy(name, (char *)input_module, MAX_FILE_NAME_SIZE - 1);  //avoid const string array to convert exception
    strupr(name);
    //all update
    if(0 == strcmp(name, "ALL"))
    {  
        printf("User input upgrade module: %s.\n", name);
        return 255;
    }
    //single update
    int i = 0;
    for(i = 0; i < MODULE_PARAM_TABLE_ELE_NUM; i++)
    {
        if(0 == strcmp(name, (char *)module_param_table[i].module_name))
        {
            printf("User input upgrade module: %s.\n", name);
            printf("i: %d\n",i);
            return i;
        }
    }
    printf("User input upgrade module invalid.\n");
    return -1;   
}
//检查升级的主板输入是否有效
int check_user_main_update_option(unsigned char *input_module)
{
    char name[MAX_FILE_NAME_SIZE] = {0};   
    if(NULL == input_module)
    {
        return -1;
    }
    strncpy(name, (char *)input_module, MAX_FILE_NAME_SIZE - 1);  //avoid const string array to convert exception
    strupr(name);
    int i = 0;
    for(i = 0; i < MODULE_PARAM_TABLE_ELE_NUM; i++)
    {
        if(0 == strcmp(name, (char *)module_param_table[i].module_name))
        {
            printf("User input upgrade main module: %s.\n", name);
            return i;
        }
    }
    printf("User input upgrade main module invalid.\n");
    return -1;  
}
//整体升级时的参数设定
void set_all_module_update_param(void)
{
    int i = 0;
    for(i = 0; i < MODULE_PARAM_TABLE_ELE_NUM; i++)
    {
        report.result[i].module_id = module_param_table[i].module_id;
        strncpy((char *)report.result[i].module_name, (char *)module_param_table[i].module_name, MAX_FILE_NAME_SIZE);
        strncpy((char *)report.result[i].bin_file_name, (char *)module_param_table[i].module_bin_file_name, MAX_BIN_FILE_NAME_SIZE);
    }
    report.update_cnt = MODULE_PARAM_TABLE_ELE_NUM;
    update_module_idx = 0;  //upgrade starts from slot 0 
}
//获取本地时间
void get_local_time(unsigned char *time_str)
{
    if(NULL == time_str)
    {
        return;
    }
    time_t raw_time;
    struct tm *tblock; 
    raw_time = time(NULL);
    tblock = localtime(&raw_time);
    strcpy((char *)time_str, asctime(tblock));
}
//设定升级后报告信息
void set_report_infomation(int result, int Estage, const char *Ecode)
{
    upgrade_result_type *upgrade_result = &report.result[update_module_idx];
    upgrade_result->result = result;
    upgrade_result->Estage = Estage;
    strncpy((char *)upgrade_result->Ecode, Ecode, (MAX_ERROR_CODE_STR_SIZE - 1)); 
    get_local_time(upgrade_result->update_time);
}
//设定升级前的报告信息
void set_report_infomation_ex(char *version_before, char *version_after)
{
    upgrade_result_type *upgrade_result = &report.result[update_module_idx];
    if(version_before != NULL)
    {
        strncpy( (char *)upgrade_result->app_version_before, version_before, MAX_VERSION_STR - 1);
    }
    if(version_after != NULL)
    {
        strncpy( (char *)upgrade_result->app_version_after, version_after, MAX_UPDATE_TIME_SIZE - 1);
    }
    get_local_time(upgrade_result->update_time);

}
//失败状态列表
const unsigned char Control_fail_stage_string[][48] = 
{
    {"firmware_wait_check_move_control_version"},
    {"fireware_wait_main_module_id_respond"},
    {"Normal"},
    {"Wait checking app version before"},
    {"Check version ok"},
    {"Pre update"},
    {"Athentication"},
    {"Waiting to start update"},
    {"Updating"},
    {"Updated and send reboot"},
    {"Wait finishing update"},
    {"Wait checking app version after"},
    {"Delete upgrade file"},
    {"Abort upgrade"},
    {"Upgrade finished"},
    {"Reserved"},
    {"Reserved"},
    {"Reserved"},
    {"Reserved"},
};
//打印升级报告
int report_module_update_status(void)
{
    int i = 0;
    int ret = -1;
    printf("**********************Report update module firmware results*************************\n");    
    for(i = 0; i < report.update_cnt; i++)
    {
        printf("#################Update module ####################\n");
        printf("main module name: %s\n",  report.result[i].main_module_name);
        printf("module name: %s\n",  report.result[i].module_name);
        printf("update bin file name: %s\n", report.result[i].bin_file_name);
        printf("update time: %s\n", report.result[i].update_time);
        printf("update result: %s\n", (report.result[i].result != 0)? "SUCCESS" : "FAIL" );
        ret = (report.result[i].result != 0)? 0 : -1;
        if(0 == report.result[i].result)
        {
            if( report.result[i].Estage < GET_ARRAY_ELE_NUM(Control_fail_stage_string) )
            {
                printf("Fail stage: In %s.\n", Control_fail_stage_string[report.result[i].Estage]);
            }
            else
            {
                printf("Fail stage: %d\n", report.result[i].Estage);
            }
            printf("Fail message: %s\n", report.result[i].Ecode);
        }

        printf("APP version before update : %s\n", report.result[i].app_version_before);
        printf("APP version after update : %s\n", report.result[i].app_version_after);
        printf("\n\n");
    }
    printf("**********************Report update module firmware results*************************\n");
    return ret;
}

int manage_module_updates(unsigned char *state, unsigned char option , unsigned char main_option)
{
    static struct timeval tv_start;
    struct timeval tv_check;
    switch(*state)
    {
        case CHECK_MOVE_CONTROL_VERSION:
            //查询运控版本
        {
            upd_cmd_209_type upd_cmd_209 = {0};
            set_upd_info(&upd_cmd_209.upd_info, MODULE_MOVE_CONTROL_BOARD);
            upd_cmd_209.check = 1;  //notice to check.
            construct_serial_frame(&serial_frame, UPD_CMD_209, (void *)&upd_cmd_209);
            pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_209 << 8) + MAX_CMD_RESEND_TIME);
            set_cmd_overtime_clock(UPD_CMD_209, MAX_CMD_OVERTIME_VALUE);
            write_upgrade_flag(FIRMWARE_WAIT_CHECK_MOVE_CONTROL_VERSION);
            *state = WAIT_MOVE_CONTROL_VERSION_RESPOND;
        }
            break;
        case WAIT_MOVE_CONTROL_VERSION_RESPOND:
        {
            check_cmd_overtime_clock();
        }
            break;
        case SEND_MAIN_MODULE_ID:
        {
            //发送需要升级的主板id
            upd_cmd_212_type upd_cmd_212 = {0};
            upd_cmd_212.update_node = module_param_table[main_option].module_id;
            set_upd_info(&upd_cmd_212.upd_info, MODULE_MOVE_CONTROL_BOARD);
            //set_upd_info(&upd_cmd_212.upd_info, main_update_option);
            construct_serial_frame(&serial_frame, UPD_CMD_212, (void *)&upd_cmd_212);
            pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_212 << 8) + MAX_CMD_RESEND_TIME);
            set_cmd_overtime_clock(UPD_CMD_212, MAX_CMD_OVERTIME_VALUE);
            write_upgrade_flag(FIRMWARE_WAIT_MAIN_MODULE_ID_RESPOND);
            *state = WAIT_MAIN_MODULE_ID_RESPOND;
        }
            break;
        case WAIT_MAIN_MODULE_ID_RESPOND:
        { 
            check_cmd_overtime_clock();
        }
            break;
        case INIT_UPDATE_LIST:  
            memset(&report, 0, sizeof(report));     
            if(255 == option )
            {
                set_all_module_update_param();
            }
            else
            {
                //single update
                report.update_cnt = 1;
                update_module_idx = 0;
                report.result[update_module_idx].main_module_id = module_param_table[main_option].module_id;
                strncpy((char *)report.result[update_module_idx].main_module_name, (char *)module_param_table[main_option].module_name, MAX_FILE_NAME_SIZE);
                report.result[update_module_idx].module_id = module_param_table[option].module_id;
                strncpy((char *)report.result[update_module_idx].module_name, (char *)module_param_table[option].module_name, MAX_FILE_NAME_SIZE);
                //got specified bin file name
                if(specified_bin_file_name != NULL)
                {
                    strncpy( (char *)report.result[update_module_idx].bin_file_name, \
                        (char *)specified_bin_file_name, MAX_BIN_FILE_NAME_SIZE - 1);
                }
                else
                {
                    strncpy((char *) report.result[update_module_idx].bin_file_name, \
                        (char *)module_param_table[option].module_bin_file_name, MAX_BIN_FILE_NAME_SIZE - 1);
                }
            }
            send_update_status_cmd(0, MODULE_MOVE_CONTROL_BOARD);
            gettimeofday(&tv_start, NULL);
            *state = INIT_SINGLE_UPDATE_PARAM;
            break;
        case INIT_SINGLE_UPDATE_PARAM:
            gettimeofday(&tv_check, NULL);
            if(( (tv_check.tv_sec*1000 + tv_check.tv_usec/1000) - (tv_start.tv_sec*1000 + tv_start.tv_usec/1000) ) >= 2000)
            {
                write_upgrade_flag(FIRMWARE_NORMAL);
                strncpy((char *)FIRMWARE_BIN_FILE, \
                (char *)report.result[update_module_idx].bin_file_name, MAX_BIN_FILE_NAME_SIZE - 1);

                *state = DO_UPDATE;
            }
            break;
        case DO_UPDATE:
            if( (report.result[update_module_idx].module_id >= MODULE_MOTOR_DRIVER_FRONT_LEFT) \
              &&(report.result[update_module_idx].module_id <= MODULE_MOTOR_DRIVER_BACK_RIGHT) \
              &&(false == report.motor_driver_main_bus_comm_disable))
            {
                usleep(20000);

                int i = 0;
                for(i = MODULE_MOTOR_DRIVER_FRONT_LEFT; i <= MODULE_MOTOR_DRIVER_BACK_RIGHT; i++)
                {
                    send_update_status_cmd_once(0, i);
                    usleep(20000);
                    
                }
                report.motor_driver_main_bus_comm_disable = true;
            }

            if( (report.result[update_module_idx].module_id >= MODULE_MOTOR_TURN_FRONT) \
              &&(report.result[update_module_idx].module_id <= MODULE_MOTOR_BREAK_BACK) \
              &&(0 == report.motor_driver_aux_bus_comm_disable))
            {
                usleep(20000);
               
                int i = 0;
                for(i = MODULE_MOTOR_TURN_FRONT; i <= MODULE_MOTOR_BREAK_BACK; i++)
                {
                    send_update_status_cmd_once(0, i);
                    usleep(20000);
                    
                }
                report.motor_driver_aux_bus_comm_disable = true;
            }        
            do_single_firmware_update(firmware_upgrade_flag, &(report.result[update_module_idx]) );
            if(FIRMWARE_UPGRADE_FINISHED == firmware_upgrade_flag)
            {
                *state = RECORD_STATUS;
            }
            break;

        case RECORD_STATUS:
            update_module_idx++;
            if(update_module_idx >= report.update_cnt)
            {
                *state = REPORT_STATUS;
				
                unsigned char ret = 2;  //update success
                unsigned char i;
                for(i = 0; i < report.update_cnt; i++)
                {
                    if(0 == report.result[i].result)
                    {
                        ret = 3;  //update fail
                    }
                }

                if(true == report.motor_driver_main_bus_comm_disable)
                {
                    usleep(20000);                   
 
                    int i = 0;
                    for(i = MODULE_MOTOR_DRIVER_FRONT_LEFT; i <= MODULE_MOTOR_DRIVER_BACK_RIGHT; i++)
                    {
                        send_update_status_cmd_once(ret, i);  //2 or 3
                        usleep(20000);
                        
                    }
                    report.motor_driver_main_bus_comm_disable = false;
                }
    
                if(true == report.motor_driver_aux_bus_comm_disable)
                {
                    usleep(20000);
                    int i = 0;
                    for(i = MODULE_MOTOR_TURN_FRONT; i <= MODULE_MOTOR_BREAK_BACK; i++)
                    {
                        send_update_status_cmd_once(ret, i);  //2 or 3
                        usleep(20000);
                    }
                    report.motor_driver_aux_bus_comm_disable = false;
                }

 
                send_update_status_cmd(ret , MODULE_MOVE_CONTROL_BOARD);
                gettimeofday(&tv_start, NULL);
            }
            else
            {
                *state = INIT_SINGLE_UPDATE_PARAM;
            }
            break;

        case REPORT_STATUS:
            gettimeofday(&tv_check, NULL);
            if(( (tv_check.tv_sec*1000 + tv_check.tv_usec/1000) - (tv_start.tv_sec*1000 + tv_start.tv_usec/1000) ) >= 4000)
            {
                int ret = report_module_update_status();
                exit(ret);
            }    
            break;
        default:
            break;      
    }
    return 0;
}



