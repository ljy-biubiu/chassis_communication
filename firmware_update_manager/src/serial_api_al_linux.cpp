

/*linux header*/
#include<termios.h>     //struct termios

#include<stdio.h>       //delete file
#include<stdlib.h>      //perror printf

#include<unistd.h>  
#include<fcntl.h>       //open close read write

#include<string.h>      //memset

/*common header*/
#include "serial_api_al_linux.hpp"
#include "serial_api.hpp"
#include "serial_cmd.hpp"
#include "crc16.hpp"
#include "manage_updates.hpp"

//linux header
#include <inttypes.h>    //or stdint.h uint16_t uint32_t

#include <sys/types.h> 
#include <unistd.h>  //file operation open read write lseek close
#include<fcntl.h> 



unsigned char SERIAL_DEVICE_PORT[DEVICE_PORT_STR_LENGTH] = {0};

unsigned char FIRMWARE_BIN_FILE[UPDATE_FILE_STR_LENGTH] = {0};

unsigned char update_target_id = 0;


unsigned char bootloader_version[MAX_VERSION_STR] = {0};
unsigned char app_version[MAX_VERSION_STR] = {0};

extern signed char update_option;

/*the serial frame header definition.*/
const unsigned char serial_frame_header[FRAME_HEADER_SIZE] = {'C', 'T', 'R', 'L'};

uint16_t firmware_upgrade_flag = FIRMWARE_NORMAL;

/*!!!add all the defined cmd type data length here.*/
const cmd_length_table_type cmd_data_length[] =
{
    {CMD_NORMAL,    sizeof(cmd_normal_type)},
    /********USER CODE BEGIN********/

    /********USER CODE END*********/
    {UPD_CMD_200,   sizeof(upd_cmd_200_type)},
    {UPD_CMD_201,   sizeof(upd_cmd_201_type)},
    {UPD_CMD_202,   sizeof(upd_cmd_202_type)},
    {UPD_CMD_203,   sizeof(upd_cmd_203_type)},
    {UPD_CMD_204,   sizeof(upd_cmd_204_type)},
    {UPD_CMD_205,   sizeof(upd_cmd_205_type)},
    {UPD_CMD_206,   sizeof(upd_cmd_206_type)},
    {UPD_CMD_209,   sizeof(upd_cmd_209_type)},
    {UPD_CMD_210,   sizeof(upd_cmd_210_type)},
    {UPD_CMD_211,   sizeof(upd_cmd_211_type)},
    {UPD_CMD_212,   sizeof(upd_cmd_212_type)},
};

const router_map_type router_map[] = 
{
    /********USER CODE BEGIN********/

    {MODULE_CONTROL_PC,                     PORT_RJ45},
    {MODULE_MOTOR_DRIVER_FRONT_LEFT,        PORT_USART1},
    {MODULE_MOTOR_DRIVER_FRONT_RIGHT,       PORT_UART2},

    /********USER CODE END*********/
    
};
    


int get_cmd_data_length(unsigned char cmd_id)
{
    int i = 0;

    for(i = 0; i< sizeof(cmd_data_length)/sizeof(cmd_data_length[0]); i++)
    {
        if(cmd_id == cmd_data_length[i].cmd_id)
        {
            return cmd_data_length[i].cmd_length;
        }
    }

    return -1;
}




int get_cmd_router_port(unsigned char dest_module_id)
{
    int i = 0;

    for(i = 0; i< sizeof(router_map)/sizeof(router_map[0]); i++)
    {
        if(dest_module_id == router_map[i].dest_module)
        {
            return router_map[i].rout_port;
        }
    }

    return -1;
}


firmware_packet_type fw_packet = {0};


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
int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)  
{  
    struct termios newtio,oldtio; 
    
    if  ( tcgetattr( fd,&oldtio)  !=  0) 
    {  
        perror("Get Serial attr failed.");  
        return -1;  
    } 
    
    memset( &newtio, 0, sizeof( newtio ) );  
    newtio.c_cflag  |=  CLOCAL | CREAD; //CLOCAL:disable the modem control line; CREAD:open receiver.  
    newtio.c_cflag &= ~CSIZE; //data bit mask:S5,S6,S7 or CS8  
  
    switch( nBits )  
    {  
        case 7:  
            newtio.c_cflag |= CS7;  
            break;
        
        case 8:  
            newtio.c_cflag |= CS8;  
            break;  
    }  
  
    switch( nEvent )  
    {  
        case 'O':  
            newtio.c_cflag |= PARENB; //enable parity check  
            newtio.c_cflag |= PARODD;  //odd 
            newtio.c_iflag |= (INPCK | ISTRIP); // INPACK:enable input parity check; ISTRIP:discard 8th bit  
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
  
    switch( nSpeed )  
    {  
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
  
    if(1 == nStop)
    {
        newtio.c_cflag &=  ~CSTOPB; //set 0 to enable 1 stop bit
    }
    else if (2 == nStop) 
    {
        newtio.c_cflag |=  CSTOPB;  //set 1 to enable 2 stop bit
    }

    //newtio.c_cc[VTIME]  = 1; //VTIME: ready delay in noncannoical mode, uint in 1/10 sec.
    //newtio.c_cc[VMIN] = SERIAL_FRAME_SIZE; //VMIN:minimum received char count in noncanonical mode.
    newtio.c_cc[VTIME]  = 0; //VTIME: ready delay in noncannoical mode, uint in 1/10 sec.
    newtio.c_cc[VMIN] = 0; //VMIN:minimum received char count in noncanonical mode.
    tcflush(fd,TCIFLUSH); // adopt change after all fd file has transfered. discard all received but not read data before adopt change.
    
    if((tcsetattr(fd, TCSANOW, &newtio)) != 0) //TCSANOW:adopt change immediately.  
    {  
        perror("Serial attr set error");  
        return -1;  
    }
    
    DEBUG_PRINTF("set done!\n\r");  
    return 0;
    
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
int do_open_serial(void)
{
    int fd1, ret;  

    /*Open device port*/
    fd1 = open( (char *)SERIAL_DEVICE_PORT, O_RDWR | O_NOCTTY);  
    if (fd1 == -1)
    {
        DEBUG_PRINTF("Port %s not ready.\n", SERIAL_DEVICE_PORT);
        return -1; 
    }
    DEBUG_PRINTF("open %s success.\n", SERIAL_DEVICE_PORT);  

    /*Set device*/
    //ret = set_opt(fd1, 115200, 8, 'N', 1);  
    ret = set_opt(fd1, 115200, 8, 'N', 1); 
    if (ret == -1)
    {
        DEBUG_PRINTF("SET %s failed.\n", SERIAL_DEVICE_PORT);  
        return -1; 
    }
    
    DEBUG_PRINTF("SET %s success.\n", SERIAL_DEVICE_PORT);
    return fd1;

}


/*****************************************************************************
 Prototype    : open_serial_port
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
int open_serial_port(void)
{
    int fd1;

    if(module_serial_fd != -1)
    {
        return module_serial_fd;
    }

    fd1 = 0;
    
    //while(-1 == (fd1 = do_open_serial()))
    //{
    //    usleep(1000000UL);
    //}

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
void colse_serial_port(int fd)
{
    //close(fd);
    module_serial_fd = 0;
}


extern int ros_buffer_write(unsigned char *src, int cnt);
extern int ros_buffer_read(unsigned char *dest, int cnt);


int send_serial_flow(int fd, unsigned char *src, int cnt)
{
    int ret = 0;
    int temp = cnt;

    while(temp)
    {
        ret = ros_buffer_write(src, temp); //write(fd, src, temp);
        temp -= ret;
    }

    //test
     DEBUG_PRINTF("send out buf size %d.\n", cnt);

    return cnt;
    
}


int recv_serial_flow(int fd, unsigned char *dest, int cnt)
{
    //return read(fd, dest,  cnt);
    return ros_buffer_read(dest, cnt);
}


uint16_t read_upgrade_flag(void)
{
    return firmware_upgrade_flag;
}


void write_upgrade_flag(uint16_t flag)
{
    firmware_upgrade_flag = flag;
}


uint32_t sp_total_crc = 0;

int init_firmware_upgrade_params(unsigned long size)
{   
    char extra_packet = 0;
    
    memset(&fw_packet, 0, sizeof(firmware_packet_type));

    //\B6\C1ȡ\CEļ\FE\A3\AC\BB\F1\B5\C3\CEļ\FE\B4\F3С
    fw_packet.firmware_size = size;
    fw_packet.packet_length = FIRMWARE_PACKET_SIZE;

    extra_packet = (fw_packet.firmware_size % fw_packet.packet_length) ? 1 : 0;
    
    fw_packet.total_packets = fw_packet.firmware_size / fw_packet.packet_length + extra_packet;
    fw_packet.current_packet = extra_packet;

    fw_packet.actual_length =  (fw_packet.current_packet == fw_packet.total_packets) ? \
            (fw_packet.firmware_size % fw_packet.packet_length) : fw_packet.packet_length;

    fw_packet.read_offset = 0;

    sp_total_crc = 0;

    //test
    DEBUG_PRINTF2("fw_packet.firmware_size: %d\r\n", fw_packet.firmware_size);
    DEBUG_PRINTF2("fw_packet.current_packet: %d\r\n", fw_packet.current_packet);
    DEBUG_PRINTF2("fw_packet.total_packets: %d\r\n",  fw_packet.total_packets);
    DEBUG_PRINTF2("fw_packet.packet_length: %d\r\n", fw_packet.packet_length);
    DEBUG_PRINTF2("fw_packet.actual_length: %d\r\n", fw_packet.actual_length);
    DEBUG_PRINTF2("fw_packet.read_offset: %d\r\n", fw_packet.read_offset);

    return 0;
}


int firmware_bin_file_fd = -1;

int check_firmware_file_exist(void)
{
    int ret;
#if 0    
    /*no lock file*/
    if(-1 != access(FIRMWARE_BIN_FILE, F_OK))
    {
        return -1;
    }
#endif 

    ret = access((char *)FIRMWARE_BIN_FILE, F_OK);
    
    return ret; 
}


int remove_firmware_file(void)
{
    if( 0 == remove((char *)FIRMWARE_BIN_FILE))
    {
         DEBUG_PRINTF("Delete firmware file success.\n");
         //DEBUG_PRINTF("Fake: Delete firmware file success.\n");
    }
    else
    {
        DEBUG_PRINTF("Delete firmware file failed.\n");
    }
    return 0;
}


int init_firmware_upgrade(void)
{   
    int fd;
    unsigned long file_size;
    
    if(-1 == (fd = open((char *)FIRMWARE_BIN_FILE, O_RDONLY)) )
    {   
        DEBUG_PRINTF("Open %s Error\n", FIRMWARE_BIN_FILE);   
        return fd;   
    }  

    firmware_bin_file_fd = fd;

    file_size= lseek(fd, 0L, SEEK_END);  
    lseek(fd, 0L, SEEK_SET); 

    //test
    uint32_t  total_crc = 0;
    unsigned long cnt = file_size;
    unsigned char temp = 0;

    while(cnt)
    {
        if(1 == read(fd, &temp, 1))
        {
            total_crc += temp;
            cnt--;
        }
    }
    DEBUG_PRINTF_L2("the total file crc is %d.\n", total_crc);
    lseek(fd, 0L, SEEK_SET); 

    init_firmware_upgrade_params(file_size);

    return fd;
}



int update_firmware_upgrade_params(unsigned short require_packet)
{
    if( (0 == require_packet) || (require_packet > fw_packet.total_packets) )
    {
        return -1;
    }

    fw_packet.current_packet = require_packet;


    /*this has problem*/
    //fw_packet.actual_length =  (fw_packet.current_packet >= fw_packet.total_packets) ? \
    //        (fw_packet.firmware_size % fw_packet.packet_length) : fw_packet.packet_length;
    //fw_packet.total_packets = (fw_packet.firmware_size + fw_packet.packet_length - 1) / fw_packet.packet_length;

    uint16_t actual_temp = (fw_packet.firmware_size % fw_packet.packet_length) ? \
        (fw_packet.firmware_size % fw_packet.packet_length) : fw_packet.packet_length;

    fw_packet.actual_length =  (fw_packet.current_packet < fw_packet.total_packets) ? \
             fw_packet.packet_length : actual_temp;

    fw_packet.read_offset = fw_packet.packet_length * (fw_packet.current_packet - 1);

    //test
    DEBUG_PRINTF2_L2("fw_packet.current_packet: %d, ", fw_packet.current_packet);
    DEBUG_PRINTF2_L2("total_packets: %d, ",  fw_packet.total_packets);
    DEBUG_PRINTF2_L2("packet_length: %d, ", fw_packet.packet_length);
    DEBUG_PRINTF2_L2("actual_length: %d, ", fw_packet.actual_length);
    DEBUG_PRINTF2_L2("read_offset: %d.\r\n", fw_packet.read_offset);

    return 0;
}


int get_firmware_packet_data(unsigned char *dest, uint32_t file_offset, uint16_t length)
{
    int ret = 0;

    if(NULL == dest)
    {
        return -1;
    }
    
    
    while(length)
    {
       lseek(firmware_bin_file_fd, file_offset, SEEK_SET); 
       ret = read(firmware_bin_file_fd, dest, length);

       ret = (ret >= 0) ? ret : 0;
       
       dest = dest - ret;
       length = length - ret;
    }

    return 0;
}


void set_upd_info(update_info_type *info, unsigned char target_id)
{
    info->dest = target_id;
    info->src = MODULE_SELF;
}

const unsigned char run_area_string[][10] =
{
    {"Boot"},
    {"App"},
    {"App2"},
    {"Reserved"},
    {"Reserved"},
};

const unsigned char target_status_string[][48]=
{
    {"Firmware Normal"},
    {"Firmware Pre update"},
    {"Firmware Updating"},
    {"Firmware Updated"},
    {"Reserved"},
    {"Reserved"},
    {"Reserved"},
};

int checkMoveControlAppVersion(char* version)
{
    char *p;
    p = strchr(version,'V');
    int first_version = atoi(p+1);
    p = strchr(p+1,'.');
    int second_version = atoi(p+1);
    p = strchr(p+1,'.');
    int third_version = atoi(p+1);
    
    if(first_version > MOVE_CONTROL_VERSIOM_FIRST_MIN)
        return 1;
    else if(first_version == MOVE_CONTROL_VERSIOM_FIRST_MIN && second_version > MOVE_CONTROL_VERSIOM_SECOND_MIN)
        return 1;
    else if(first_version == MOVE_CONTROL_VERSIOM_FIRST_MIN && second_version == MOVE_CONTROL_VERSIOM_SECOND_MIN && third_version > MOVE_CONTROL_VERSIOM_THIRD_MIN)
        return 1;
    return -1;
}

void local_module_update_cmd_process(serial_frame_type *frame)
{
    update_info_type *update_info_recv = (update_info_type *)(&(frame->data));

    /*process cmd data*/
    switch(frame->cmd)
    {
        case UPD_CMD_212:
        {
            clear_cmd_overtime_clock(UPD_CMD_212);
            clear_resend_cmd(UPD_CMD_212);
            upd_cmd_212_type *upd_cmd_212 = (upd_cmd_212_type *)&(frame->data.upd_cmd_212);
            //if(upd_cmd_212->upd_info.dest == main_update_option){
            if(upd_cmd_212->update_node == module_param_table[main_update_option].module_id){
                printf("the main module id respond is right! go to update!\n");
                manage_update_state = INIT_UPDATE_LIST;
                write_upgrade_flag(FIRMWARE_NORMAL);
            }else{
                printf("the main module id respond is wrong! abort update!\n");
                set_report_infomation(0, firmware_upgrade_flag, " the main module id respond is wrong! abort update!");                
                write_upgrade_flag(FIRMWARE_ABORT_UPGRADE);
                manage_update_state = RECORD_STATUS;                  

            }
        }
            break;
        case UPD_CMD_210:
        {
        
            upd_cmd_210_type *upd_cmd_210 = (upd_cmd_210_type *)&(frame->data.upd_cmd_210);
            // printf("upd_cmd_210->upd_info.src: %d\n",upd_cmd_210->upd_info.src);
            // printf("module_param_table[option].module_id: %d\n",module_param_table[update_option].module_id);
            // if(upd_cmd_210->upd_info.src != module_param_table[update_option].module_id){
            //     break;
            // }
            
            clear_cmd_overtime_clock(UPD_CMD_209);
            clear_resend_cmd(UPD_CMD_209);

            printf("*************************GET Firmware Version Info****************************\n");
            
            if( upd_cmd_210->run_area < GET_ARRAY_ELE_NUM(run_area_string) )
            {
                printf("run_area: %s\n", run_area_string[upd_cmd_210->run_area]);
            }
            else
            {
                printf("run_area: %d\n", upd_cmd_210->run_area);
            }

            if( upd_cmd_210->update_status < GET_ARRAY_ELE_NUM(target_status_string) )
            {
                printf("update_status: %s\n", target_status_string[upd_cmd_210->update_status]);
            }
            else
            {
                printf("update_status: %d\n", upd_cmd_210->update_status);
            }
            
            printf("boot_ver: %s\n", upd_cmd_210->boot_ver);
            printf("app_ver: %s\n", upd_cmd_210->app_ver);
            printf("update_lib_ver: %s\n", upd_cmd_210->update_lib_ver);
            printf("*************************GET Firmware Version Info****************************\n");

            if(read_upgrade_flag() == FIRMWARE_WAIT_CHECK_BEFORE_VERSION)
            {
              set_report_infomation_ex((char *)upd_cmd_210->app_ver, NULL);

              write_upgrade_flag(FIRMWARE_CHECK_VERSION_OK);
              //write_upgrade_flag(FIRMWARE_ABORT_UPGRADE);
            }

            if(read_upgrade_flag() == FIRMWARE_WAIT_CHECK_AFTER_VERSION)
            {
                set_report_infomation_ex(NULL, (char *)upd_cmd_210->app_ver);
                
                DEBUG_PRINTF("**********Firmware upgrade complete, return to normal.**********\n");
                write_upgrade_flag(FIRMWARE_DELETE_UPGRADE_FILE);
            }


            if(read_upgrade_flag() == FIRMWARE_WAIT_CHECK_MOVE_CONTROL_VERSION)
            {
                //升级前获取运控版本成功,进行版本判断
                DEBUG_PRINTF("********** Get move control app ver successed **********\n");
                printf("******************* Compare the move control app version ******************\n");
                printf("move control version get: %s\n", upd_cmd_210->app_ver);
                printf("move control verison min: %d.%d.%d\n",MOVE_CONTROL_VERSIOM_FIRST_MIN,MOVE_CONTROL_VERSIOM_SECOND_MIN,MOVE_CONTROL_VERSIOM_THIRD_MIN);
                char version_c[MAX_VERSION_STR];
                memcpy(version_c,upd_cmd_210->app_ver,MAX_VERSION_STR);
                int check_result = checkMoveControlAppVersion(version_c);
                if(-1 == check_result){
                    if(module_param_table[update_option].module_id == MODULE_MOVE_CONTROL_BOARD){
                        printf("the the move control verison is not supported! but the move control board update is allowed!\n");
                        write_upgrade_flag(FIRMWARE_NORMAL);
                        manage_update_state = INIT_UPDATE_LIST;
                    }else{
                        set_report_infomation(0, firmware_upgrade_flag, "the move control verison is not supported! pleaese update move control firstly!");                
                        write_upgrade_flag(FIRMWARE_ABORT_UPGRADE);
                        manage_update_state = RECORD_STATUS;                  
                    }
                }else{
                    printf("******************* mov control app version is higher than limit , go to update ******************\n");
                    write_upgrade_flag(FIRMWARE_WAIT_MAIN_MODULE_ID_RESPOND);
                    manage_update_state = SEND_MAIN_MODULE_ID;
                }
            }
        }
        break;
    
        /*app, to notify a firmware upgrade available.*/
        case UPD_CMD_200:
        {
			//clear_cmd_overtime_clock(UPD_CMD_200);
            //clear_resend_cmd(UPD_CMD_200);
			
            if( (read_upgrade_flag() != FIRMWARE_PRE_UPDATE) && (read_upgrade_flag() != FIRMWARE_WAIT_START_UPDATE) )
            {
                DEBUG_PRINTF("Receive UPD_CMD_200 with wrong state [%d].\n", read_upgrade_flag());
                set_report_infomation(0, firmware_upgrade_flag, "Receive cmd_200 in wrong state.");
                return;
            }

            upd_cmd_200_type *upd_cmd_200 = (upd_cmd_200_type *)&(frame->data.upd_cmd_200);

            if(1 == upd_cmd_200->status)
            {
                DEBUG_PRINTF("**********Receive acceptment to firmware upgrade. Change state to FIRMWARE_WAIT_START_UPDATE.**********\n");
                write_upgrade_flag(FIRMWARE_WAIT_START_UPDATE);
            }
            else
            {
                DEBUG_PRINTF("**********Receive decline to firmware upgrade! %d %d\n", upd_cmd_200->status, frame->data.upd_cmd_200.version[0]);
                set_report_infomation(0, firmware_upgrade_flag, "Receive decline to upgrade.");                
                write_upgrade_flag(FIRMWARE_ABORT_UPGRADE);
            }
        }

          break;

        

#if 0
        /*app, to authenticate upgrade right*/
        case UPD_CMD_201:
        {
            if(read_upgrade_flag() != FIRMWARE_ATHENTICATION)
            {
                return -1;
            }
            
            upd_cmd_201_type *upd_cmd_201 = (void *)&(frame->data.upd_cmd_201);

            if(1 == upd_cmd_201->status)
            {
                DEBUG_PRINTF("Receive athentication success to firmware upgrade.\n");
                write_upgrade_flag(FIRMWARE_WAIT_START_UPDATE);
            }
            else
            {
                DEBUG_PRINTF("******Receive athentication fail to firmware upgrade!\n");
                write_upgrade_flag(FIRMWARE_NORMAL);
            }

            
        }
            break;
#endif

        /*boot, stm32 to pi, to notify it can receive upgrade data now.*/
        case UPD_CMD_202:
        {
            //CMD_200 wait for reboot and send CMD_202
            clear_cmd_overtime_clock(UPD_CMD_200);
            clear_resend_cmd(UPD_CMD_200);
            
            if( (read_upgrade_flag() != FIRMWARE_WAIT_START_UPDATE) && (read_upgrade_flag() != FIRMWARE_PRE_UPDATE) )
            {

                DEBUG_PRINTF("Receive UPD_CMD_202 with wrong state [%d].\n", read_upgrade_flag());
                set_report_infomation(0, firmware_upgrade_flag, "Receive cmd_202 in wrong state.");
                return;
            }

            DEBUG_PRINTF("**********Receive cmd to begin upgrade.**********\n");
            write_upgrade_flag(FIRMWARE_UPDATING);

            /*send first package here*/
            //init_firmware_upgrade_params(48000);

            
            upd_cmd_203_type upd_cmd_203 = {0};

            set_upd_info(&upd_cmd_203.upd_info, update_target_id);
           
            upd_cmd_203.status = 0;  //notify upgrade data.
            upd_cmd_203.total_packets = fw_packet.total_packets;
            upd_cmd_203.current_packet = fw_packet.current_packet;
            upd_cmd_203.packet_length = fw_packet.packet_length;
            upd_cmd_203.actual_length = fw_packet.actual_length;

            /*read databuf from file here: fw_packet.read_offset, fw_packet.actual_length*/         
            /*test*/
            //memset(upd_cmd_203.data, 8, fw_packet.actual_length);

            get_firmware_packet_data(upd_cmd_203.data, fw_packet.read_offset, fw_packet.actual_length);

            //test
            int cnt = 0;
            while(cnt < fw_packet.actual_length)
            {
                    sp_total_crc += upd_cmd_203.data[cnt];
                    cnt++;
            }
            DEBUG_PRINTF("the upd_cmd_203's data sp_total_crc is %d.\n", sp_total_crc);
            
            construct_serial_frame(&serial_frame, UPD_CMD_203,  (void *)&upd_cmd_203);
            pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_203<<8) + MAX_CMD_RESEND_TIME);

            set_cmd_overtime_clock(UPD_CMD_203, MAX_CMD_OVERTIME_VALUE);
            
        }     
            break;

        /*boot, pi to stm32, receive upgrade data.*/
        //case UPD_CMD_203:
        //    break;
            
        /*boot, stm32 to pi, reply CMD_203 transfer results.*/
        case UPD_CMD_204:
        {
            clear_cmd_overtime_clock(UPD_CMD_203);
            clear_resend_cmd(UPD_CMD_203);

            clear_resend_param(frame->cmd);
        
            upd_cmd_203_type upd_cmd_203 = {0};
        
            DEBUG_PRINTF("Receive UPD_CMD_204.\n");
            if(read_upgrade_flag() != FIRMWARE_UPDATING)
            {
                DEBUG_PRINTF("Receive UPD_CMD_204 with wrong state [%d].\n", read_upgrade_flag());
                set_report_infomation(0, firmware_upgrade_flag, "Receive cmd_204 in wrong state.");
                return;
            }

            upd_cmd_204_type *upd_cmd_204 = (upd_cmd_204_type *)&(frame->data.upd_cmd_204);

            /*get last packet transfer results*/
            /*last packet transfer ok*/
            if(0 == upd_cmd_204->status)
            {
                 DEBUG_PRINTF("Receive UPD_CMD_204 ok status, current_packet:%d.\r\n", upd_cmd_204->current_packet);
                 
                if(upd_cmd_204->current_packet < fw_packet.total_packets)
                //if(cmd_204->current_packet < 1)  //for test
                {

                    DEBUG_PRINTF("Receive OK result for flashing packet[%d], send next packet.\n", upd_cmd_204->current_packet);
            
                    /*read file*/

                    /*pack and send new data*/
                    update_firmware_upgrade_params(upd_cmd_204->current_packet + 1);
                    
                    //cmd_203_type cmd_203 = {0};

                    set_upd_info(&upd_cmd_203.upd_info, update_target_id);
                   
                    upd_cmd_203.status = 0;  //notify upgrade data.
                    upd_cmd_203.total_packets = fw_packet.total_packets;
                    upd_cmd_203.current_packet = fw_packet.current_packet;
                    upd_cmd_203.packet_length = fw_packet.packet_length;
                    upd_cmd_203.actual_length = fw_packet.actual_length;

                    /*read databuf from file here: fw_packet.read_offset, fw_packet.actual_length*/
                    
                    /*test*/
                    //memset(cmd_203.data, 5, fw_packet.actual_length);
                    get_firmware_packet_data(upd_cmd_203.data, fw_packet.read_offset, fw_packet.actual_length);

                    //test
                    int cnt = 0;
                    while(cnt < fw_packet.actual_length)
                    {
                            sp_total_crc += upd_cmd_203.data[cnt];
                            cnt++;
                    }
                    DEBUG_PRINTF("the upd_cmd_203's data sp_total_crc is %d.\n", sp_total_crc);
                            
                    construct_serial_frame(&serial_frame, UPD_CMD_203, (void *)&upd_cmd_203);
                    pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_203<<8) + MAX_CMD_RESEND_TIME);

                    set_cmd_overtime_clock(UPD_CMD_203, MAX_CMD_OVERTIME_VALUE);
            
                }
                else
                {
                     
                    /*when =, this means the last packet is responsed, send reboot command*/                
                    DEBUG_PRINTF("**********Transfer upgrade data complete, send reboot command.**********\n");

                    upd_cmd_205_type upd_cmd_205 = {0};

                    set_upd_info(&upd_cmd_205.upd_info, update_target_id);
                
                    upd_cmd_205.status = 0;  //notice to reboot.
                
                    construct_serial_frame(&serial_frame, UPD_CMD_205, (void *)&upd_cmd_205);
                    pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_205 << 8) + MAX_CMD_RESEND_TIME);

                    set_cmd_overtime_clock(UPD_CMD_205, MAX_CMD_OVERTIME_VALUE * 3);
                
                    write_upgrade_flag(FIRMWARE_UPDATED_REBOOT);
                    DEBUG_PRINTF("In state FIRMWARE_UPDATED_REBOOT send upd_cmd_205.\n");
                
                }

            }

            /*mismatch packet order, but cannot deal the unresponse transmit*/
            else if(1 == upd_cmd_204->status )
            {
                /*shold not happen, this is not the deal.*/
                DEBUG_PRINTF("Receive mismatch packet order[%d], resend the correct one!!!\n", upd_cmd_204->current_packet);
        
                /*read file*/

                /*pack and send new data, usually is the lastone*/
                update_firmware_upgrade_params(upd_cmd_204->current_packet);
                
                //upd_cmd_203_type upd_cmd_203 = {0};

                set_upd_info(&upd_cmd_203.upd_info, update_target_id);
               
                upd_cmd_203.status = 0;  //notify upgrade data.
                upd_cmd_203.total_packets = fw_packet.total_packets;
                upd_cmd_203.current_packet = fw_packet.current_packet;
                upd_cmd_203.packet_length = fw_packet.packet_length;
                upd_cmd_203.actual_length = fw_packet.actual_length;

                /*read databuf from file here: fw_packet.read_offset, fw_packet.actual_length*/
                
                /*test*/
                //memset(upd_cmd_203.data, 5, fw_packet.actual_length);
                get_firmware_packet_data(upd_cmd_203.data, fw_packet.read_offset, fw_packet.actual_length);

                //test
                DEBUG_PRINTF("the upd_cmd_203's data sp_total_crc is %d.\n", sp_total_crc);
                        
                construct_serial_frame(&serial_frame, UPD_CMD_203, (void *)&upd_cmd_203);
                pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_203<<8) + MAX_CMD_RESEND_TIME);

                set_cmd_overtime_clock(UPD_CMD_203, MAX_CMD_OVERTIME_VALUE);
            
            }

            /*packet data length not ok*/
            else if(2 == upd_cmd_204->status)
            {
                /*shold not happen, this is not the deal.*/
                DEBUG_PRINTF("Receive mismatch packet length, correct at design time!!!\n");
                set_report_infomation(0, firmware_upgrade_flag, "Receive mismatch packet length.");
                
                write_upgrade_flag(FIRMWARE_ABORT_UPGRADE);
            }

            /*write flash CRC failed*/
            else if(3 == upd_cmd_204->status )
            {
                /*error, upgrade failed*/
                DEBUG_PRINTF("Receive writing flash crc failed, abandom firmware upgrade!!!\n");
                set_report_infomation(0, firmware_upgrade_flag, "write flash crc failed.");
                
                write_upgrade_flag(FIRMWARE_ABORT_UPGRADE);
            }

            /*unkown error*/
            else  
            {
               
               DEBUG_PRINTF("Receive UPD_CMD_204 unkown status!!!\n");

               DEBUG_PRINTF2("upd_cmd_204->status[%d], upd_cmd_204->current_packet[%d].\n", upd_cmd_204->status, upd_cmd_204->current_packet);
               /*resend current packet*/
               /*but you shouldn't receive a packet here!!! do elsewhere*/
               
               set_report_infomation(0, firmware_upgrade_flag, "Receive UPD_CMD_204 unkown status.");
               
               write_upgrade_flag(FIRMWARE_ABORT_UPGRADE);
            }     
            
        }
        
            break;

        /*boot, pi to stm32, notify to reboot*/
        case UPD_CMD_205:
            clear_resend_cmd(UPD_CMD_205);
            write_upgrade_flag(FIRMWARE_WAIT_FINISH_UPDATE);
            break;

        /*app, stm32 to pi, notify upgrade success.*/
        case UPD_CMD_206:
        {
            clear_cmd_overtime_clock(UPD_CMD_205);
            clear_resend_cmd(UPD_CMD_205);
            
            //wait for target module normal
            sleep(2);
        
            //check version after finished.
            upd_cmd_209_type upd_cmd_209 = {0};

            set_upd_info(&upd_cmd_209.upd_info, update_target_id);
       
            upd_cmd_209.check = 1;  //notice to check.
    
            construct_serial_frame(&serial_frame, UPD_CMD_209, (void *)&upd_cmd_209);
            pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_209 << 8) + MAX_CMD_RESEND_TIME);

            set_cmd_overtime_clock(UPD_CMD_209, MAX_CMD_OVERTIME_VALUE);
        }
            //DEBUG_PRINTF("**********Firmware upgrade complete, return to normal.**********\n");
            write_upgrade_flag(FIRMWARE_WAIT_CHECK_AFTER_VERSION);
            break;

        case UPD_CMD_211:
            clear_resend_cmd(UPD_CMD_211);
            break;

        default:
            return;
            
    }

    return;

}



void transparent_transmit_update_frame(serial_frame_type *frame, int port)
{
    if (-1 == port)
    {
        return;
    }

    int frame_length = frame->cmd_length + sizeof(frame_crc);
    frame_length = frame_length;  //avoid compiler bakes

    switch(port)
    {

        /********USER CODE BEGIN********/
        case PORT_USART1:
            //add data send code here.
            break;

        case PORT_UART2:
            //add data send code here.
            break;

        case PORT_USB:
            //add data send code here.
            break;

        case PORT_RJ45:
            //add data send code here.
            break;

        /********USER CODE END********/

        default:
            break;
    }
    
}


void process_update_cmd(serial_frame_type *frame)
{
    update_info_type *update_info = (update_info_type *)(&(frame->data));

    //update frame to self: receive and process
    if(update_info->dest == MODULE_SELF)
    {
        local_module_update_cmd_process(frame);
    }
    
    //update frame to other: check router table and send to dest port
    else
    {   int port;  
        port = get_cmd_router_port(update_info->dest);
        transparent_transmit_update_frame(frame, port);
    }
}


void process_normal_cmd(serial_frame_type *frame)
{
    unsigned char cmd;;
    unsigned int length;
    unsigned char *data; 

    if(NULL == frame)
    {
        return;
    }

    cmd = frame->cmd;
    length = frame->cmd_length - SERIAL_FIXED_HEADER_SIZE;
    data = (unsigned char *)&(frame->data);
		
		//avoid compiler warnning
		length = length;
		data = data;

    switch(cmd)
    {
        /********USER CODE BEGIN********/
        case CMD_NORMAL:
            
            //do routin work here.

            
            break;
        /********USER CODE END*********/

        default:
            break;
    }

    
}


/*process a serial frame of cmd*/
int process_cmd(serial_frame_type *frame)
{

    if(NULL == frame)
    {
        return -1;
    }

    /*process cmd data*/
    if(frame->cmd <= CMD_NORMAL_MAX)
    {
        process_normal_cmd(frame);
    }
    else
    {
        process_update_cmd(frame);
    }

    return 0;
}


void send_update_status_cmd(unsigned char status, unsigned char target_id)
{
    upd_cmd_211_type upd_cmd_211 = {0};

    set_upd_info(&upd_cmd_211.upd_info, target_id);
       
    upd_cmd_211.status = status;  //upgrade notice to module.
    
    construct_serial_frame(&serial_frame, UPD_CMD_211, (void *)&upd_cmd_211);
    pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_211 << 8) + MAX_CMD_RESEND_TIME);
}


void send_update_status_cmd_once(unsigned char status, unsigned char target_id)
{
    upd_cmd_211_type upd_cmd_211 = {0};

    set_upd_info(&upd_cmd_211.upd_info, target_id);
       
    upd_cmd_211.status = status;  //upgrade notice to module.

    construct_serial_frame(&serial_frame, UPD_CMD_211, (void *)&upd_cmd_211);
    pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_211 << 8) + 4);
}



