
//#include "cti_fpga_serial/user_cmd.hpp"
/*linux header*/
#include<termios.h>     //struct termios
#include<stdio.h>       //delete file
#include<stdlib.h>      //perror printf


#include<unistd.h>  
#include<fcntl.h>       //open close read write
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/select.h>
#include <string.h>      //memset


/*common header*/
#include "serial_api_al_linux.hpp"
#include "serial_api.hpp"
#include "serial_cmd.hpp"
#include "crc16.hpp"


//linux header
#include <inttypes.h>    //or stdint.h uint16_t uint32_t

#include <sys/types.h> 
#include <unistd.h>  //file operation open read write lseek close
#include<fcntl.h> 
#include<linux/serial.h>


unsigned char SERIAL_DEVICE_PORT[DEVICE_PORT_STR_LENGTH] = {0};

/*the serial frame header definition.*/
const unsigned char serial_frame_header[FRAME_HEADER_SIZE] = {'C', 'T', 'R', 'L'};

fd_set fds;

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
	
	newtio.c_lflag &= ~ICANON;
  
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
    fd1 = open( (char *)SERIAL_DEVICE_PORT, O_RDWR | O_NOCTTY);// | O_NDELAY);
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
    while(-1 == (fd1 = do_open_serial()))
    {
        usleep(1000000UL);
    }

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
    close(fd);
    module_serial_fd = 0;
}


int send_serial_flow(int fd, unsigned char *src, int cnt)
{
    int ret = 0;
    int temp = cnt;
    while(temp)
    {
        ret = write(fd, src, temp);
        temp -= ret;
    }
    //test
    DEBUG_PRINTF("send out buf size %d.\n", cnt);
    return cnt;
}
/*
int recv_serial_flow(int fd, unsigned char *dest, int cnt)
{
    return read(fd, dest,  cnt);
}
*/
struct timeval timeout={0,100};
int recv_serial_flow(int fd, unsigned char *dest, int cnt)
{
    int ret=-1;
    FD_ZERO(&fds); 
    FD_SET(fd,&fds); 
    switch(select(fd+1,&fds,NULL,NULL,&timeout)) 
    {
    case -1: 
        printf("serial read error!\n"); 
        break; 
    case 0:
        break; 
    default:
        if(FD_ISSET(fd,&fds))
        {
            ret = read(fd, dest, cnt);
        }
    }
    return ret;
}

void local_module_update_cmd_process(serial_frame_type *frame)
{
    (void *)frame;
}

void transparent_transmit_update_frame(serial_frame_type *frame, int port)
{
    (void *)frame;
}

int (*process_update_cmd_cb)(unsigned char cmd, unsigned char *data, unsigned int length) = NULL;

void process_update_cmd(serial_frame_type *frame)
{
    unsigned char cmd;
    unsigned int length;
    unsigned char *data;

    if(NULL == frame)
    {
        return;
    }

    cmd = frame->cmd;
    length = frame->cmd_length + 2;
    data = (unsigned char *)frame;
    memcpy(data + frame->cmd_length, (uint8_t *)&frame->crc16, 2);

    //avoid compiler warnning
    length = length;
    data = data;

    if(process_update_cmd_cb != NULL);
    {
        process_update_cmd_cb(cmd, data, length);
    }
}


int (*process_nomal_cmd_cb)(unsigned char cmd, unsigned char *data, unsigned int length) = NULL;

void process_normal_cmd(serial_frame_type *frame)
{
    unsigned char cmd;
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

    if(process_nomal_cmd_cb != NULL);
    {
        process_nomal_cmd_cb(cmd, data, length);
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
        // if(frame->cmd == CMD_UPD_210 ) 
        // {
        //     unsigned char *data;
        //     data =(unsigned char *)&(frame->data);
        //     recv_from_firmware_version_type_1 *recv_version;
        //     recv_version=(recv_from_firmware_version_type_1*)data;
        //     printf("recv cmd 210, recv_version->upd_info.dest: %d\n",recv_version->upd_info.dest);
        //     if(recv_version->upd_info.dest == MODULE_CHECK_UPD_1)
        //     {
        //         serial_frame_type normal_frame;
        //         normal_frame = *frame;
        //         //printf("recv cmd 210, process as normal cmd.\n");
        //         process_normal_cmd(&normal_frame);
        //     }
        //     else
        //     {
        //         //printf("recv cmd 210, process as update cmd.\n");
        //         process_update_cmd(frame);
        //     }   
        // }   
        // else
        // {
             //206,212,210这几个命令都普通模式和升级模式都要处理
             if(frame->cmd == CMD_UPD_206  || frame->cmd == CMD_UPD_212 || frame->cmd == CMD_UPD_210)
             {
                serial_frame_type normal_frame;
                normal_frame = *frame;
                //printf("recv cmd 206, process as normal cmd.\n");
                process_normal_cmd(&normal_frame);
             }
             //printf("recv cmd %d\n, process as update cmd.\n",frame->cmd);
             process_update_cmd(frame);
        // }
        
    }
    return 0;
}





