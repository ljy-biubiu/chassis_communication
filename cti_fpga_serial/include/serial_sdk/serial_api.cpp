/******************************************************************************

  Copyright (C),  2017, Candela Technology Innovation Co., Ltd.

 ******************************************************************************
  File Name     : serial_api.c
  Version       : Initial Draft
  Author        : Huangwei
  Created       : 2018/05/18
  Last Modified :
  Description   : This file offers the serial port data api.
  Function List :
  History       :
  1.Date        : 2018/05/18
    Author      : Huangwei
    Modification: Created file

******************************************************************************/
#include<stdio.h>  
#include<stdlib.h>   //perror printf

#include<string.h> 

#include "serial_sdk/serial_cmd.hpp"
#include "serial_sdk/crc16.hpp"
#include "serial_sdk/serial_api.hpp"
#include "serial_sdk/serial_api_al_linux.hpp"
//#include "serial_sdk/udp_api_al_linux.hpp"
#include <unistd.h>

/*----------------------------------------------*
 * external variables                           *
 *----------------------------------------------*/

/*----------------------------------------------*
 * external routine prototypes                  *
 *----------------------------------------------*/

/*----------------------------------------------*
 * internal routine prototypes                  *
 *----------------------------------------------*/

/*----------------------------------------------*
 * constants                                    *
 *----------------------------------------------*/

//important header + cmd + length size to specify a frame header struct
#define LOW_RECEIVE_BUFFER_THRESHOLD    10   

#define RECV_STATE_FINDING_HEADER       0
#define RECV_STATE_CHECK_FIXED_HEADER   1
#define RECV_STATE_RECEIVING_CRC        2

#define RECEIVE_RING_ADD(start, cnt)    ( (start + (cnt) )% RECEIVE_BUFFER_SIZE )
#define RECEIVE_RING_MINUS(start, cnt)  ( (start + RECEIVE_BUFFER_SIZE - (cnt)) % RECEIVE_BUFFER_SIZE )

/*----------------------------------------------*
 * project-wide global variables                *
 *----------------------------------------------*/


/*----------------------------------------------*
 * module-wide global variables                 *
 *----------------------------------------------*/
/*serial flow ring buf*/

serial_recv_buf_type serial_recv = {0}; 

serial_send_buf_type serial_snd = {0};

serial_frame_type serial_frame = {0};


serial_candidate_frame_type candidate = {0};

recv_frame_queue_type recv_frame_queue = {0};
send_frame_queue_type snd_frame_queue = {0};

int module_serial_fd = -1;


/*----------------------------------------------*
 * routines' implementations                    *
 *----------------------------------------------*/



void init_frame_queue(void)
{
    memset(&recv_frame_queue, 0, sizeof(recv_frame_queue));
    memset(&snd_frame_queue, 0, sizeof(snd_frame_queue));
}


void init_recv_buffer(void)
{
    serial_recv.start = 0;
    serial_recv.cnt = 0;
    serial_recv.state = RECV_STATE_FINDING_HEADER;
    memset(serial_recv.buf, 0, sizeof(serial_recv.buf));

    init_crc();
}


/*****************************************************************************
 Prototype    : query_serial_data
 Description  : read serial data and put data into buf.
 Input        : None
 Output       : the receive data structure serial_recv.
 Return Value : the size of read data. 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2018/05/18
    Author       : Huangwei
    Modification : Created function

*****************************************************************************/
int query_serial_data(int fd)
{
    int pos = 0;
    int remain_buffer_cnt = 0;
    int ret;

    remain_buffer_cnt = RECEIVE_BUFFER_SIZE - serial_recv.cnt;
    
    /*low available buffer level*/
    if(remain_buffer_cnt < LOW_RECEIVE_BUFFER_THRESHOLD)
    {
        DEBUG_PRINTF("Low recv buffer level!****cleaning recv buffer!\n\r");

        init_recv_buffer();      
        ret = 0;
        
        return ret;    
    }

    /*if near buffer end, just read some data to fill buffer tail.*/
    pos = RECEIVE_RING_ADD(serial_recv.start, serial_recv.cnt);
    

    if(pos >= serial_recv.start)
    {
        /*remain_buffer_cnt > (RECEIVE_BUFFER_SIZE - pos)*/
        ret = recv_serial_flow(fd, serial_recv.buf + pos,  ((RECEIVE_BUFFER_SIZE - pos) > SERIAL_FRAME_SIZE) ? \
                                SERIAL_FRAME_SIZE : (RECEIVE_BUFFER_SIZE - pos));
    }
    else
    {
        ret = recv_serial_flow(fd, serial_recv.buf + pos,  (remain_buffer_cnt > SERIAL_FRAME_SIZE) ? \
                                SERIAL_FRAME_SIZE : remain_buffer_cnt); 
    }

    if(ret > 0)
    {     
        serial_recv.cnt += ret;
    }

    //TEST
    if(ret > 0)
    {
        int i;
        DEBUG_PRINTF("Receive %d bytes data, recvpos(%d), data:[", ret, pos);
        for(i=0; i<ret; i++)
        {
            DEBUG_PRINTF2("%x ", serial_recv.buf[pos + i]);
        #if 0
            if(i == ret - 1)
            {
                DEBUG_PRINTF2_L2("/");
            }
            else
            {
                DEBUG_PRINTF2_L2(".");
            }
        #endif
        }
        DEBUG_PRINTF2("]\n");
    }

    return ret;
}


/*****************************************************************************
 Prototype    : find_frame_header
 Description  : find a frame header from received serial ring buffer.
 Input        : 
                pos     start position of received data for check.               
 Output       : 
                found   find a valid frame header.
 Return Value : the count size when finished finding. 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2018/05/18
    Author       : Huangwei
    Modification : Created function

*****************************************************************************/
int find_frame_header(int pos, char *found)
{
    int i, match_cnt;
   
    if( (pos < 0) || (pos >= RECEIVE_BUFFER_SIZE) )
    {
        DEBUG_PRINTF("Test point 1\n");
        return 0;
    }

    i = 0;
    while(i < (serial_recv.cnt - FRAME_HEADER_SIZE))
    {
        match_cnt = 0;
        while(match_cnt < FRAME_HEADER_SIZE)
        {
            
            if(serial_recv.buf[RECEIVE_RING_ADD(pos, i)] != serial_frame_header[match_cnt])
            {
              i++;
              break;;
            }
            else
            {
              match_cnt++;
              i++;
            }
        }

        if(FRAME_HEADER_SIZE == match_cnt)
        {
            *found = 1;
            break;
        }
    }
 
    return i;

}


/*****************************************************************************
 Prototype    : parse_data_from_recv
 Description  : parse a frame data to dest from receive buffer ring.
 Input        : unsigned char *dest: destination buffer.  
                unsigned char *src: source buffer ring.   
                int pos_start: start position of valid data to parse.       
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2018/05/18
    Author       : Huangwei
    Modification : Created function

*****************************************************************************/
void parse_data_from_recv(unsigned char *dest, int pos_start, int cnt)
{ 
    int copy1, copy2;
    
    if(NULL == dest)
    {
        return;
    }

    if( (pos_start < 0) || (pos_start >= RECEIVE_BUFFER_SIZE) || cnt > serial_recv.cnt)
    {
        DEBUG_PRINTF("parse param range error.\n");
        return;
    }

    /*parse data*/
    copy1 = ((RECEIVE_BUFFER_SIZE - pos_start) >= cnt) ? cnt : (RECEIVE_BUFFER_SIZE - pos_start);
    copy2 = cnt - copy1;
    //DEBUG_PRINTF("copy1:%d, copy2:%d. total:%d.\n", copy1, copy2, copy1 + copy2);

    memcpy(dest, serial_recv.buf + pos_start, copy1);

    if(copy2 > 0)
    {
        memcpy(dest + copy1, serial_recv.buf, copy2);
    }
        
}


/*****************************************************************************
 Prototype    : parse_recv_frame
 Description  : parse a frame from receive buffer ring to frame queue.   
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2018/05/18
    Author       : Huangwei
    Modification : Created function

*****************************************************************************/
int parse_recv_frame(void)
{
    unsigned char tail = (recv_frame_queue.head + recv_frame_queue.cnt) % MAX_RECV_FRAME_QUEUE;
    serial_frame_type *frame = &recv_frame_queue.frame[tail];

    if(recv_frame_queue.cnt >= MAX_RECV_FRAME_QUEUE)
    {
        return 0;
    }

    memset(frame, 0, sizeof(recv_frame_queue.frame[0]));
    memcpy(frame->header, candidate.header, sizeof(frame->header));
    frame->cmd = candidate.cmd;
    frame->cmd_length = candidate.cmd_length;

    /*here we make a mistake*/
    //parse_data_from_recv((unsigned char *)&frame->data, serial_recv.start, \
    //    frame->cmd_length - sizeof(frame->header) - sizeof(frame->cmd) - sizeof(frame->cmd_length));
    parse_data_from_recv((unsigned char *)&frame->data, (serial_recv.start + sizeof(serial_fixed_header_type))%RECEIVE_BUFFER_SIZE, \
    frame->cmd_length - sizeof(serial_fixed_header_type));  
    frame->crc16 = candidate.crc16;
    recv_frame_queue.cnt += 1;
    return 1;
}



/*****************************************************************************
 Prototype    : find_serial_frame
 Description  : find a valid frame in received buffer ring, copy to frame
                buffer.
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2018/05/18
    Author       : Huangwei
    Modification : Created function

*****************************************************************************/
void find_serial_frame(int fd)
{
    int ret, i;
    static int candidate_pos = 0;
    char found_header = 0;

    int length = 0;

    if(fd < 0)
    {
        return;
    }
    
    /*already flush recv param after query.*/
    ret = query_serial_data(fd);
    recv_data_cnt += ret;
    recv_data_cnt %= 65535;
    recv_ret = ret;

    if(ret <= 0)
    {
        //DEBUG_PRINTF("FAILED TP1.\n");
        return;
    }

    DEBUG_PRINTF_L2("Finding a frame from total %d buffer chars, new received %d chars, start pos:%d.\n", serial_recv.cnt, ret, serial_recv.start);

    //recv_pthread_cnt++;
    //recv_pthread_cnt %=65535;

    while(serial_recv.cnt)
    {
        switch(serial_recv.state)
        {
          case RECV_STATE_FINDING_HEADER:
              DEBUG_PRINTF_L2("In state RECV_STATE_FINDING_HEADER*********************\n");
              if(serial_recv.cnt <= FRAME_HEADER_SIZE)
              {
                  DEBUG_PRINTF_L2("Need more data to find header.\n");
                  return;
              }
              
              //char found_header = 0;
              found_header = 0;
              ret = find_frame_header(serial_recv.start, &found_header);
              
              if(found_header) //find header bytes
              {
                  /*hold to header*/
                  serial_recv.start = RECEIVE_RING_ADD(serial_recv.start, ret - FRAME_HEADER_SIZE);
                  serial_recv.cnt -= (ret - FRAME_HEADER_SIZE);

                  DEBUG_PRINTF("get candidate frame header. start_pos: %d, ret: %d\n", serial_recv.start, ret);

                  serial_recv.state = RECV_STATE_CHECK_FIXED_HEADER;
              }
              else
              {
                  DEBUG_PRINTF_L2("Doesnt find header in recv buffer.\n");
                  /*Not a valid header, abandon it.*/
                  serial_recv.start = RECEIVE_RING_ADD(serial_recv.start, ret);
                  serial_recv.cnt -= ret;
              }
              break;

          case RECV_STATE_CHECK_FIXED_HEADER:
              DEBUG_PRINTF_L2("In state RECV_STATE_CHECK_FIXED_HEADER*********************\n");
              if(serial_recv.cnt < sizeof(serial_fixed_header_type))
              {
                  DEBUG_PRINTF_L2("Need more data to find fixed header.\n");
                  return;
              }

              memset(&candidate, 0, sizeof(candidate));
              parse_data_from_recv((unsigned char *)&candidate, serial_recv.start, sizeof(serial_fixed_header_type));
#if 0
              /*do fix_header check here(cmd, cmd_length).*/
              length = get_cmd_data_length(candidate.cmd);

              if( (candidate.cmd > CMD_MAX) || (-1 == length) \
                || (candidate.cmd_length != (sizeof(serial_fixed_header_type) + length) ) ) 
#else
              //for ros module seperate
              length = MAX_CMD_DATA_SIZE;
              if(candidate.cmd_length > (sizeof(serial_fixed_header_type) + length) )

#endif
              {
                  DEBUG_PRINTF("Received cmd[%d] and cmd_length[%d] mismatched! Expect cmd_id<%d, length=%d(%d+%d). But accept when in code developing.\n", \
                        candidate.cmd, candidate.cmd_length, CMD_MAX, (int)(sizeof(serial_fixed_header_type) + length), \
                        (int)sizeof(serial_fixed_header_type), length);

                  /*not a valid header*/
                  serial_recv.start = RECEIVE_RING_ADD(serial_recv.start, FRAME_HEADER_SIZE);
                  serial_recv.cnt -= FRAME_HEADER_SIZE;
                  
                  serial_recv.state = RECV_STATE_FINDING_HEADER;
                  return;
              }
              else
              {
                  DEBUG_PRINTF("get cmd %d, cmd_length %d\n", candidate.cmd, candidate.cmd_length);
              }
              

              serial_recv.state = RECV_STATE_RECEIVING_CRC;
              break;

          case RECV_STATE_RECEIVING_CRC:
              DEBUG_PRINTF_L2("In state RECV_STATE_RECEIVING_CRC*********************\n");
              if(serial_recv.cnt < candidate.cmd_length + 2) //total frame
              {
                  DEBUG_PRINTF_L2("Need more data to find crc.\n");
                  return;
              }

              /*indicate to crc*/
              candidate_pos = RECEIVE_RING_ADD(serial_recv.start, candidate.cmd_length);
              parse_data_from_recv((unsigned char *)&candidate.crc16, candidate_pos, sizeof(candidate.crc16));

              /*calculate candidate frame crc*/
              init_crc();
              for(i = 0; i < candidate.cmd_length; i++)
              {
                  candidate_pos = RECEIVE_RING_ADD(serial_recv.start, i);
                  calc_crc(&serial_recv.buf[candidate_pos], 1);
              }

              if(frame_crc == candidate.crc16)
              {
                  //recv_pthread_crc_status = 0;
                  recv_crc_status = 0;
                  /*copy data to s_Msg*/
                  DEBUG_PRINTF("crc check ok, found a valid frame.***, pos:%d.\n", serial_recv.start);
                  if(!parse_recv_frame())
                  {
                      /*the frame queue is full*/
                      DEBUG_PRINTF("Recv frame queue high level detected, cnt:%d.\n", recv_frame_queue.cnt);
                  }
                  else
                  {
                      DEBUG_PRINTF("Get new frame in recv frame queue***, total cnt:%d.\n", recv_frame_queue.cnt);
                  }
                  
                  serial_recv.start = RECEIVE_RING_ADD(serial_recv.start, candidate.cmd_length + 2);
                  serial_recv.cnt -= (candidate.cmd_length + 2);
                  serial_recv.state = RECV_STATE_FINDING_HEADER;
              }
              else
              {
                  //recv_pthread_crc_status = 1;
                  recv_crc_status = -1;
                  DEBUG_PRINTF("bad crc check(recv:0x%x, calc:0x%x). Find a fake or damaged frame.\n", candidate.crc16, frame_crc);
                  serial_recv.start = RECEIVE_RING_ADD(serial_recv.start, FRAME_HEADER_SIZE);
                  serial_recv.cnt -= FRAME_HEADER_SIZE;
                  serial_recv.state = RECV_STATE_FINDING_HEADER;
              }
              break;

          default:
              /*wish nerver reach here.*/
              DEBUG_PRINTF("Oops!***, wrong recv state:%d.\n", serial_recv.state);
              serial_recv.state = RECV_STATE_FINDING_HEADER;
              break;
        }
    }
}


/*****************************************************************************
 Prototype    : pack_send_frame_buf
 Description  : translate a user defined frame data to a send buffer flow.
 Input        : User defined frame data  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2018/05/18
    Author       : Huangwei
    Modification : Created function

*****************************************************************************/
void pack_send_frame_buf(serial_frame_type *frame)
{
    unsigned char *pos= serial_snd.buf;
    
    if(NULL == frame)
    {
        return;
    }

    memset(&serial_snd, 0, sizeof(serial_snd));

    memcpy(pos, frame, frame->cmd_length);


    int i = 0;
#if 0
    for(i=0; i<(frame->cmd_length + sizeof(frame->crc16));i++)
    {
        printf("[%d] ", *((unsigned char *)frame + i));
    }
    printf("\n");
#endif

    init_crc();
    frame->crc16 = calc_crc(pos, frame->cmd_length);
    
    pos = pos + frame->cmd_length;
    memcpy(pos, (void *)&(frame->crc16), sizeof(frame->crc16));
    
    serial_snd.cnt = frame->cmd_length + sizeof(frame->crc16);

    //test
    DEBUG_PRINTF_L2("the following frame buf is packed:\n", );
    for(i=0; i<serial_snd.cnt;i++)
    {
        DEBUG_PRINTF2_L2("%d.", serial_snd.buf[i]);
    }
    DEBUG_PRINTF2_L2("\n");
   
    DEBUG_PRINTF("The being send frame buf calc_crc is 0x%x.\n", frame->crc16);
    
}


/*****************************************************************************
 Prototype    : pack_send_frame_buf_in)queue
 Description  : translate a user defined frame data to a send buffer flow queue.
 Input        : User defined frame data  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2018/05/18
    Author       : Huangwei
    Modification : Created function

*****************************************************************************/
int pack_send_frame_buf_in_queue(serial_frame_type *frame, unsigned short resend)
{
    serial_send_buf_type *snd_buf;
    
    if(NULL == frame)
    {
        return -1;
    }

    if(snd_frame_queue.cnt >= MAX_SEND_FRAME_QUEUE)
    {
        DEBUG_PRINTF("Send frame queue high level detected, cnt:%d.\n", snd_frame_queue.cnt);
        return -1;
    }

    /*set resend param*/
    snd_frame_queue.resend[(snd_frame_queue.head + snd_frame_queue.cnt) % MAX_SEND_FRAME_QUEUE] = resend;

    /*add buf to the queue tail.*/
    snd_buf= &(snd_frame_queue.frame[(snd_frame_queue.head + snd_frame_queue.cnt) % MAX_SEND_FRAME_QUEUE]);

    /*!!!we go a memory leak usage here, type mismatch!!!!*/
    //memset(snd_buf->buf, 0, sizeof(serial_send_buf_type));
    memset(snd_buf->buf, 0, SERIAL_FRAME_SIZE);

    memcpy(snd_buf->buf, frame, frame->cmd_length);

    //test
    int i = 0;
    DEBUG_PRINTF("the following frame buf is packed:\n");
    for(i=0; i<(frame->cmd_length + sizeof(frame->crc16));i++)
    {
        DEBUG_PRINTF2("%d.", *((unsigned char *)frame + i));
    }
    DEBUG_PRINTF2("\n");

    init_crc();
    frame->crc16 = calc_crc(snd_buf->buf, frame->cmd_length);
    memcpy(snd_buf->buf + frame->cmd_length, &frame_crc, sizeof(frame_crc));

    /*the size of current send buf*/
    snd_buf->cnt = frame->cmd_length + sizeof(frame->crc16);

    /*the size of send frame queue*/
    snd_frame_queue.cnt++;
   
    DEBUG_PRINTF2("the send buf calc crc is 0x%x.\n", frame->crc16);

    return 0;
    
}

int construct_serial_frame_ex(serial_frame_type * frame, unsigned char cmd, unsigned int length, void * data)
{
    if( (NULL == frame) || (NULL == data) )
    {
        DEBUG_PRINTF("NULL frame or data ptr when init serial frame!!!\n");
        return -1;
    }

    
    if(length + sizeof(serial_fixed_header_type) > sizeof(serial_frame_type)) 
    {
        DEBUG_PRINTF("***********Oops, we got a cmd_data_length[cmd] > sizeof(serial_frame.data) [%d vs %d], \
            correct the union_data_type!!!\n", length, (int)sizeof(serial_frame_type) - (int)sizeof(serial_fixed_header_type));
        return -1;
    }

    memset(frame, 0, sizeof(serial_frame_type));
    memcpy(frame->header, serial_frame_header, sizeof(frame->header));
    frame->cmd = cmd;
    frame->cmd_length = sizeof(serial_fixed_header_type) + length;

    /*or direct copy the send_frame_queue[n].buf?*/
    //memcpy((void *)&(serial_frame.data.upd_cmd_210), data, cmd_data_length[cmd]); //both is good.
    memcpy((void *)&(frame->data), data, length);

    return 0;
}



/*****************************************************************************
 Prototype    : send_serial_frame
 Description  : send out a serial frame.
 Input        : User defined frame data  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2018/05/18
    Author       : Huangwei
    Modification : Created function

*****************************************************************************/

int send_single_serial_frame(serial_frame_type *frame, unsigned char need_setting)
{
    int fd, ret;

    if(need_setting)
    {
        fd = open_serial_port();        
    }
    else
    {
        fd = module_serial_fd;
    }
    pack_send_frame_buf(frame);
    //--for test
    /*
    int i;
    printf("send:\n");

    for(i=0;i<serial_snd.cnt;i++){
        printf("%x ",serial_snd.buf[i]);
    }
    printf("\n");
    */
    ret = send_serial_flow(fd, serial_snd.buf, serial_snd.cnt);
    DEBUG_PRINTF("Send serial frame, total %d bytes.\n", ret);

    if(need_setting)
    {
        colse_serial_port(fd); 
    }
    return ret;
}

int send_update_serial_frame(const unsigned char *data, unsigned int len)
{
    int fd = -1, ret = -1;
    fd = module_serial_fd;
    if(fd >= 0){
        ret = send_serial_flow(fd, (unsigned char*)data, len);
    }
    return ret;
}

void process_received_serial_frames(void)
{

    //DEBUG_PRINTF("recv_frame_queue.cnt: %d.\n" recv_frame_queue.cnt);

    if(recv_frame_queue.cnt >=3)
    {
        DEBUG_PRINTF("High process_received_serial_frames level[%d] detected!!!\n", recv_frame_queue.cnt);
    } 

    while(recv_frame_queue.cnt > 0)
    {

        serial_frame_type *frame = &(recv_frame_queue.frame[recv_frame_queue.head]);
        //printf("recv frame with cmd:%d\n",frame->cmd);

        process_cmd(frame);

        recv_frame_queue.head = (recv_frame_queue.head + 1) % MAX_RECV_FRAME_QUEUE;
        recv_frame_queue.cnt--;
    }
}



enum
{
    NORMAL_SEND = 0,
    TRY_RESEND = 1,
    END_SEND = 2,

};



void clear_resend_param(unsigned char cmd)
{
    //if( ((snd_frame_queue.resend[snd_frame_queue.head]>> 8) & 0xff) == cmd)
    {
        snd_frame_queue.resend[snd_frame_queue.head] = 0;
        DEBUG_PRINTF("Catch response, set no resend.\n");
    }
}


#include <sys/time.h>

struct timeval tv;
//int64_t ts = (int64_t)tv.tv_sec*1000 + tv.tv_usec/1000;
int32_t ts_set;
int32_t ts_cur;


void send_serial_frames_if_necessary(void)
{
   if(snd_frame_queue.cnt >=4)
    {
        DEBUG_PRINTF("High snd_frame_queue level detected!!!\n");
    }

    while(snd_frame_queue.cnt > 0)
    //if(snd_frame_queue.cnt > 0)
    {
        
        serial_send_buf_type *frame = &(snd_frame_queue.frame[snd_frame_queue.head]);

        switch(snd_frame_queue.state)
        {
            case NORMAL_SEND: //send a frame;
                send_serial_flow(module_serial_fd, frame->buf, frame->cnt); 
                DEBUG_PRINTF("send out a frame buf.\n");

                if(snd_frame_queue.resend[snd_frame_queue.head] != 0)
                {
                    gettimeofday(&tv, NULL);
                    ts_set = tv.tv_sec*10 + tv.tv_usec/100000 + 10;  //unit per 100 ms, set 1s later
                    snd_frame_queue.state = TRY_RESEND;
                    DEBUG_PRINTF("Acorrding param, set resend timer.\n");
                    return;  //wait next time
                }
                else
                {
                    snd_frame_queue.state = END_SEND;
                    DEBUG_PRINTF("Acorrding param, set no resend. to complete.\n");
                }
                break;

            case TRY_RESEND:
                /*low byte stands for resend cnt*/
                if(snd_frame_queue.resend[snd_frame_queue.head] != 0)
                {
                    gettimeofday(&tv, NULL);
                    ts_cur = tv.tv_sec*10 + tv.tv_usec/100000;  //unit per 100 ms
                    if(ts_cur >= ts_set)  //timesup
                    {
                        send_serial_flow(module_serial_fd, frame->buf, frame->cnt); 
                        DEBUG_PRINTF("Resend out a frame buf.\n");

                        snd_frame_queue.resend[snd_frame_queue.head] = 0;

                        snd_frame_queue.state = END_SEND;  
                        break;
                    }
                    else
                    {
                        return; //wait next time
                    }
                    
                    //return;
                }

                /*the outer has received the response and clear the resend*/
                else
                {
                    snd_frame_queue.state = END_SEND;  
                    DEBUG_PRINTF("The proc get response, to complete.\n");
                }
            
                break;

            case END_SEND:
                DEBUG_PRINTF("a frame buf send out complete.\n");
                snd_frame_queue.resend[snd_frame_queue.head] = 0;
                
                snd_frame_queue.head = (snd_frame_queue.head + 1) % MAX_SEND_FRAME_QUEUE;
                snd_frame_queue.cnt--;
                snd_frame_queue.state = NORMAL_SEND;
                break;

            default:
                DEBUG_PRINTF("send state error!!!\n");
                snd_frame_queue.resend[snd_frame_queue.head] = 0;
                snd_frame_queue.state = NORMAL_SEND;
                break;    
        }
       
    }


}



void module_serial_init(void)
{
    init_frame_queue();
    init_recv_buffer();
    module_serial_fd = open_serial_port();
}



/*process_received_serial_frames() and find_serial_frame(fd) must in same thread,
  or the snd_frame_queue.cnt will conflict.*/
void module_serial_process(void)
{
    find_serial_frame(module_serial_fd);
    process_received_serial_frames();
    send_serial_frames_if_necessary();

}


void* serial_recv_thread(void* arg) 
{
    while(1)
    {
        find_serial_frame(module_serial_fd);
        usleep(2000);
    }
    return NULL;
}


void module_serial_process_thread(void)
{
    //find_serial_frame(module_serial_fd);
    process_received_serial_frames();
    send_serial_frames_if_necessary();

}



