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
#include <sstream>
#include <ros/ros.h>
#include "cti_fpga_serial/updateinfo.h"



#include<stdio.h>  
#include<stdlib.h>  
#include<string.h> 

#include <unistd.h> //sleep(s)


#include "serial_cmd.hpp"

#include "serial_api.hpp"

#include "serial_api_al_linux.hpp"

#include "manage_updates.hpp"

//#include <pthread.h>


#define ROS_COMM_BUF_SIZE       2048

typedef struct
{
    unsigned char buf[ROS_COMM_BUF_SIZE];
    int start;
    int end;
}ros_comm_type;
ros_comm_type ros_recv = {0};

ros_comm_type ros_send = {0};


//Ê±Œäµœ²é¶©ÔÄ£¬ÓÐÔòµ÷ÓÃÉýŒ¶¿âµÄÈë¶ÓÁÐros_recv_data_call_backº¯Êý(µÃµœÊýŸÝºÍ³€¶È£¬µ÷ÓÃ¿ª·ÅµÄÈë¶ÓÁÐº¯Êý)
//Òª·¢²ŒÊ±¿âµ÷ÓÃros¿ª·ÅµÄÖ÷Ñ­»··¢ËÍº¯Êý£¬žÃº¯ÊýÖ±œÓ·¢ËÍÐÅÏ¢µœROSœÚµã
    


int get_ring_buf_available_size(ros_comm_type *ring)
{
    if(NULL == ring)
    {
        return 0;
    }

    int buf_size = sizeof(ring->buf);
    int actual_len  = (ring->end + buf_size - ring->start) % buf_size;
    
    return (buf_size - actual_len);
}

int get_ring_buf_used_size(ros_comm_type *ring)
{
    if(NULL == ring)
    {
        return 0;
    }

    int buf_size = sizeof(ring->buf);
    int used_len  = (ring->end + buf_size - ring->start) % buf_size;
    
    return used_len;
}



int ros_buffer_write(unsigned char *src, int cnt)
{
    if(NULL == src)
    {
        return 0;
    }

    int avail_len  = get_ring_buf_available_size(&ros_send);
    int write_len = cnt > avail_len ? avail_len : cnt;

    int i = 0;
    for(i = 0; i< write_len; i++)
    {
        ros_send.buf[ros_send.end] = src[i];
        ros_send.end = (ros_send.end + 1) % ROS_COMM_BUF_SIZE;
    }

    printf(">>>>>>ros_buffer_write size %d. ros_send start[%d], end[%d]\n", write_len, ros_send.start, ros_send.end);

    return write_len;
    
}



int ros_buffer_read(unsigned char *dest, int cnt)
{
    if(NULL == dest)
    {
        return 0;
    }

    int actual_len  = (ros_recv.end + ROS_COMM_BUF_SIZE - ros_recv.start) % ROS_COMM_BUF_SIZE;
    int recv_len = cnt > actual_len ? actual_len : cnt;

    if(0 == recv_len)
    {
        return 0;
    }

    int i;
    for(i = 0; i< recv_len; i++)
    {
        dest[i] = ros_recv.buf[ros_recv.start];
        ros_recv.start = (ros_recv.start + 1) % ROS_COMM_BUF_SIZE;
    }

    printf("<<<<<<ros_buffer_read size %d. ros_recv start[%d], end[%d]\n", recv_len, ros_recv.start, ros_recv.end);    

    return recv_len;
    
}


void ros_recv_data_call_back(const cti_fpga_serial::updateinfo &msg)
{

    //ÀŽÁËÊýŸÝ·ÅµœÈ«ŸÖ»º³åÖÐ×öºÃ±êŒÇ£¬ÈÃrecv_seial_flowÀŽÈ¡(Ñ­»·¶ÓÁÐ)¡£
    int recv_len = 0;

    recv_len = msg.data.size();
    
    int avail_len = get_ring_buf_available_size(&ros_recv);
    int write_len = recv_len > avail_len ? avail_len : recv_len;

    printf("recv a ros package: seq_num[%u], data_size[%u].\n", msg.seq_num, (unsigned int)msg.data.size());

    if(avail_len < write_len)
    {
       printf("ros recv buffer is not enough! discard[%d].\n", write_len - avail_len);
    }
    else
    {
        int i;
        for(i = 0; i < recv_len; i++)
        {
            ros_recv.buf[ros_recv.end] =  msg.data[i];
            ros_recv.end = (ros_recv.end + 1) % ROS_COMM_BUF_SIZE;
        }
    }


}



void *ros_update_comm_send(void * param)
{
    param = param;

    
}


void *ros_update_comm_recv(void * param)
{
    param = param;
}


int main(int argc, char* argv[])    // manage_update PORT MODULE_NAME [BINFILE_NAME]
{
    printf("**************GET Update Control APP Version Info******************\n");
    printf ("Control Version: %s\n",CONTROL_VERSION);
    printf("**************GET Update Control APP Version Info******************\n");
    printf("\n");
    printf("\n");
    sleep(1);

    for(int i = 0; i < argc;i++){
        printf("argv[%d]: %s\n",i,argv[i]);
    }

    if(5 == argc)
    {
        strncpy((char *)SERIAL_DEVICE_PORT, argv[1], DEVICE_PORT_STR_LENGTH - 1);
        specified_bin_file_name = (unsigned char *)argv[4];
    }
    else
    {
        printf("Input param error.\n");
        printf("--Usage:\n./rosrun firmware_update_manager firmware_update_manager_node  PORT_NAME  MAIN_MODULE_NAME  MODULE_NAME  [SPECIFIED_BIN_FILE_NAME]\n");
        printf("\tPORT_NAME\t    	the communication port. Now force to DEFAULT_PORT.\n");
        printf("\tMAIN_MODULE_NAME\t		main module name to upgrade.\n");
        for(int i = 0;i < MODULE_PARAM_TABLE_ELE_NUM; i++){
            printf("\t\t%-30s\t ",(char *)module_param_table[i].module_name);
            if((i+1)%3 == 0){
                printf("\n");
            }
        }
        printf("\n");
        printf("\tMODULE_NAME\t		a module name to upgrade.\n");
        for(int i = 0;i < MODULE_PARAM_TABLE_ELE_NUM; i++){
            printf("\t\t%-30s\t ",(char *)module_param_table[i].module_name);
            if((i+1)%3 == 0){
                printf("\n");
            }
        }
        printf("\n");
        printf("\tSPECIFIED_BIN_FILENAME	specify a bin filename to upgrade for a module.\n");
        printf("\tif not specified, default bin filename is \"MODULE_NAME.BIN\"\n");

        exit(-1);
    }
    //main_update_option is the key to search the main_module_param_table
    main_update_option = check_user_update_option((unsigned char *)argv[2]);
    // printf("main_update_option: %d\n",(int)main_update_option);
    if(main_update_option < 0)
    {
        printf("Input main module name param error.\r\n");
        exit(-1);
    }

    //update_option is the key to search the module_param_table
    update_option = check_user_update_option((unsigned char *)argv[3]);
    // printf("update_option: %d\n",(int)update_option);
    if(update_option < 0)
    {
        printf("Input module name param error.\r\n");
        exit(-1);
    }
    
    ros::init(argc, argv, "firmware_update_manager");
    ros::NodeHandle nh;
    ros::Publisher manager_pub = nh.advertise<cti_fpga_serial::updateinfo>("cti/fpga_serial/stmupdate", 20);
    ros::Subscriber manager_sub = nh.subscribe("cti/fpga_serial/stminfo", 5, ros_recv_data_call_back);
    ros::Rate loop_rate(200);

    module_serial_init();

    printf("Waiting for ros node sync, start in 3s...\n");
    sleep(1);
    printf("Start in 2s...\n");
    sleep(1);
    printf("Start in 1s...\n");
    sleep(1);


    uint32_t seq_num = 1;

    while(ros::ok())
    {
        manage_module_updates(&manage_update_state, update_option, main_update_option);
        module_serial_process();

        int send_buf_size = get_ring_buf_used_size(&ros_send);
        send_buf_size = send_buf_size > 1024 ? 1024 : send_buf_size;
        if(send_buf_size > 0)
        {
            cti_fpga_serial::updateinfo msg;
            msg.seq_num = seq_num++;
#if 0            
            msg.data_size = send_buf_size;

            int i = 0;
            for(i = 0; i < send_buf_size; i++)
            {
                msg.data[i] = ros_send.buf[ros_send.start];
                ros_send.start = (ros_send.start + 1) % ROS_COMM_BUF_SIZE;
            }
#else
            int i = 0;

            for(i = 0; i < send_buf_size; i++)
            {
                msg.data.push_back(ros_send.buf[ros_send.start]);
                ros_send.start = (ros_send.start + 1) % ROS_COMM_BUF_SIZE;
            }
#endif
 
            manager_pub.publish(msg); 
            printf(">>>>>>>>>>>firmware_update_manager published a msg, seq_num[%u], size[%u].\n", msg.seq_num, msg.data.size());

        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    /*wait thread exit.*/
    //pthread_join(thr_ros_recv, NULL);

    return 0;
}




