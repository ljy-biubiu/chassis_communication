/*
 * This file is part of cti_fpga_data.
 * Author : wangpeng
 * Date: 2019-12-12
 * describe: .h file for cti_fpga_data.cpp
 */
#ifndef CTI_FPGA_DATA_H
#define CTI_FPGA_DATA_H

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <iostream>
#include <deque>
#include <string>
#include <limits>
#include <algorithm>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "cti_msgs/BatteryState.h"
#include "cti_msgs/TargetPose.h"
#include "cti_msgs/Range.h"
#include "cti_msgs/BatteryCell.h"
#include "cti_msgs/BatteryCellsState.h"
#include "cti_msgs/VehicleCtlRunInfo.h"
#include "cti_msgs/RobotVersionDisplay.h"
#include "cti_msgs/TabState.h"
#include "cti_msgs/BoxState.h"
#include "cti_msgs/Sins.h"
#include "cti_msgs/State.h"
#include <Eigen/Dense>
#include "ctilog/log.hpp"
#include "ctilog/loghelper.cpp.hpp"
#include "time.h"
#include <sensor_msgs/Imu.h>
#include <sstream>
#include <iomanip>
#include <vector>
#include <pthread.h>
#include <std_msgs/UInt16.h>
#include <cti_msgs/AutoTransmission.h>
#include <std_msgs/UInt8.h>
#include <cti_fpga_serial/navigationlog.h>
#include <std_msgs/UInt32MultiArray.h>
#include <cti_monitor/node_status_publisher.h>
#include <cti_fpga_serial/vehiclestate.h>
#include <cti_rblite_msgs/BoxAskResponse.h>
#include <cti_rblite_msgs/BoxInfo.h>
#include <std_msgs/UInt16MultiArray.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32MultiArray.h>
#include <deque>

#define cmin(x,y) ((x<y)?(x):(y))
#define MAX_FILTER_NUM 4  
constexpr char const* kN = "fpga-data";
#define PI 3.14159265358979323846
using namespace cti::log;
#define ult_vehicle 1
#define ult_dustbox 2

//---数据状态结构体
typedef struct {
    std::deque<uint16_t> data_buffer;
    float wight[MAX_FILTER_NUM];
    uint16_t data_output;
}ULT_DATA_ST;

class CtiFpgaData{
public:
    CtiFpgaData();
    void boxload_Callback(const cti_msgs::State::ConstPtr &msg);
    void rfidsingle_Callback(const cti_msgs::BoxState::ConstPtr &msg);
    void chassiserror_Callback(const std_msgs::UInt32MultiArray::ConstPtr &msg);
    void navigationlog_Callback(const cti_fpga_serial::navigationlog::ConstPtr &msg);
    void chassisInfo_Callback(const cti_fpga_serial::vehiclestate::ConstPtr &msg);
    void boxinfo_Callback(const cti_rblite_msgs::BoxAskResponse::ConstPtr &msg);
    void initLog(const std::string name);
    void lin_ult_data_Callback(const std_msgs::UInt16MultiArray::ConstPtr &msg);
    void dustbox_lin_ult_data_Callback(const std_msgs::UInt16MultiArray::ConstPtr &msg);
    void kalman_init();
    void kalman_filter(uint8_t ult_type,uint16_t ult_data[] , int len);

    //--ros param
    ros::Subscriber boxload_sub;
    ros::Subscriber rfid_sub;
    ros::Subscriber  chassis_error_sub;
    ros::Subscriber navigation_log_sub;
    ros::Publisher  rfid_pub;
    ros::Publisher  box_exist_pub;
    ros::Subscriber chassis_info_sub;
    ros::Subscriber box_info_sub;
    ros::Subscriber lin_ult_data_sub;
    ros::Subscriber dustbox_lin_ult_data_sub;
    
    //--rfid param
    std::vector<cti_msgs::TabState> tabstate_rfid1; 
    std::vector<cti_msgs::TabState> tabstate_rfid2;
    std::vector<cti_msgs::TabState> tabstate_rfid3;
    std::vector<cti_msgs::TabState> tabstate_rfid4;
    //--判断第2个位置RFID信息使用的变量
    std::string boxloadid_old ;//装箱消息id缓存
    int boxloadstatus_old ;//装箱消息状态缓存
    std::string rfid4th_old ;//最后一个位置的RFID读取信息缓存
    std::string rfid4thid_old ;//装箱命令状态缓存
    //--判断rfid接收超时
    double oldtime_rfid1 = 0;
    double oldtime_rfid2 = 0;
    double oldtime_rfid3 = 0;
    double oldtime_rfid4 = 0;
    bool recv_rfid1_timeout = false;
    bool recv_rfid2_timeout = false;
    bool recv_rfid3_timeout = false;
    bool recv_rfid4_timeout = false;
    //--rfid接收为空计数
    uint8_t rfid1_empty_cnt;
    uint8_t rfid2_empty_cnt;
    uint8_t rfid3_empty_cnt;
    uint8_t rfid4_empty_cnt;
    //--rfid read lock
    uint8_t rfid2_read_lock;

    //--消息定义
    cti_msgs::BoxState rfid_all;//一次发送车上所有rfid读卡器的信息
    std_msgs::UInt8 box_exist;
    //--底盘错误代码，错误模块枚举
    typedef enum module_error_enum{
        motion_control_board = 1,
        left_front_driver = 2,
        right_front_driver = 3,
        left_rear_driver = 4,
        right_rear_driver = 5,
        front_turn_driver = 6,
        rear_turn_driver = 7,
        front_brake_driver = 8,
        rear_brake_driver = 9,        
        battery_board = 10,
    }module_error_t;
    //错误级别
    typedef enum{
        warning_level = 1,
        serious_level = 2,
        deadly_level  = 3
    }chassis_error_level_enum;
    //lin通信超声波编号枚举
    typedef enum ult_type_enum{
        front_left_ult,
        front_right_ult,
        right_front_down_ult,
        right_front_up_ult,
        right_center_ult,
        rear_right_ult,
        rear_left_ult,
        left_center_ult,
        left_front_down_ult,
        left_front_up_ult,
        back_center_ult,
        unused,
        max_type_ult   
    }ult_type_t;

    //lin通信超声波名称
    std::string ult_name[max_type_ult]={
        "front_left_ult",
        "front_right_ult",
        "right_front_down_ult",
        "right_front_up_ult",
        "right_center_ult",
        "rear_right_ult",
        "rear_left_ult",
        "left_center_ult",
        "left_front_down_ult",
        "left_front_up_ult",
        "back_center_ult",
        "unused"
    };

    //箱子lin通信超声波编号枚举
    typedef enum dustbox_ult_type_enum{
        dustbox_rear_right_ult,
        dustbox_rear_left_ult,
        dustbox_bottom_ult,
        dustbox_max_type_ult   
    }dustbox_ult_type_t;

    
    double VEHICLE_WIDTH;   //车宽
    double REAR_ULT_DIST;   //后方超声波安装水平距离
    double FRONT_ULT_DIST;  //前方超声波安装水平距离
    double min_range;       //超声波有效数据最小值
    double max_range;       //超声波有效数据最大值
    double rear_ult_detect_offset;  //后方超声波探测的offset(用于调整对计算出来的点是否在车身以内的判断)
    double rear_ult_detect_max;     //后方超声波在进行计算之前进行的再次最大值限制，为了消除前面计算产生的漂移
    double rear_ult_detect_min;     //后方超声波在进行计算之前进行的再次最小值限制，为了消除前面计算产生的漂移
    double front_ult_detect_offset; //前方超声波探测的offset(用于调整对计算出来的点是否在车身以内的判断)
    double front_ult_detect_max;    //前方超声波在进行计算之前进行的再次最大值限制，为了消除前面计算产生的漂移
    double front_ult_detect_min;    //前方超声波在进行计算之前进行的再次最小值限制，为了消除前面计算产生的漂移
    ULT_DATA_ST ult_state[max_type_ult];  //储存超声波数据状态的结构体数据

    ros::Publisher lin_ult_pub[max_type_ult];  //单个超声波数据发布 发布器
    ros::Publisher lin_ult_front_center_pub;  //前中超声波数据发布
    ros::Publisher lin_ult_rear_center_pub;    //后中超声波数据发布
    ros::Publisher pub_all;                //所有超声波数据发布
    ros::Publisher pub_all_before_kalman; //发布卡尔曼滤波之前的所有车身超声波数据
    ros::Publisher pub_all_filtered;   //发布卡尔曼滤波之后的所有车身超声波数据
    ros::Publisher pub_0;  //发布超声波数据用于webtools显示
    ros::Publisher pub_1;  //发布超声波数据用于webtools显示
    ros::Publisher pub_2;  //发布超声波数据用于webtools显示
    ros::Publisher pub_3;  //发布超声波数据用于webtools显示
    ros::Publisher ult_topic_names_pub; //发布所有超声波的话题名称（用于给罗必鑫避障）
    ros::Timer timer_;  //定时发送超声波话题名称
    void timerCallback(const ros::TimerEvent& event);

    ros::Publisher dustbox_ult_rear_center_pub;  //箱子后中超声波数据发布
    ros::Publisher dustbox_ult_bottom_pub;        //箱子底部的超声波数据发布
    ros::Publisher pub_dustbox_all_before_kalman;  //箱子超声波 卡尔曼 滤波之前的数据
    ros::Publisher pub_dustbox_all_filtered;     //箱子超声波 卡尔曼 滤波之后的数据 
    ULT_DATA_ST dustbox_ult_state[dustbox_max_type_ult];  //箱子超声波数据数组

private:
    uint8_t RFIDTIMEOUT_DUR; 
    double RFID_EMPTY_LIMIT;
    int dustbox_LIN_ult_pub; //吸尘箱lin通信超声波发布
    int vehicle_LIN_ult_pub; //车身lin通信超声波发布
    int vehicle_rear_LIN_ult_pub; //车身尾部lin通信超声波发布
    //前中超声波发布前过滤
    int front_center_buffer_max;
    float front_center_pre;
    std::deque<float> front_center_buffer;
    //后中超声波发布前过滤
    double rear_ground_filter_dist;
    int rear_center_buffer_max;
    float rear_center_pre;
    std::deque<float> rear_center_buffer;
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    std::shared_ptr<cti_monitor::NodeStatusPublisher> node_status_publisher_ptr_;//节点状态检测
    double msg_max_range;//超声波消息中的max_range
    //超声波卡尔曼滤波结构体
    typedef struct Kalman
    {
        float lastP; //上次的协方差
        float nowP;  //本次的协方差
        float x_hat; //卡尔曼滤波的计算值，即后验最优值
        float Kg;    //卡尔曼最优系数
        float Q;     //过程噪声
        float R;     //测量噪声
    }Kalman;
    std::vector<Kalman> ult_Kalman_list_;//车身超声波卡尔曼滤波list
    std::vector<Kalman> dustbox_ult_Kalman_list_;//箱子超声波卡尔曼滤波list
    bool kalman_enable_; //卡尔曼滤波使能
    double kalman_R_;    //卡尔曼观测噪声
    double kalman_Q_;    //卡尔曼预测噪声，影响收敛速度
};

#endif
