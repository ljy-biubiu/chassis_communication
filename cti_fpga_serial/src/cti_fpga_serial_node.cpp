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
#include <geometry_msgs/TwistStamped.h>
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
#include "cti_msgs/Rtcm.h"
#include "cti_fpga_serial/user_cmd.hpp"
#include "cti_fpga_serial/updateinfo.h"
#include  "cti_fpga_serial/cmdanswer.h"
#include  "cti_fpga_serial/firmwareinfo.h"
#include  "cti_fpga_serial/ultModeSetState.h"
#include  "cti_fpga_serial/fwVerCheckState.h"
#include <Eigen/Dense>
#include "ctilog/log.hpp"
#include "ctilog/loghelper.cpp.hpp"
#include <sys/time.h>
#include <sensor_msgs/Imu.h>
#include <sstream>
#include <iomanip>
#include <vector>
#include <pthread.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <cti_msgs/AutoTransmission.h>
#include <cti_fpga_serial/navigationlog.h>
#include <std_msgs/UInt32MultiArray.h>
#include <cti_fpga_serial/serialstatus.h>
#include <cti_msgs/GnssRTK.h>
#include <std_msgs/Int32MultiArray.h>
#include <cti_msgs/RobotLocalizerState.h>
#include <cti_msgs/DustbinControl.h>
#include <cti_msgs/DustbinState.h>
#include <std_msgs/Int8.h>
#include <cti_monitor/node_status_publisher.h>
#include <cti_fpga_serial/vehiclestate.h>
#include <std_msgs/Int64.h>
#include "cti_fpga_serial/dustbinidstate.h"
#include "jsoncpp/json/json.h"
#include <cstring>
#include <std_msgs/Int16MultiArray.h>
#include <cti_msgs/DustbinControlNew.h>
#include <cti_msgs/DustbinStateNew.h>
#include <cti_msgs/DustboxControl.h>
#include <cti_msgs/DustboxState.h>
#include <cti_msgs/Data.h>
#include <cti_msgs/DataArray.h>
#include <cti_fpga_serial/rangeRawData.h>
#include <std_msgs/Float32MultiArray.h>
#include <queue>
#include <std_msgs/UInt16MultiArray.h>
#include <cti_fpga_serial/cleanFuncTestState.h>
#include <cti_fpga_serial/LoraIdSetState.h>
#include "cti_msgs/ErrorStatus.h"
#include <memory>

#define DEBUG_PRINT 0
static std::shared_ptr<cti_monitor::NodeStatusPublisher> node_status_publisher_ptr_;
//*********************************** 全局变量声明
//thread of reading data
pthread_t th_serial_recv;
//KN:name in log
constexpr char const* kN = "fpga-serial";
using namespace cti::log;
#define cmax(x,y) ((x>y)?(x):(y))
#define cmin(x,y) ((x<y)?(x):(y))
#define PI 3.14159265358979323846
#define GRAV  9.7803
#define SWEEPER_ULT_FILTER_NUM 2
serial_frame_type user_frame = {0};
std::string serial_port;
double car_wheel_base;
//--data buf
std::deque<serial_frame_include_id_type> databuf;
const unsigned int DATABUF_NUM_MAX = 5;
bool  stm32_update_flag = false;
unsigned int TIMEOUT = 0;
uint32_t seq_num = 0;
//--里程计
double odom_x = 0, odom_x_4wd = 0;
double odom_y = 0, odom_y_4wd = 0;
double odom_th = 0, odom_th_4wd = 0;
static nav_msgs::Odometry  odom, odom_4wd;
//--IMU
static cti_msgs::Sins sins_info;
static sensor_msgs::Imu imu_info;
//--灯控
uint16_t light_type=0;
//--路程
double mileage = 0;
//--电池电量
static cti_msgs::BatteryState batteryState;
static cti_msgs::BatteryCellsState BatCellsState;
//--版本信息
static cti_msgs::RobotVersionDisplay RobotVersionDisplay;
//--超声波数据
static cti_msgs::Range rangeDatas;
static sensor_msgs::Range rangeData;
//--运行信息
static cti_msgs::VehicleCtlRunInfo runinfo;
//--命令ID全局计数变量
uint16_t cmd_id_global = 0;
//--已发送命令的储存map，map的大小限制。
std::map<uint16_t, double> cmd_send_map;
const int SENDBUF_SIZE_MAXMUM = 1;
//--应答成功/失败的命令计数
uint32_t cmd_answer_success_cnt=0;
uint32_t cmd_answer_fail_cnt=0;
//--cmd send count
uint32_t cmd_withid_send_cnt = 0;
uint32_t cmd_withid_recv_cnt = 0;
//--消息应答情况
static cti_fpga_serial::cmdanswer cmd_answer_cnt;
//--timer3 计时器回调函数变量
uint8_t sendtime_loop_num = 50;
//--查询到版本标志位
bool get_version_flag = false;
//--顶升、锁箱、命令应答全局变量
int8_t global_up_down_flag = 0;
int8_t global_switch_flag = 0;
double cmd_answer_timeout = 0;
//position global temp
float global_pose[3] = {0};
float global_q[4] = {0};
//rfid 信息
cti_msgs::BoxState rfid_all;//一次发送车上所有rfid读卡器的信息
cti_msgs::BoxState rfid_single;//单个方式每一个raid的信息
std::vector<cti_msgs::TabState> oldtabstate_rfid1; 
std::vector<cti_msgs::TabState> oldtabstate_rfid2;
std::vector<cti_msgs::TabState> oldtabstate_rfid3;
std::vector<cti_msgs::TabState> oldtabstate_rfid4;
double oldtime_rfid1 = 0;
double oldtime_rfid2 = 0;
double oldtime_rfid3 = 0;
double oldtime_rfid4 = 0;
bool recv_rfid1_timeout = false;
bool recv_rfid2_timeout = false;
bool recv_rfid3_timeout = false;
bool recv_rfid4_timeout = false;
const double   RFIDTIMEOUT_DUR = 4;
//--通讯状态信息
uint8_t  recvrfid_loop_num = 0;
uint16_t recv_rfid_cnt = 1;
uint16_t recv_rfid_cnt_old = 0;

uint8_t  recvctrl_loop_num = 0;
uint16_t recv_ctrl_cnt = 1;
uint16_t recv_ctrl_cnt_old = 0;

uint8_t  recvctrl_rate_loop_num = 0;
uint16_t recv_ctrl_rate_cnt = 1;
uint16_t recv_ctrl_rate_cnt_old = 0;

uint8_t  recvult_loop_num = 0;
uint16_t recv_ult_cnt = 1;
uint16_t recv_ult_cnt_old = 0;

uint8_t  recvcmd_loop_num = 0;
uint16_t recv_cmd_cnt = 1;
uint16_t recv_cmd_cnt_old = 0;

uint8_t  recv_pthread_loop_num = 0;
uint16_t recv_pthread_cnt = 1;
uint16_t recv_pthread_cnt_old = 0;
uint16_t recv_pthread_crc_status = 0;// crc校验，0：正确 1：错误

const double   RECVTIMEOUT_DUR = 0.02;
const uint8_t RECV_CTRL_RATE_MIN = 15; //控制信息上传频率控制，少于15hz串口状态报错
//--通讯状态消息
static cti_fpga_serial::serialstatus serial_status;
uint16_t old_serial_status = 65535;
//-- 障碍物信息
int16_t global_k = 0; //前障碍物信息
int16_t global_k_back = 0; //后障碍物信息
//--激光对箱消息
static cti_msgs::Rtcm box_laser;
//--底盘导航log消息
static cti_fpga_serial::navigationlog navigation_log;
//--气压计数据储存全局变量
float baro_raw_old = 0;
//--气压计原始数据值检测消息
static std_msgs::UInt16 baro_status;
//--底盘错误码全局变量
uint8_t  module_type_global = 0;
uint32_t module_error_code_global = 0;
uint8_t  module_error_level_global = 0;
//--portIndex全局变量
uint8_t send_ctl_portIndex_cnt = 0;
uint8_t send_contraposition_portIndex_cnt = 0;
uint8_t send_poweroff_portIndex_cnt = 0;
uint8_t send_timenow_portIndex_cnt = 0;
//--sd卡格式化结果消息
static std_msgs::UInt8 formatsdcard_result;
//--gps消息
static cti_msgs::GnssRTK gps_data;
//--定位状态全局变量
static int32_t robotlocalizerstate_global = 0;
//--版本号全局变量
static int8_t control_version_right = 0; //运控版本兼容状态 0：运控版本可用; 1：运控版本过高; -1：运控版本过低;
static int max_control_board_version_head_ = 0;
static int min_control_board_version_head_ = 0;
static int max_control_board_version_mid_ = 0;
static int min_control_board_version_mid_ = 0;
static int max_control_board_version_end_ = 0;
static int min_control_board_version_end_ = 0;
//--ptp时间同步全局变量
bool need_resend_rough = false; 
time_sync_rough_msg_type sync_rough_msg_saved;
bool check_rough_sync_res_timeoout;
uint8_t check_rough_sync_res_cnt;
uint8_t send_fine_sync_cnt = 0;
bool need_send_rough = true;
bool need_send_fine_sync = false;
//--清扫箱控制变量
bool check_clean_mechine_state;
uint8_t recv_clean_mechine_motor_status;
motion_to_dustbin_t resend_to_dustbin_cmd;
//--清扫车超声波消息
static sensor_msgs::Range sweeperRangeData;
//--顶升测试标志位
bool lift_test_switch_ = false;
bool lift_test_flag=false;
uint16_t recv_lift_test_num = 0;
//--运控版本兼容状态消息
static std_msgs::Int8 firmwareVersionCheck; //
//--车上的箱子类型状态
int32_t box_type;
//--清扫箱箱号设置状态
dusbin_rf_set_state_t  dustbin_set_state;
//--清扫箱在车上设置标志位
bool set_dustbin_on_car = false; //false:未进行设置；true:设置中；
//--读取清扫箱id标志位
bool read_dustbin_id = false;
//--默认清扫箱箱号设置
int default_sweeper_id = 0;
//--清扫箱id设置接受
cti_fpga_serial::dustbinidstate dustbinidstate;
//--发送到集尘箱命令
motion_to_dust_box_t send_to_dust_box_cmd;
//--发送到清扫箱命令
motion_to_dustbin_t send_to_dustbin_cmd;
//--环卫车清扫命令
motion_to_clean_t send_to_clean_cmd;
motion_to_clean_t resend_to_clean_cmd;
//--机器人类型
int robot_type = 0;
//--机器人版本号
std::string cti_run_ver; 
//--默认的边刷伸展比例和速度..针对没有加这两个的旧结构体的兼容
int default_side_brush_transform;
int default_side_brush_speed;
//--是否安装的lin通信的超声波
int LIN_ult_installed;
//--dataarray类型控制话题的发送结构体
motion_to_clean_t_new send_to_clean_cmd_new;
//--存放水量计算的结构体
vehicle_water_box_status vehicle_water_status;
std::string config_file_path;
std::deque<uint8_t>  vehicle_water_tank_top;
std::deque<uint8_t>  vehicle_water_tank_bottom;
int max_vehicle_water_tank_restore = 6;
//--取消定位和障碍物对底盘速度限制的开关
bool localization_limit = true;
bool obstatle_disobs_limit = true;
//--通讯检测超时时间
double box_chat_timeout_secs;
double chassis_chat_timeout_secs;
chat_state chassis_chat_state;
chat_state box_chat_state;
int dustbox_lora_id = -1; // 清扫箱设定id号, -1:取消id

//--需要构建控制指令的顺序和lin通讯的超声波序号之间的对应表 在cpp文件中构建（-1,代表不存在, -2：不受指令控制）
int8_t linult_order_to_control[] = {2, -1, 1, -2, -1, 7, 6, -1, 10, 9, 4, 3, 8, 5};
//----------------------------------------;
lin_ult_mode_union vehicle_defalt_linult_mode;//2号前右，3号右前下，6号后右，9号左前下 箱子2号（后左）只收不发，其余即收也发
lin_ult_mode_union vehicle_all_stop_work_mode;//关闭所有超声波，有效位置为0,无效位全部为1
ultmode_set_state vehicle_ult_set_state;
uint64_t vehicle_ult_cmd = 0xffffffffffffffff; //接收到的超声波控制命令用于打开，关闭单个超声波
uint8_t vehicle_ult_check_resend_time = 0;
uint8_t vehicle_ult_set_resend_time = 0;


lin_ult_mode_union dustbox_defalt_linult_mode;//2号前右，3号右前下，6号后右，9号左前下 箱子2号（后左） 只收不发，其余即收也发
lin_ult_mode_union dustbox_all_stop_work_mode;//关闭所有超声波，有效位置为0,无效位全部为1
ultmode_set_state dustbox_ult_set_state;
uint64_t dustbox_ult_cmd = 0xffffffffffffffff; //接收到的超声波控制命令用于打开，关闭单个超声波
uint8_t dustbox_ult_check_resend_time = 0;
uint8_t dustbox_ult_set_resend_time = 0;
double msg_max_range;

//--是否在执行清扫功能测试的标志位
//--测试说明，由网页端下发测试参数和指令，此时测试标志位为true，不再接收导航下发的指令
//收到预期执行结果或者是执行超时，标志位置为false,继续接收导航下发的指令
clean_function_test_state_t clean_function_test_state;

int recv_data_cnt = 0;
int recv_ret = 0;
int recv_crc_status = 0;
//嵌入式版本查询状态结构体
fw_ver_check_state_t fw_ver_check_state;

//*********************************** ros发布器声明
ros::Publisher odom_pub;
ros::Publisher odom_pub_4wd;//里程计信息发布 
ros::Publisher odom_pub_calc;
ros::Publisher stm32_pub;
//ros::Publisher steer_pub; //2020年4月2日停用
ros::Publisher battery_pub;//电池信息发布
ros::Publisher batcell_pub;//单个电池信息发布
//ros::Publisher info_pub;//2020年4月2日停用
ros::Publisher ctlinfo_pub;
ros::Publisher state_pub;
ros::Publisher firmvion_pub;
ros::Publisher ranges_pub;
ros::Publisher range_pub[max_type_ult];
ros::Publisher sweeper_range_pub[sweeper_max_type_ult];
ros::Publisher new_sweeper_range_pub[SWEEPER_ULT_MODULE_NUM][new_sweeper_max_type_ult];
ros::Publisher dustbox_range_pub[dustbox_max_type_ult];
ros::Publisher raw_lin_range_pub;//lin通信超声波原始数据发布
ros::Publisher dustbox_bottom_range_pub;
ros::Publisher cmd_answer_pub;
ros::Publisher sins_pub;
ros::Publisher imudata_pub;
ros::Publisher boxrfid_pub_all;
ros::Publisher boxrfid_pub_single;
ros::Publisher serial_status_pub;
ros::Publisher box_laser_pub;//激光对箱信息发布
ros::Publisher chassis_error_pub;//底盘错误信息发布
ros::Publisher navigation_log_pub;//底盘重要信息上传
ros::Publisher baro_status_pub;//气压计状态发布
ros::Publisher formatsdcard_pub;//SD卡格式化结果回复
ros::Publisher gps_pub;//gps信息发布
ros::Publisher compass_pub;//compass信息发布
ros::Publisher node_status_pub;//节点状态标志话题
ros::Publisher dustbin_state_pub;//清扫箱状态发布
ros::Publisher dustbin_state_pub_new;//清扫箱状态发布,环卫车的清扫状态发布,新的消息类型
ros::Publisher firmware_version_status_pub;//运控版本兼容状态发布
ros::Publisher firmware_version_check_pub;//查询地盘固件版本信息的返回详细信息
ros::Publisher recv_chassis_info_pub;//收到运控上传所有信息发布
ros::Publisher set_dustbin_id_state_pub;//清扫箱id设置状态发布
ros::Publisher recv_dustbin_id_state_pub;//接受到清扫箱id信息发布
ros::Publisher dust_box_state_pub;//集尘箱状态发布
ros::Publisher dust_box_autopush_pub;//集尘箱自动倒垃圾发布
ros::Publisher dust_box_fanspeed_pub;//集尘箱风机速度发布
ros::Publisher dustbin_damboard_pub;//清扫箱挡板状态
ros::Publisher dustbox_wireless_charge_state_pub;//集尘箱无线充电状态
ros::Publisher boxlock_state_pub;//顶升磁吸锁状态
ros::Publisher dust_box_state_pub_json;//集尘箱状态发布json类型
ros::Publisher dust_box_state_pub_new; //集尘箱状态发布,新的消息类型
ros::Publisher rain_sensor_pub; //雨水传感器数据发布
ros::Publisher smart_trash_state_pub; //智能垃圾箱状态发布

ros::Publisher dust_box_state_info_pub; // 垃圾箱状态发布 cti_msgs/DataArray类型
ros::Publisher dust_vehicle_state_info_pub; // 环卫车清扫状态发布 cti_msgs/DataArray类型

ros::Publisher dustbox_5g_state_pub;//吸尘箱5g状态发布
ros::Publisher dustbox_batterycell_pub;//吸尘箱电池电量

ros::Publisher chat_statue_pub; //通讯状态发布(箱子和底盘)
ros::Publisher lin_ult_data_pub;//lin通信超声波原始数据发布
ros::Publisher dustbox_rear_range_pub;//吸尘箱超声波数据原始数据发布

ros::Publisher clean_function_test_state_pub;//清扫功能测试状态发布
ros::Publisher clean_function_test_report_pub;//清扫功能测试报告发布
ros::Publisher lora_id_set_state_pub;//lora_id 设置状态发布
ros::Publisher  wheel_speed_pub;//四轮速度发布
ros::Publisher error_publisher;//错误信息发布
ros::Publisher vehicle_ult_set_state_pub;//车身超声波模式设置状态发布
ros::Publisher dustbox_ult_set_state_pub;//吸尘箱超声波模式设置状态发布
ros::Publisher fw_check_state_pub;//嵌入式版本查询状态发布




//************************************** 初始函数
static void SigsHandler(int sig)
{
    ros::shutdown();
}
void init_signal()
{
    ::signal(SIGCHLD, SIG_IGN);// SIGCHLD
    ::signal(SIGINT, SigsHandler);// ^ + C
    ::signal(SIGTERM, SigsHandler);// 请求中断
    ::signal(SIGKILL, SigsHandler);// 强制中断
}
void initLog(const std::string name)
{
    //set the logger file name, defaut is "logger.log"
    Logger::setDefaultLogger(name);
    //set log output mode {CoutOrCerr,File,Both}
    Logger::getLogger().setOutputs(Logger::Output::File);
    //set log level mode {Fata,Erro,Warn,Note,Info,Trac,Debu,Deta}
    Logger::getLogger().setLogLevel(LogLevel::Info);
    //enable display thread id , defaut is "false"
    Logger::getLogger().enableTid(false);
    //enable display line id number, defaut is "true"
    Logger::getLogger().enableIdx(true);
    //1M  默认最小8*1024 默认最大sizeof(long)*32*1024*1024
    Logger::getLogger().setMaxSize(400*1024*1024);
}
//发布错误信息
void pubErrorMsg(std::string modular_name, uint error_code, uint error_level, std::string error_msg){
    cti_msgs::ErrorStatus error_msgs;
    error_msgs.stamp = ros::Time::now();
    error_msgs.module_name = modular_name;
    error_msgs.error_code = error_code;
    error_msgs.error_info = error_msg;
    error_msgs.level = error_level;
    error_publisher.publish(error_msgs);       
}
//************************************** 由四元数计算yaw
double calcYawFromQuaternion(const tf::Quaternion &q)
{
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}
//************************************** 把要发送的结构体放进缓冲区
void pushData(std::unique_ptr<serial_frame_include_id_type> data)
{
    if(data != NULL)
    {
        if(databuf.size() > DATABUF_NUM_MAX)
        {
            Info("CMD_DROP: "<<"id: "<< int(databuf.front().id) <<"cmd: "<<int(databuf.front().cmd));
            databuf.pop_front();
	    }
        //时间同步命令优先发送
        if(data->cmd == TIME_SYNC_FINE_CMD  || data->cmd == TIME_SYNC_DELAY_CMD)
        {
            databuf.push_front(*data);
        }
        else{
            databuf.push_back(*data);
        }
    }
}
//************************************** 发送的结构体构建
std::unique_ptr<serial_frame_include_id_type> construct_frame_include_id(uint16_t id, unsigned char cmd, int need_id, serial_frame_type frame){
    std::unique_ptr<serial_frame_include_id_type> ret(new serial_frame_include_id_type);
    ret->id = id;
    ret->cmd = cmd;
    ret->need_id = need_id;
    ret->frame = frame;
    return ret;
}

//************************************** 发送数据
void sendData(void)
{
    if(cmd_send_map.empty() && !databuf.empty())
    {
        if(databuf.front().need_id == 1)
        { 
            double starttime=ros::Time::now().toSec();
            uint16_t id=databuf.front().id;
            std::pair<std::map<uint16_t,double>::iterator,bool> ret;
            ret = cmd_send_map.insert(std::pair<uint16_t,double>(id,starttime));
            if (ret.second==false)
            {
                Info("CMD_EXITED: "<<"id: "<< int(ret.first->first));
            }
            cmd_withid_send_cnt++;    
        }
        send_single_serial_frame(&(databuf.front().frame), 0);
        //Info("cmd_send_id: "<< int(databuf.front().id)<<" cmd:"<<int(databuf.front().cmd));
       // printf("send id :%d\n",databuf.front().id);
        databuf.pop_front();
    }
}

//************************************** 16进制转string函数
std::string hex2string(uint8_t *data,unsigned int Len)
{
    int i=0;
    std::stringstream ss;
    for(i=0;i<Len;i++)
    {
        ss <<std::setw(2) << std::setfill('0') << std::hex << (int)data[i];
    }
    std::string str(ss.str()); 
    return str;
}

//************************************** 控制命令话题回调函数
void cmd_vel_callback(const geometry_msgs::TwistStamped::ConstPtr &twistStamped)
{
     node_status_publisher_ptr_->CHECK_RATE("/topic/cti_fpga_serial/cmd_vel",20,15,"topic /cmd_vel rate is too slow");
    if(stm32_update_flag){
        return;
    }
    if(control_version_right != 0)
    {
        return;
    }
    double msgs_time = twistStamped->header.stamp.toSec();
    double now_time = ros::Time::now().toSec();
    if ((now_time - msgs_time)>1)
    {
        return;
    }
    ros::Time start_time = ros::Time::now();
    send_to_control_cmd_type control_cmd;
    send_ctl_portIndex_cnt++;
    send_ctl_portIndex_cnt %= 255;
    control_cmd.portIndex = send_ctl_portIndex_cnt;
    control_cmd.linkCnt = 0;
    control_cmd.cmd_id=cmd_id_global++;
    control_cmd.linkFlag = 1;
    control_cmd.cmd_vel_Vx = twistStamped->twist.linear.x;
    node_status_publisher_ptr_->CHECK_MAX_VALUE("/value/cti_fpga_serial/control_cmd/vx",control_cmd.cmd_vel_Vx,4.5,5,"value control_cmd:vx is too high");
    control_cmd.cmd_vel_Vy = twistStamped->twist.angular.y;
    control_cmd.cmd_vel_W = twistStamped->twist.angular.z;
    control_cmd.cmd_turn_mode = abs(twistStamped->twist.angular.x);
    control_cmd.cmd_break_flag = abs(twistStamped->twist.linear.z);
    control_cmd.switch_flag = global_switch_flag;
    control_cmd.up_down_flag = global_up_down_flag;
    control_cmd.light_type = light_type;
    control_cmd.pose[0] = global_pose[0];
    control_cmd.pose[1] = global_pose[1];
    control_cmd.pose[2] = global_pose[2];
    control_cmd.q[0] = global_q[0];
    control_cmd.q[1] = global_q[1];
    control_cmd.q[2] = global_q[2];
    control_cmd.q[3] = global_q[3]; 
    control_cmd.imu_flag = 10;
    control_cmd.front_obstacle_distance = global_k;
    control_cmd.rear_obstacle_distance = global_k_back;
    if( 0 != robotlocalizerstate_global)
    {
        control_cmd.robot_status.bits.localizer = 1;
    }
    else
    {
        control_cmd.robot_status.bits.localizer = 0;
    }  
    construct_serial_frame_ex(&user_frame, SEND_TO_CONTROL_CMD, sizeof(control_cmd), &control_cmd);
    // serial_frame_include_id_type user_frame_include_id;
    // user_frame_include_id.id = control_cmd.cmd_id;
    // user_frame_include_id.cmd = SEND_TO_CONTROL_CMD;
    // user_frame_include_id.need_id = 1;
    // user_frame_include_id.frame = user_frame;
    pushData(construct_frame_include_id(control_cmd.cmd_id,SEND_TO_CONTROL_CMD,1,user_frame));

    serial_status.send_localizer =  control_cmd.robot_status.bits.localizer;

    Info("S_CT_PI: "<<control_cmd.cmd_id
    <<" S_CT_VX: "<<twistStamped->twist.linear.x
    <<" S_CT_VY: "<<twistStamped->twist.linear.y
    <<" S_CT_TA: "<<twistStamped->twist.angular.z
    <<" S_CT_TM: "<<abs(twistStamped->twist.angular.x)
    <<" S_CT_LT: "<<light_type
    <<" S_CT_K: "<<global_k
    <<" S_CT_KB: "<<global_k_back
    <<" S_CT_BK: "<<abs(twistStamped->twist.linear.z)
    <<" S_CT_LP: "<<(int)global_up_down_flag
    <<" S_CT_LL: "<<robotlocalizerstate_global
    <<" S_CT_BL: "<<(int)control_cmd.switch_flag
    );
    static int send_loop_cnt = 0;
    send_loop_cnt++;
    if(send_loop_cnt >= 20)
    {
        Info(" S_CT_PS: "<<global_pose[0]<<" "<<global_pose[1]<<" "<<global_pose[2]
        <<" S_CT_Q: "<<global_q[0]<<" "<<global_q[1]<<" "<<global_q[2]<<" "<<global_q[3]);
        send_loop_cnt = 0;
    }
}

//************************************** 对箱命令话题回调函数<已经弃用>
void boxContraposition_Callback(const cti_msgs::TargetPose::ConstPtr &msg)
{
    if(stm32_update_flag){
        return;
    }
    send_to_control_contraposition_type  control_contraposition_cmd;
    send_contraposition_portIndex_cnt++;
    send_contraposition_portIndex_cnt %= 255;
    control_contraposition_cmd.portIndex = send_contraposition_portIndex_cnt;
    double q_x = msg->pose.pose.orientation.x;
    double q_y = msg->pose.pose.orientation.y;
    double q_z = msg->pose.pose.orientation.z;
    double q_w = msg->pose.pose.orientation.w;
    tf::Quaternion q(q_x, q_y, q_z, q_w);
    double q_yaw = calcYawFromQuaternion(q);
    //---
    if(msg->command == cti_msgs::TargetPose::LOAD){ //装箱
        control_contraposition_cmd.dock_flag = 1;
    }else if(msg->command == cti_msgs::TargetPose::UNLOAD){  //卸箱
        control_contraposition_cmd.dock_flag = 2;
    }else if(msg->command == cti_msgs::TargetPose::LOAD+10){ //前方障碍
        control_contraposition_cmd.dock_flag = 3;
    }else{
        control_contraposition_cmd.dock_flag = 0;
    }
    
    control_contraposition_cmd.current_X = msg->pose.pose.position.x;
    control_contraposition_cmd.current_Y = msg->pose.pose.position.y;
    control_contraposition_cmd.current_Yaw = q_yaw;
    construct_serial_frame_ex(&user_frame, SEND_TO_CONTROL_CONTRAPOSITION, sizeof(control_contraposition_cmd), &control_contraposition_cmd);
    // serial_frame_include_id_type user_frame_include_id;
    // user_frame_include_id.id = 0;
    // user_frame_include_id.cmd = SEND_TO_CONTROL_CONTRAPOSITION;
    // user_frame_include_id.need_id = 0;
    // user_frame_include_id.frame = user_frame;
    // pushData(&user_frame_include_id);
    pushData(construct_frame_include_id(0,SEND_TO_CONTROL_CONTRAPOSITION,0,user_frame));
}

//************************************** 升降箱命令话题回调函数
void boxUpDown_Callback(const std_msgs::Int32::ConstPtr &msg)
{
    if(stm32_update_flag || lift_test_flag){
        return;
    }
    global_up_down_flag = msg->data;
    static int old_data=std::numeric_limits<int>::min();
    if(msg->data != old_data)
    {
        old_data = msg->data;
    }
}

//************************************** 顶升测试升降命令回调
void lifttest_Callback(const std_msgs::Int32::ConstPtr &msg)
{
    if(stm32_update_flag){
        return;
    }
    global_up_down_flag = msg->data;
    recv_lift_test_num  %= 5000;
    recv_lift_test_num++;
    lift_test_flag = true;
}
//************************************** 锁箱命令话题回调函数
void boxLock_Callback(const std_msgs::Int32::ConstPtr &msg)
{
    if(stm32_update_flag){
        return;
    }
    global_switch_flag = msg->data; //0：磁吸锁松开（销子弹出，箱子被锁住） 1：磁吸锁吸合（销子下降，箱子不被锁住）
    static int old_data=std::numeric_limits<int>::min();
    if(msg->data != old_data)
    {
        old_data = msg->data;
    }
}

//************************************** 掉电命令话题回调函数（转用为休眠指令）
void powerOff_Callback(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
    if(stm32_update_flag){
        return;
    }
    send_to_poweroff_cmd_type poweroff_cmd;
    poweroff_cmd.power_off_flag[0] = msg->data[0];
    poweroff_cmd.power_off_flag[1] = 0;//msg->data[1];
    poweroff_cmd.power_off_flag[2] = 0;//msg->data[2];
    poweroff_cmd.power_off_flag[3] = 0;//msg->data[3];
    poweroff_cmd.cmd_id=cmd_id_global++;
    construct_serial_frame_ex(&user_frame, SEND_TO_POWEROFF_CMD, sizeof(send_to_poweroff_cmd_type), &poweroff_cmd);
    // serial_frame_include_id_type user_frame_include_id;
    // user_frame_include_id.id = poweroff_cmd.cmd_id;
    // user_frame_include_id.cmd = SEND_TO_POWEROFF_CMD;
    // user_frame_include_id.frame = user_frame;
    // user_frame_include_id.need_id =1;
    // pushData(&user_frame_include_id);
    pushData(construct_frame_include_id(poweroff_cmd.cmd_id,SEND_TO_POWEROFF_CMD,1,user_frame));
    Info("S_PW: "<<msg->data[0]);
}

//************************************** 固件升级命令话题回调函数
void stmUpdate_Callback(const cti_fpga_serial::updateinfo::ConstPtr &msg)
{
    TIMEOUT = 0;//清空定时器
    if(msg->seq_num >=1){
        stm32_update_flag = true;
        send_update_serial_frame(msg->data.data(),msg->data.size());
    }else{
        stm32_update_flag = false;
        seq_num = 0;
    }
}
//************************************** 灯控命令话题回调函数
void lightType_Callback(const std_msgs::Int32::ConstPtr &msg)
{
    light_type = msg->data;
}

//************************************** 位置命令话题回调函数
void position_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    global_pose[0] = msg->pose.position.x;
    global_pose[1] = msg->pose.position.y;
    global_pose[2] = msg->pose.position.z;
    global_q[0] = msg->pose.orientation.w;   
    global_q[1] = msg->pose.orientation.x;   
    global_q[2] = msg->pose.orientation.y;   
    global_q[3] = msg->pose.orientation.z;
}

//************************************** 障碍物命令话题回调函数
void disObs_Callback(const cti_msgs::AutoTransmission::ConstPtr &msg)
{
    if(obstatle_disobs_limit){
        double msgs_time = msg->header.stamp.toSec();
        double now_time = ros::Time::now().toSec();
        if ((now_time - msgs_time)>1)
        {
            return;
        }
        global_k = (msg->k)*100; 
        global_k_back = (msg->k_back)*100;
    }else{
        global_k = 10.0*100;
        global_k_back = 10.0*100;
    }
}

//************************************** 查询固件版本号命令话题回调函数
void checkversion_Callback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
    if(stm32_update_flag){
        return;
    }
    if(fw_ver_check_state.state == CH_START_CHECK ||
    fw_ver_check_state.state == CH_WAIT_212_RESPOND ||
    fw_ver_check_state.state == CH_212_RESPOND_SUCCESS ||
    fw_ver_check_state.state == CH_WAIT_210_RESPOND){
        return;
    }
    fw_ver_check_state.reset();
    fw_ver_check_state.major_id_send = msg->data[0];
    fw_ver_check_state.minor_id_send = msg->data[1];
    fw_ver_check_state.state = CH_START_CHECK;
    
    // send_to_check_version_type check_version_cmd;
    // update_info_type update_info_struct;
    // update_info_struct.src = MODULE_CONTROL_PC;
    // update_info_struct.dest = msg->data;
    // check_version_cmd.upd_info = update_info_struct;
    // check_version_cmd.check = 01;
    // construct_serial_frame_ex(&user_frame, SEND_TO_CHECK_PROGRAM_VERSION, sizeof(check_version_cmd), &check_version_cmd);
    // // serial_frame_include_id_type user_frame_include_id_1;
    // // user_frame_include_id_1.id = 0;
    // // user_frame_include_id_1.cmd = SEND_TO_CHECK_PROGRAM_VERSION;
    // // user_frame_include_id_1.need_id = 0;
    // // user_frame_include_id_1.frame = user_frame;
    // // pushData(&user_frame_include_id_1);
    // pushData(construct_frame_include_id(0,SEND_TO_CHECK_PROGRAM_VERSION,0,user_frame));
    Info("S_CH_VE_MA: "<<(int)msg->data[0]
        <<"S_CH_VE_MI"<<(int)msg->data[1]);
}

//************************************** 格式化SD卡命令话题回调函数
void formatsdcard_Callback(const std_msgs::UInt8::ConstPtr &msg)
{
    if(stm32_update_flag){
        return;
    }
    //结构体实例化
    send_format_sd_card_cmd_type format_sdcard_cmd;
    //数据赋值
    format_sdcard_cmd.portIndex = msg->data;
    //数据打包
    construct_serial_frame_ex(&user_frame, SEND_FORMAT_SD_CARD_CMD, sizeof(format_sdcard_cmd), &format_sdcard_cmd);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_FORMAT_SD_CARD_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    //数据再次打包放入发送队列
    pushData(construct_frame_include_id(0,SEND_FORMAT_SD_CARD_CMD,0,user_frame));
    Info("S_FO_CD: "<<(int)msg->data);
}

//************************************** 清扫箱控制回调函数 
void dustbinControl_Callback(const cti_msgs::DustbinControl::ConstPtr &msg)
{
    if(stm32_update_flag){
        return;
    }
    send_to_dustbin_cmd.port = 0;
    send_to_dustbin_cmd.engin_start = msg->engin_start;
    send_to_dustbin_cmd.lift_motor= msg->lift_motor;
    send_to_dustbin_cmd.main_brush = msg->main_brush;
    send_to_dustbin_cmd.spray_motor = msg->spray_motor;
    send_to_dustbin_cmd.dust_suppresion= msg->dust_suppresion;
    send_to_dustbin_cmd.side_brush = msg->side_brush;
    send_to_dustbin_cmd.led= msg->led;
    send_to_dustbin_cmd.control_mode= 2;//2：导航控制
    if(4 == box_type){
        send_to_dustbin_cmd.dustbin_on_car = 1;//清扫箱在车上
    }else{
        send_to_dustbin_cmd.dustbin_on_car = 0;
    }
    
    construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD, sizeof(send_to_dustbin_cmd), &send_to_dustbin_cmd);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,SEND_TO_DUSTBIN_CMD,0,user_frame));

    Info("S_DBI_ES: "<<(int)msg->engin_start
    <<" S_DBI_LM: "<<(int)msg->lift_motor
    <<" S_DBI_MB: "<<(int)msg->main_brush
    <<" S_DBI_SM: "<<(int)msg->spray_motor
    <<" S_DBI_DS: "<<(int)msg->dust_suppresion
    <<" S_DBI_SB: "<<(int)msg->side_brush
    <<" S_DBI_LED: "<<(int)msg->led
    <<" S_DBI_DOC: "<<(int)send_to_dustbin_cmd.dustbin_on_car
    );
}

//************************************** 环卫车清扫控制回调函数
void cleanControl_Callback(const cti_msgs::DustbinControl::ConstPtr &msg)
{
    if(stm32_update_flag){
        return;
    }
    serial_status.callback_dustbin_cnt_old_cnt++;
    serial_status.callback_dustbin_cnt_old_cnt %= 32000;
    send_to_clean_cmd.port = 0;
    send_to_clean_cmd.engin_start = msg->engin_start;
    send_to_clean_cmd.lift_motor= msg->lift_motor;
    send_to_clean_cmd.main_brush = msg->main_brush;
    send_to_clean_cmd.spray_motor = msg->spray_motor;
    send_to_clean_cmd.dust_suppresion= msg->dust_suppresion;
    send_to_clean_cmd.side_brush = msg->side_brush;
    send_to_clean_cmd.led= msg->led;
    send_to_clean_cmd.control_mode= 2;//2：导航控制
    if(4 == box_type){
        send_to_clean_cmd.dustbin_on_car = 1;//清扫箱在车上
    }else{
        send_to_clean_cmd.dustbin_on_car = 0;
    }
    if(msg->engin_start == 1){
        //如果是开启清扫
        if(default_side_brush_transform >= 0 && default_side_brush_transform <= 100){
            send_to_clean_cmd.unused0 = default_side_brush_transform;
        }
        if(default_side_brush_speed >= 0 && default_side_brush_speed <= 100){
            send_to_clean_cmd.unused1 = default_side_brush_speed;
        }
    }
    if(msg->engin_start == 0){
        //如果是开启清扫
        send_to_clean_cmd.unused0 = 0;
        send_to_clean_cmd.unused1 = 0;

    }
    
    construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD, sizeof(send_to_clean_cmd), &send_to_clean_cmd);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,SEND_TO_DUSTBIN_CMD,0,user_frame));

    Info("S_CC_ES: "<<(int)msg->engin_start
    <<" S_CC_LM: "<<(int)msg->lift_motor
    <<" S_CC_MB: "<<(int)msg->main_brush
    <<" S_CC_SM: "<<(int)msg->spray_motor
    <<" S_CC_DS: "<<(int)msg->dust_suppresion
    <<" S_CC_SB: "<<(int)msg->side_brush
    <<" S_CC_LED: "<<(int)msg->led
    <<" S_CC_DOC: "<<(int)send_to_dustbin_cmd.dustbin_on_car
    );
}
//************************************** 环卫车清扫控制回调函数（新的消息类型（暂时保留））
void cleanControlNew_Callback(const cti_msgs::DustbinControlNew::ConstPtr &msg)
{
    if(stm32_update_flag){
        return;
    }
    serial_status.callback_dustbin_cnt_new_cnt++;
    serial_status.callback_dustbin_cnt_new_cnt %= 32000;
    motion_to_clean_t_new send_to_clean_cmd_new;
    send_to_clean_cmd_new.port = 0;
    send_to_clean_cmd_new.engin_start = msg->engin_start;
    send_to_clean_cmd_new.side_brush = msg->side_brush;
    send_to_clean_cmd_new.main_brush = msg->main_brush;
    send_to_clean_cmd_new.spray_motor = msg->spray_motor;
    send_to_clean_cmd_new.dust_suppresion= msg->dust_suppresion;
    send_to_clean_cmd_new.lift_motor= msg->lift_motor;
    send_to_clean_cmd_new.led= msg->led;
    send_to_clean_cmd_new.control_mode= 2;//2：导航控制
    if(4 == box_type){
        send_to_clean_cmd.dustbin_on_car = 1;//清扫箱在车上
    }else{
        send_to_clean_cmd.dustbin_on_car = 0;
    }
    send_to_clean_cmd_new.dam_board = msg->dam_board;
    send_to_clean_cmd_new.side_brush_transform = msg->side_brush_transform;
    send_to_clean_cmd_new.side_brush_speed = msg->side_brush_speed;
    send_to_clean_cmd_new.unused1 = msg->unused1;
    send_to_clean_cmd_new.unused2 = msg->unused2;
    send_to_clean_cmd_new.unused3 = msg->unused3;

    
    construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD, sizeof(send_to_clean_cmd_new), &send_to_clean_cmd_new);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,SEND_TO_DUSTBIN_CMD,0,user_frame));

    Info("S_CC_ES: "<<(int)msg->engin_start
    <<" S_CC_LM: "<<(int)msg->lift_motor
    <<" S_CC_MB: "<<(int)msg->main_brush
    <<" S_CC_SM: "<<(int)msg->spray_motor
    <<" S_CC_DS: "<<(int)msg->dust_suppresion
    <<" S_CC_SB: "<<(int)msg->side_brush
    <<" S_CC_LED: "<<(int)msg->led
    <<" S_CC_DOC: "<<(int)send_to_clean_cmd_new.dustbin_on_car
    <<" S_CC_DB: "<<(int)msg->dam_board
    <<" S_CC_SBT: "<<(int)msg->side_brush_transform
    <<" S_CC_SBS: "<<(int)msg->side_brush_speed
    <<" S_CC_UN1: "<<(int)msg->unused1
    <<" S_CC_UN2: "<<(int)msg->unused2
    <<" S_CC_UN3: "<<(int)msg->unused3
    );
}

//************************************** 环卫车清扫控制回调函数（DataArray消息类型）
void cleanControlInfo_Callback(const cti_msgs::DataArray::ConstPtr &msg)
{
    if(stm32_update_flag || clean_function_test_state.clean_function_enable){
        return;
    }

    //motion_to_clean_t_new send_to_clean_cmd_new;
    //数据解析
    for(int i = 0;i < msg->datas.size();i++){
        cti_msgs::Data info_msg = msg->datas[i];
        if(info_msg.name == "engin_start")
            //一键开启 uint8_t  1    0
            send_to_clean_cmd_new.engin_start = atoi(info_msg.data.c_str());
        else if(info_msg.name == "spray_motor")
            //喷水电机 uint8_t 1/0    0
            send_to_clean_cmd_new.spray_motor = atoi(info_msg.data.c_str());
        else if(info_msg.name == "sidebrush_lift")
            //边刷升降 int8_t 1   0
            send_to_clean_cmd_new.lift_motor = atoi(info_msg.data.c_str());
        else if(info_msg.name == "led")
            //led灯 uint8_t 1   0
            send_to_clean_cmd_new.led = atoi(info_msg.data.c_str());
        else if(info_msg.name == "dam_board")
            //挡板控制 uint8_t 0   0
            send_to_clean_cmd_new.dam_board = atoi(info_msg.data.c_str());
        else if(info_msg.name == "sidebrush_transform"){
            //边刷伸展 uint8_t 100   0
            if( atoi(info_msg.data.c_str()) < 0){
                send_to_clean_cmd_new.side_brush_transform = 0;
            }else if(atoi(info_msg.data.c_str()) > 100){
                send_to_clean_cmd_new.side_brush_transform = 100;
            }else{
                 send_to_clean_cmd_new.side_brush_transform = atoi(info_msg.data.c_str());
            }
        }
        else if(info_msg.name == "sidebrush_speed"){
            //边刷转速 uint8_t 100  0
            if( atoi(info_msg.data.c_str()) < 0){
                send_to_clean_cmd_new.side_brush_speed = 0;
            }else if(atoi(info_msg.data.c_str()) > 100){
                send_to_clean_cmd_new.side_brush_speed = 100;
            }else{
                 send_to_clean_cmd_new.side_brush_speed = atoi(info_msg.data.c_str());
            }
        }
        else if(info_msg.name == "decorate_light")
            //装饰灯 uint8_t 0:关 1:开
            send_to_clean_cmd_new.decorate_light = atoi(info_msg.data.c_str());     
        else if(info_msg.name == "lift_motor")  
            //边刷升降 int8_t 0:停止 1:上升 -1：下降  
            send_to_clean_cmd_new.lift_motor = atoi(info_msg.data.c_str());     
        else
            continue;     
    }
    //增加数据过滤,当engin_start == 0时，确保其他数据为0;
    if(send_to_clean_cmd_new.engin_start == 0){
        send_to_clean_cmd_new.spray_motor = 0;
        send_to_clean_cmd_new.dam_board = 0;
        send_to_clean_cmd_new.side_brush_transform = 0;
        send_to_clean_cmd_new.side_brush_speed = 0;
    }
    send_to_clean_cmd_new.port = 0;
    send_to_clean_cmd_new.side_brush = 0;
    send_to_clean_cmd_new.main_brush = 0;
    send_to_clean_cmd_new.dust_suppresion= 0;
    send_to_clean_cmd_new.control_mode= 2;//2：导航控制
    if(4 == box_type){
        send_to_clean_cmd.dustbin_on_car = 1;//清扫箱在车上
    }else{
        send_to_clean_cmd.dustbin_on_car = 0;
    }
    send_to_clean_cmd_new.unused1 = 0;
    send_to_clean_cmd_new.unused2 = 0;
    send_to_clean_cmd_new.unused3 = 0;


    send_to_clean_cmd.port = 0;
    send_to_clean_cmd.engin_start = send_to_clean_cmd_new.engin_start;
    send_to_clean_cmd.lift_motor= send_to_clean_cmd_new.lift_motor;
    send_to_clean_cmd.main_brush = 0;
    send_to_clean_cmd.spray_motor =send_to_clean_cmd_new.spray_motor;
    send_to_clean_cmd.dust_suppresion= 0;
    send_to_clean_cmd.side_brush = 0;
    send_to_clean_cmd.led= send_to_clean_cmd_new.led;
    send_to_clean_cmd.dam_board=send_to_clean_cmd_new.dam_board;
    send_to_clean_cmd.control_mode= 2;//2：导航控制
    if(4 == box_type){
        send_to_clean_cmd.dustbin_on_car = 1;//清扫箱在车上
    }else{
        send_to_clean_cmd.dustbin_on_car = 0;
    }
   //判断是<环卫车>还是<物流车+旧清扫箱> ,不同车辆发布不同结构体
    std::string clean_flag = "v6.0";
    std::string::size_type idx = cti_run_ver.find(clean_flag);
    if(idx != std::string::npos){
        construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD, sizeof(send_to_clean_cmd), &send_to_clean_cmd);
        // serial_frame_include_id_type user_frame_include_id_1;
        // user_frame_include_id_1.id = 0;
        // user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
        // user_frame_include_id_1.need_id = 0;
        // user_frame_include_id_1.frame = user_frame;
        // pushData(&user_frame_include_id_1);
        pushData(construct_frame_include_id(0,SEND_TO_DUSTBIN_CMD,0,user_frame));

    }else{  
        //std::cout<<"send_to_clean_cmd_new.dam_board: "<<send_to_clean_cmd_new.dam_board<<std::endl;
        //std::cout<<"send_to_clean_cmd_new.side_brush_transform: "<<send_to_clean_cmd_new.side_brush_transform<<std::endl;
        //std::cout<<"send_to_clean_cmd_new.side_brush_speed: "<<send_to_clean_cmd_new.side_brush_speed<<std::endl;    
        construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD, sizeof(send_to_clean_cmd_new), &send_to_clean_cmd_new);
        // serial_frame_include_id_type user_frame_include_id_1;
        // user_frame_include_id_1.id = 0;
        // user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
        // user_frame_include_id_1.need_id = 0;
        // user_frame_include_id_1.frame = user_frame;
        // pushData(&user_frame_include_id_1);
        pushData(construct_frame_include_id(0,SEND_TO_DUSTBIN_CMD,0,user_frame));
    }

    Info("S_CC_ES: "<<(int)send_to_clean_cmd_new.engin_start
    <<" S_CC_LM: "<<(int)send_to_clean_cmd_new.lift_motor
    <<" S_CC_MB: "<<(int)send_to_clean_cmd_new.main_brush
    <<" S_CC_SM: "<<(int)send_to_clean_cmd_new.spray_motor
    <<" S_CC_DS: "<<(int)send_to_clean_cmd_new.dust_suppresion
    <<" S_CC_SB: "<<(int)send_to_clean_cmd_new.side_brush
    <<" S_CC_LED: "<<(int)send_to_clean_cmd_new.led
    <<" S_CC_DOC: "<<(int)send_to_clean_cmd_new.dustbin_on_car
    <<" S_CC_DB: "<<(int)send_to_clean_cmd_new.dam_board
    <<" S_CC_SBT: "<<(int)send_to_clean_cmd_new.side_brush_transform
    <<" S_CC_SBS: "<<(int)send_to_clean_cmd_new.side_brush_speed
    <<" S_CC_DE_LG: "<<(int)send_to_clean_cmd_new.decorate_light
    <<" S_CC_UN1: "<<(int)send_to_clean_cmd_new.unused1
    <<" S_CC_UN2: "<<(int)send_to_clean_cmd_new.unused2
    <<" S_CC_UN3: "<<(int)send_to_clean_cmd_new.unused3
    );
}

//清扫功能测试回调函数
void cleanFunctionTest_Callback(const cti_msgs::DataArray::ConstPtr &msg){
    if(stm32_update_flag){
        return;
    }
     //数据解析
    for(int i = 0;i < msg->datas.size();i++){
        cti_msgs::Data info_msg = msg->datas[i];
        if(info_msg.name == "spray_motor")
            //喷水电机 uint8_t 1/0    0
            send_to_clean_cmd_new.spray_motor = atoi(info_msg.data.c_str());
        else if(info_msg.name == "dam_board")
            //挡板控制 uint8_t 0   0
            send_to_clean_cmd_new.dam_board = atoi(info_msg.data.c_str());
        else if(info_msg.name == "sidebrush_transform")
            //边刷伸展 uint8_t 100   0
            send_to_clean_cmd_new.side_brush_transform = atoi(info_msg.data.c_str());
        else if(info_msg.name == "sidebrush_speed")
            //边刷转速 uint8_t 100  0
            send_to_clean_cmd_new.side_brush_speed = atoi(info_msg.data.c_str());   
        else if(info_msg.name == "fanspeed")
            //风机速度  uint8_t 0 -100
            send_to_dust_box_cmd.fan_speed = atoi(info_msg.data.c_str());
        else
            continue;     
    }
    //清扫板
    send_to_clean_cmd_new.engin_start = 1;
    send_to_clean_cmd_new.lift_motor = 0;
    send_to_clean_cmd_new.decorate_light = 0;
    send_to_clean_cmd_new.port = 0;
    send_to_clean_cmd_new.side_brush = 0;
    send_to_clean_cmd_new.main_brush = 0;
    send_to_clean_cmd_new.dust_suppresion= 0;
    send_to_clean_cmd_new.control_mode= 2;//2：导航控制
    send_to_clean_cmd_new.unused1 = 0;
    send_to_clean_cmd_new.unused2 = 0;
    send_to_clean_cmd_new.unused3 = 0;
    construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD, sizeof(send_to_clean_cmd_new), &send_to_clean_cmd_new);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,SEND_TO_DUSTBIN_CMD,0,user_frame));
    
    //集尘箱
    send_to_dust_box_cmd.engin_start = 1;
    send_to_dust_box_cmd.control_mode= 2;//2：导航控制
    send_to_dust_box_cmd.port = 0;
    send_to_dust_box_cmd.lift_motor= 0;
    send_to_dust_box_cmd.main_brush = 0;
    send_to_dust_box_cmd.spray_motor = 0;
    send_to_dust_box_cmd.dust_suppresion= 0;
    send_to_dust_box_cmd.side_brush = 0;
    send_to_dust_box_cmd.unused0 = 0;
    send_to_dust_box_cmd.unused1 = 0;
    construct_serial_frame_ex(&user_frame, SEND_TO_DUST_BOX_CMD, sizeof(send_to_dust_box_cmd), &send_to_dust_box_cmd);
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_TO_DUST_BOX_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    // pushData(&user_frame_include_id_1);
    // pushData(&user_frame_include_id_1); //发三次，确保发下去
    pushData(construct_frame_include_id(0,SEND_TO_DUST_BOX_CMD,0,user_frame));
    pushData(construct_frame_include_id(0,SEND_TO_DUST_BOX_CMD,0,user_frame));
    pushData(construct_frame_include_id(0,SEND_TO_DUST_BOX_CMD,0,user_frame));
    

    //状态写入
    clean_function_test_state.clean_function_enable = true;
    clean_function_test_state.time_recv = ros::Time::now().toSec();
    clean_function_test_state.damboard_status = send_to_clean_cmd_new.dam_board;
    clean_function_test_state.side_brush_transform_state = send_to_clean_cmd_new.side_brush_transform;
    clean_function_test_state.side_brush_speed = send_to_clean_cmd_new.side_brush_speed;
    clean_function_test_state.spray_motor = send_to_clean_cmd_new.spray_motor;
    clean_function_test_state.fanspeed = send_to_dust_box_cmd.fan_speed;
    

    Info("S_CC_T_ES: "<<(int)send_to_clean_cmd_new.engin_start
    <<" S_CC_T_LM: "<<(int)send_to_clean_cmd_new.lift_motor
    <<" S_CC_T_MB: "<<(int)send_to_clean_cmd_new.main_brush
    <<" S_CC_T_SM: "<<(int)send_to_clean_cmd_new.spray_motor
    <<" S_CC_T_DS: "<<(int)send_to_clean_cmd_new.dust_suppresion
    <<" S_CC_T_SB: "<<(int)send_to_clean_cmd_new.side_brush
    <<" S_CC_T_LED: "<<(int)send_to_clean_cmd_new.led
    <<" S_CC_T_DOC: "<<(int)send_to_clean_cmd_new.dustbin_on_car
    <<" S_CC_T_DB: "<<(int)send_to_clean_cmd_new.dam_board
    <<" S_CC_T_SBT: "<<(int)send_to_clean_cmd_new.side_brush_transform
    <<" S_CC_T_SBS: "<<(int)send_to_clean_cmd_new.side_brush_speed
    <<" S_CC_T_DE_LG: "<<(int)send_to_clean_cmd_new.decorate_light
    <<" S_CC_T_UN1: "<<(int)send_to_clean_cmd_new.unused1
    <<" S_CC_T_UN2: "<<(int)send_to_clean_cmd_new.unused2
    <<" S_CC_T_UN3: "<<(int)send_to_clean_cmd_new.unused3
    <<" S_DBO_T_ES: "<<(int)send_to_dust_box_cmd.engin_start
    <<" S_DBO_T_FS: "<<(int)send_to_dust_box_cmd.fan_speed
    );
}
//************************************** 挡板控制回调函数<已经弃用>
void dustbinDamboard_Callback(const std_msgs::UInt8::ConstPtr &msg)
{
    if(stm32_update_flag){
        return;
    }
    send_to_clean_cmd.dam_board= msg->data;
    
    construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD, sizeof(send_to_clean_cmd), &send_to_clean_cmd);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,SEND_TO_DUSTBIN_CMD,0,user_frame));

    Info("S_CC_DB: "<<(int)msg->data);
}

//************************************** 边刷伸缩控制函数<已经弃用>
void dustbinSideBrushTrans_Callback(const std_msgs::UInt8::ConstPtr &msg)
{
    if(stm32_update_flag){
        return;
    }
    send_to_clean_cmd.unused0= msg->data;
    
    construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD, sizeof(send_to_clean_cmd), &send_to_clean_cmd);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,SEND_TO_DUSTBIN_CMD,0,user_frame));

    Info("S_CC_SBT: "<<(int)msg->data);
}

//************************************** 智能垃圾箱控制回调函数
void smartTrash_Callback(const cti_msgs::DataArray::ConstPtr &msg)
{
    if(stm32_update_flag){
        return;
    }

    smart_trash_cmd send_to_smart_trash;
    for(int i = 0;i < msg->datas.size();i++){
        cti_msgs::Data info_msg = msg->datas[i];
        if(info_msg.name == "lift_mode")
            //1:上升 2：下降：3:急停  uint8_t
            send_to_smart_trash.lift_mode = atoi(info_msg.data.c_str());
        else if(info_msg.name == "reaction_control")
            //1:关闭感应开盖 0:允许感应开盖
            send_to_smart_trash.reaction_control = atoi(info_msg.data.c_str());
        else
            continue;     
    }
    send_to_smart_trash.unuse2 = 0;
    send_to_smart_trash.unuse3 = 0;
    send_to_smart_trash.unuse4 = 0;
    
    construct_serial_frame_ex(&user_frame, SMART_TRASH_SEND, sizeof(send_to_smart_trash), &send_to_smart_trash);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SMART_TRASH_SEND;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,SMART_TRASH_SEND,0,user_frame));

    Info("S_ST_LM: "<<(int)send_to_smart_trash.lift_mode);
}

//************************************** 吸尘箱5g状态查询回调函数（转为物联网卡id获取）
void dustbox5GCheck_Callback(const cti_msgs::DataArray::ConstPtr &msg)
{
    if(stm32_update_flag){
        return;
    }

    dustbox_5g_check_cmd_t  dustbox_5g_check;
    for(int i = 0;i < msg->datas.size();i++){
        cti_msgs::Data info_msg = msg->datas[i];
        if(info_msg.name == "port")
            dustbox_5g_check.port = atoi(info_msg.data.c_str());
        else if(info_msg.name == "msg1")
            dustbox_5g_check.msg1 = atoi(info_msg.data.c_str());
        else if(info_msg.name == "unused1")
            dustbox_5g_check.unused1 = atoi(info_msg.data.c_str());
        else if(info_msg.name == "unused2")
            dustbox_5g_check.unused2 = atoi(info_msg.data.c_str());
        else
            continue;     
    }

    construct_serial_frame_ex(&user_frame, DUST_BOX_5G_CHECK_CMD, sizeof(dustbox_5g_check), &dustbox_5g_check);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = DUST_BOX_5G_CHECK_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,DUST_BOX_5G_CHECK_CMD,0,user_frame));

    Info("S_DOB_5G_CH_PO: "<<(int)dustbox_5g_check.port
       <<"S_DOB_5G_CH_MSG1: "<<(int)dustbox_5g_check.msg1);
}

//************************************** 大屏控制回调函数,用来记录log方便分析问题
void ledshow_Callback(const std_msgs::Int32::ConstPtr &msg)
{
    Info("LED: "<<(int)msg->data);
}
//************************************** 集尘箱控制回调函数（弃用）
void dustBoxControl_Callback(const cti_msgs::DustbinControl::ConstPtr &msg)
{
    if(stm32_update_flag){
        return;
    }
    serial_status.callback_dustbox_cnt_old_cnt++;
    serial_status.callback_dustbox_cnt_old_cnt %= 32000;
    send_to_dust_box_cmd.port = 0;
    send_to_dust_box_cmd.engin_start = msg->engin_start;
    send_to_dust_box_cmd.lift_motor= msg->lift_motor;
    send_to_dust_box_cmd.main_brush = msg->main_brush;
    send_to_dust_box_cmd.spray_motor = msg->spray_motor;
    send_to_dust_box_cmd.dust_suppresion= msg->dust_suppresion;
    send_to_dust_box_cmd.side_brush = msg->side_brush;
    send_to_dust_box_cmd.led= msg->led;
    send_to_dust_box_cmd.control_mode= 2;//2：导航控制


    construct_serial_frame_ex(&user_frame, SEND_TO_DUST_BOX_CMD, sizeof(send_to_dust_box_cmd), &send_to_dust_box_cmd);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_TO_DUST_BOX_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,SEND_TO_DUST_BOX_CMD,0,user_frame));

    Info("S_DBO_ES: "<<(int)msg->engin_start);
}

//************************************** 集尘箱控制回调函数,新的消息类型,把控制信息集成到一起<弃用>
void dustBoxControlNew_Callback(const cti_msgs::DustboxControl::ConstPtr &msg)
{
    if(stm32_update_flag){
        return;
    }
    serial_status.callback_dustbox_cnt_new_cnt++;
    serial_status.callback_dustbox_cnt_new_cnt %= 32000;
    //结构体并没有改变只是把风机速度和风机启动都放在了这里
    send_to_dust_box_cmd.port = 0;
    send_to_dust_box_cmd.engin_start = msg->engin_start;
    send_to_dust_box_cmd.lift_motor= msg->lift_motor;
    send_to_dust_box_cmd.main_brush = msg->main_brush;
    send_to_dust_box_cmd.spray_motor = msg->spray_motor;
    send_to_dust_box_cmd.dust_suppresion= msg->dust_suppresion;
    send_to_dust_box_cmd.side_brush = msg->side_brush;
    send_to_dust_box_cmd.led= msg->led;
    send_to_dust_box_cmd.control_mode= 2;//2：导航控制
    send_to_dust_box_cmd.fan_speed = msg->fan_speed;
    send_to_dust_box_cmd.unused0 = msg->unused0;
    send_to_dust_box_cmd.unused1 = msg->unused1;

    construct_serial_frame_ex(&user_frame, SEND_TO_DUST_BOX_CMD, sizeof(send_to_dust_box_cmd), &send_to_dust_box_cmd);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_TO_DUST_BOX_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,SEND_TO_DUST_BOX_CMD,0,user_frame));

    Info("S_DBO_ES: "<<(int)msg->engin_start
        <<"S_DBO_FS: "<<(int)msg->fan_speed
    );
}


//************************************** 集尘箱控制回调函数,cti_msgs::DataArray 消息类型
void dustBoxControlInfo_Callback(const cti_msgs::DataArray::ConstPtr &msg)
{
    if(stm32_update_flag || clean_function_test_state.clean_function_enable){
        return;
    }
    //数据解析
    for(int i = 0;i < msg->datas.size();i++){
        cti_msgs::Data info_msg = msg->datas[i];
        if(info_msg.name == "engin_start")
            //吸尘箱启动 uint8_t
            send_to_dust_box_cmd.engin_start = atoi(info_msg.data.c_str());
        else if(info_msg.name == "fanspeed")
            //风机速度  uint8_t
            send_to_dust_box_cmd.fan_speed = atoi(info_msg.data.c_str());
        else if(info_msg.name == "led")
            //led灯 uint8_t
            send_to_dust_box_cmd.led = atoi(info_msg.data.c_str());
        else if(info_msg.name == "dust_suppresion")
            //led灯 uint8_t
            send_to_dust_box_cmd.dust_suppresion = atoi(info_msg.data.c_str());
        else
            continue;     
    }
    send_to_dust_box_cmd.control_mode= 2;//2：导航控制
    send_to_dust_box_cmd.port = 0;
    send_to_dust_box_cmd.lift_motor= 0;
    send_to_dust_box_cmd.main_brush = 0;
    send_to_dust_box_cmd.spray_motor = 0;
    //send_to_dust_box_cmd.dust_suppresion= 0;
    send_to_dust_box_cmd.side_brush = 0;
    send_to_dust_box_cmd.unused0 = 0;
    send_to_dust_box_cmd.unused1 = 0;


    construct_serial_frame_ex(&user_frame, SEND_TO_DUST_BOX_CMD, sizeof(send_to_dust_box_cmd), &send_to_dust_box_cmd);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_TO_DUST_BOX_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,SEND_TO_DUST_BOX_CMD,0,user_frame));

    Info("S_DBO_ES: "<<(int)send_to_dust_box_cmd.engin_start
        <<"S_DBO_FS: "<<(int)send_to_dust_box_cmd.fan_speed
    );
}

//************************************** 集尘箱自动倒垃圾控制回调函数
void dustBoxAutopushControl_Callback(const std_msgs::UInt8::ConstPtr &msg)
{
    if(stm32_update_flag ){
        return;
    }
    send_to_dust_box_cmd.auto_push= msg->data;

    construct_serial_frame_ex(&user_frame, SEND_TO_DUST_BOX_CMD, sizeof(send_to_dust_box_cmd), &send_to_dust_box_cmd);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_TO_DUST_BOX_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,SEND_TO_DUST_BOX_CMD,0,user_frame));


    //倾倒指令发下去之后 ，恢复空闲值 0
    send_to_dust_box_cmd.auto_push = 0;

    Info("S_DBO_AP: "<<(int)msg->data);
}
//************************************** 集尘箱风机速度控制回调函数<弃用>
void dustBoxFanSpeed_Callback(const std_msgs::UInt8::ConstPtr &msg)
{
    if(stm32_update_flag){
        return;
    }
    send_to_dust_box_cmd.fan_speed= msg->data;
    construct_serial_frame_ex(&user_frame, SEND_TO_DUST_BOX_CMD, sizeof(send_to_dust_box_cmd), &send_to_dust_box_cmd);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_TO_DUST_BOX_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,SEND_TO_DUST_BOX_CMD,0,user_frame));
    Info("S_DBO_FS: "<<(int)msg->data);
}
//************************************** 清扫箱控制(带重发和超时)回调函数<没用，原来设计为app下发指令单独控制>
void dustbinControlResend_Callback(const cti_msgs::DustbinControl::ConstPtr &msg)
{ 
    if(stm32_update_flag){
        return;
    }
    check_clean_mechine_state = true;

    resend_to_dustbin_cmd.port = 0;
    resend_to_dustbin_cmd.engin_start = msg->engin_start;
    resend_to_dustbin_cmd.lift_motor= msg->lift_motor;
    resend_to_dustbin_cmd.main_brush = msg->main_brush;
    resend_to_dustbin_cmd.spray_motor = msg->spray_motor;
    resend_to_dustbin_cmd.dust_suppresion= msg->dust_suppresion;
    resend_to_dustbin_cmd.side_brush = msg->side_brush;
    resend_to_dustbin_cmd.led= msg->led;
    resend_to_dustbin_cmd.control_mode= 2;//2：导航控制
    
    construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD, sizeof(resend_to_dustbin_cmd), &resend_to_dustbin_cmd);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,SEND_TO_DUSTBIN_CMD,0,user_frame));

    Info("RS_DBI_ES: "<<(int)msg->engin_start
    <<" RS_DBI_LM: "<<(int)msg->lift_motor
    <<" RS_DBI_MB: "<<(int)msg->main_brush
    <<" RS_DBI_SM: "<<(int)msg->spray_motor
    <<" RS_DBI_DS: "<<(int)msg->dust_suppresion
    <<" RS_DBI_SB: "<<(int)msg->side_brush
    <<" RS_DBI_LED: "<<(int)msg->led
    <<" RS_DBI_DOC: "<<(int)send_to_dustbin_cmd.dustbin_on_car
    );
}

//************************************** 环卫车清扫控制(带重发和超时)回调函数<没用，原来设计为app下发指令单独控制>
void cleanControlResend_Callback(const cti_msgs::DustbinControl::ConstPtr &msg)
{ 
    if(stm32_update_flag){
        return;
    }
    check_clean_mechine_state = true;

    resend_to_clean_cmd.port = 0;
    resend_to_clean_cmd.engin_start = msg->engin_start;
    resend_to_clean_cmd.lift_motor= msg->lift_motor;
    resend_to_clean_cmd.main_brush = msg->main_brush;
    resend_to_clean_cmd.spray_motor = msg->spray_motor;
    resend_to_clean_cmd.dust_suppresion= msg->dust_suppresion;
    resend_to_clean_cmd.side_brush = msg->side_brush;
    resend_to_clean_cmd.led= msg->led;
    resend_to_clean_cmd.control_mode= 2;//2：导航控制
    
    construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD, sizeof(resend_to_clean_cmd), &resend_to_clean_cmd);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,SEND_TO_DUSTBIN_CMD,0,user_frame));

    Info("RS_CC_ES: "<<(int)msg->engin_start
    <<" RS_CC_LM: "<<(int)msg->lift_motor
    <<" RS_CC_MB: "<<(int)msg->main_brush
    <<" RS_CC_SM: "<<(int)msg->spray_motor
    <<" RS_CC_DS: "<<(int)msg->dust_suppresion
    <<" RS_CC_SB: "<<(int)msg->side_brush
    <<" RS_CC_LED: "<<(int)msg->led
    <<" RS_CC_DOC: "<<(int)send_to_dustbin_cmd.dustbin_on_car
    );
}


//************************************** 车上箱子状态回调函数，用来获取箱子类型来判断是否为旧的清扫箱，是的话禁止箱子在车上时打开垃圾仓门。
void boxtype_Callback(const std_msgs::Int32::ConstPtr &msg)
{
    set_dustbin_on_car = true;
    box_type = msg->data;
    Info("CB_BT: "<<(int)box_type);
}

//**************************************清扫箱id设定命令回调函数
void sweeperBoxID_Callback(const std_msgs::Int64::ConstPtr &msg){
    dustbox_lora_id = msg->data;
    if(stm32_update_flag){
        return;
    }
    Info("CB_SID: " << (int)msg->data);
    if(msg->data == -1){//取消配对
        //如果收到的id和现在设置的不同（无论是否在设置）或者（收到的id和已经设置的id不同且没有在设置）
        if(dustbin_set_state.send_id != 9999 ||
            (dustbin_set_state.recv_id != 9999 && !dustbin_set_state.need_setting)){//没有取消成功才发送
            dustbin_set_state.need_setting = true;
            dustbin_set_state.setting_start_time = ros::Time::now().toSec();
            dustbin_set_state.send_id = 9999;
            dustbin_set_state.recv_id = -2;
            dustbin_set_state.set_read_switch = true;
        }
    }else{
        //如果收到的id和现在设置的不同（无论是否在设置）或者（收到的id和已经设置的id不同且没有在设置）
        if(  dustbin_set_state.send_id != msg->data || 
            (dustbin_set_state.recv_id != msg->data && !dustbin_set_state.need_setting)){
            dustbin_set_state.need_setting = true;
            dustbin_set_state.setting_start_time = ros::Time::now().toSec();
            dustbin_set_state.send_id = msg->data;
            dustbin_set_state.recv_id = -2;
            dustbin_set_state.set_read_switch = true;
        }
    }
}
//*********************************************无线充电控制命令回调
void wirelessCharge_Callback(const std_msgs::UInt8::ConstPtr &msg){
    if(stm32_update_flag){
        return;
    }
    battery_board_cmd_t  send_battery_board;
    Info("S_WC: " << (int)msg->data);
    send_battery_board.wireless_charge = msg->data;
    send_battery_board.unused1 = 0;
    send_battery_board.unused2 = 0;
    send_battery_board.unused3 = 0;
    send_battery_board.unused4 = 0;
    send_battery_board.unused5 = 0;

    construct_serial_frame_ex(&user_frame, SEND_BATTERY_CMD, sizeof(send_battery_board), &send_battery_board);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_BATTERY_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,SEND_BATTERY_CMD,0,user_frame));
}

//***************************************lin通讯超声波控制接收
void ultCmdCallback(const std_msgs::Int64::ConstPtr &msg){
    //车身超声波处理
    if(!LIN_ult_installed){
        return;
    }
    static int64_t ult_cmd_pre = 0;
    if(ult_cmd_pre ==  msg->data){
        return;
    }else{
        ult_cmd_pre = msg->data;
    }
    vehicle_ult_cmd = 0x0000000000003fff & msg->data;
    vehicle_ult_set_state.set_mode = vehicle_ult_cmd;
    //如果是关闭或者打开整体超声波需要设置，如果只是设置单个的超声波就不需要发送设置
    if( vehicle_ult_set_state.set_mode == 0){
        vehicle_ult_set_state.state = 1; //开始查询
        vehicle_ult_set_state.set_mode = vehicle_all_stop_work_mode.data;//设置全部休眠
    }
    else if(vehicle_ult_set_state.set_mode != 0 && vehicle_ult_set_state.recv_mode != vehicle_ult_set_state.set_mode){
        vehicle_ult_set_state.state = 1; //开始查询
        vehicle_ult_set_state.set_mode = vehicle_defalt_linult_mode.data;//设置默认模式
    }
    else{
        vehicle_ult_set_state.state = 0; //空闲
        vehicle_ult_set_state.set_mode = vehicle_defalt_linult_mode.data;//默认模式
    }
    
    //箱子超声波处理
    dustbox_ult_cmd = 0x000000000001c000 & msg->data;
    dustbox_ult_set_state.set_mode = dustbox_ult_cmd;
    //如果是关闭或者打开整体超声波需要设置，如果只是设置单个的超声波就不需要发送设置
    if( dustbox_ult_set_state.set_mode == 0){
        dustbox_ult_set_state.state = 1; //开始查询
        dustbox_ult_set_state.set_mode = dustbox_all_stop_work_mode.data;//设置全部休眠
    }
    else if(dustbox_ult_set_state.set_mode != 0 && dustbox_ult_set_state.recv_mode != dustbox_ult_set_state.set_mode){
        dustbox_ult_set_state.state = 1; //开始查询
        dustbox_ult_set_state.set_mode = dustbox_defalt_linult_mode.data;//设置默认模式
    }
    else{
        dustbox_ult_set_state.state = 0; //空闲
        dustbox_ult_set_state.set_mode = dustbox_defalt_linult_mode.data;//默认模式
    }




#if DEBUG_PRINT
    printf("recv ult cmd >>>>>>>>>>>>>>>>>>>>>>>>>>> %d\n",msg->data);
#endif
    Info("V_ULT_CMD: "<<(int)msg->data);
}

//************************************** 接收到控制状态后发布话题
void pub_odom(const recv_from_control_status_type *data)
{
    if(!data){
        return;
    }
//-------------------通讯检测------------------
    chassis_chat_state.module_id = 0;
    chassis_chat_state.recv_state = true;
    chassis_chat_state.time_recv = ros::Time::now().toSec();
//---------------------------odom_4wd publish
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(data->angle_yaw_radian);
    odom_4wd.header.stamp = ros::Time::now();
    odom_4wd.header.frame_id = "odom";
    //set the position
    odom_4wd.pose.pose.position.x = data->pos.x;
    odom_4wd.pose.pose.position.y = data->pos.y;
    odom_4wd.pose.pose.position.z = 0.0;
    odom_4wd.pose.pose.orientation = odom_quat;
    //set the velocity
    odom_4wd.child_frame_id = "base_link";
    odom_4wd.twist.twist.linear.x = data->vel_liner_x; // vx == 100时！是底盘有问题！！！！！！！！！
    odom_4wd.twist.twist.linear.y = data->vel_liner_y;
    odom_4wd.twist.twist.angular.z = data->vel_angular;
    //publish the message
    odom_pub_4wd.publish(odom_4wd);

//---------------------------ctlinfo publish
    runinfo.state_vehicle = data->state_vehicle; //底盘状态
    runinfo.drivers_enable = data->drivers_enable; //电机驱动使能
    runinfo.control_mode = data->control_mode;   //控制方式 1:遥控 2：导航
    runinfo.state_brake = data->state_brake;     //刹车
    runinfo.vel_liner_x = data->vel_liner_x; //x轴线速度    vx == 100时！是底盘有问题！！！！！！！！！
    runinfo.vel_liner_y = data->vel_liner_y;  //Y轴线速度
    runinfo.vel_angular = data->vel_angular;   //角速度
    runinfo.angle_front_turn = data->angle_front_turn; //前轮转角
    runinfo.angle_rear_turn = data->angle_rear_turn;   //后轮转角
    //霍尔信号
    uint8_t box_sw = 0;
    box_sw = box_sw | (data->sw_status.bits.box_pos_reach1 & 0x01);
    box_sw = box_sw | ((data->sw_status.bits.box_pos_reach2<<1) & 0x02);
    box_sw = box_sw | ((data->sw_status.bits.box_pos_reach3<<2) & 0x04);
    //触边
    box_sw = box_sw | ((!(data->sw_status.bits.lift_pressure)<<3) & 0x08);
    runinfo.box_sw = box_sw;       //霍尔信号+触边
    runinfo.front_bar = !(data->sw_status.bits.front_bar); //前防撞杆
    runinfo.rear_bar = !(data->sw_status.bits.rear_bar);  //后防撞杆
    runinfo.lift_position = data->lift_position;   //顶升位置
    runinfo.odometer = data->odometer; //里程
    ctlinfo_pub.publish(runinfo);
//---------------------------Imu publish
    
    //imu 时间戳
    imu_info.header.stamp = ros::Time::now();
    imu_info.header.frame_id = "/fpga_serial/imu";
    //四元数位姿
    imu_info.orientation.w = data->q[0];
    imu_info.orientation.x = data->q[1];
    imu_info.orientation.y = data->q[2];
    imu_info.orientation.z = data->q[3];
    if (std::isnan(data->q[0]) || std::isnan(data->q[1]) || std::isnan(data->q[2]) || std::isnan(data->q[3]))
    {
         serial_status.data = serial_status.data | 0x0020; //如果四元数出现NAN，状态位倒数第六位置1
         serial_status.recv_imu_nan = -1;
    }
    else
    {
         serial_status.data = serial_status.data & 0xFFDF; //如果四元数没有NAN，状态位倒数第六位置0
         serial_status.recv_imu_nan = 0;
    }
    //线加速度
    imu_info.linear_acceleration.x = data->acc[0] * GRAV;
    imu_info.linear_acceleration.y = data->acc[1] * GRAV;
    imu_info.linear_acceleration.z = -data->acc[2] * GRAV;
    //角速度
    imu_info.angular_velocity.x = data->gyro[0] * PI/180.0; 
    imu_info.angular_velocity.y = data->gyro[1] * PI/180.0; 
    imu_info.angular_velocity.z = -data->gyro[2] * PI/180.0;
    //发布
    imudata_pub.publish(imu_info);
    //sins时间戳
    sins_info.header.stamp = imu_info.header.stamp;
    sins_info.header.frame_id = "/fpga_serial/sins";
    //imu
    sins_info.imu_data = imu_info;
    //位置
    sins_info.position_x = data->position[0];
    sins_info.position_y = data->position[1];
    sins_info.position_z = data->position[2];
    //线速度
    sins_info.linear_velocity_x = data->linear_velocity[0];
    sins_info.linear_velocity_y = data->linear_velocity[1];
    sins_info.linear_velocity_z = data->linear_velocity[2];
    //气压计  
    sins_info.barometer_rawdata = data->baro_raw;
    //判断气压计数据突然出现大于100 的跳变
    if (abs(data->baro_raw - baro_raw_old) > 100)
    {
        baro_status.data = 1;
        baro_status_pub.publish(baro_status);
        Info("BARO_RAW_ERROR: "<<" rawdata_before: "<<baro_raw_old<<" rawdata_now: "<<data->baro_raw);
        baro_status.data = 0;
    } 
    baro_raw_old = data->baro_raw;
    sins_info.barometer_height = data->baro_height;
    //标志位
    sins_info.state_flag = data->state_flag;
    //发布
    sins_pub.publish(sins_info);

//---------------------------box_laser publish
    box_laser.header.stamp = ros::Time::now();
    box_laser.header.frame_id = "/fpga_serial/box_laser";
    uint8_t id_data[6];
    for(int i = 0;i<6;i++)
    {
        id_data[i] = data->laser_id[i];
    }
    std::string id_str;
    id_str = hex2string(&(id_data[0]),6);
    box_laser.rtcm_type = id_str;
    box_laser.data.push_back(data->laser_data[0]);
    box_laser.data.push_back(data->laser_data[1]);
    box_laser_pub.publish(box_laser);
    box_laser.data.clear();
//------------------------磁吸锁状态发布
std_msgs::UInt8 boxlockstate;
boxlockstate.data = data->laser_id[0];
boxlock_state_pub.publish(boxlockstate); //按位赋值 bit1: 左磁吸锁， bit2: 右磁吸锁  3：（代表磁吸锁吸合）  0：代表放开

//--------------------------磁力计发布
    std_msgs::Int32MultiArray compass_msg;
    compass_msg.data.push_back(data->compass1_str[0]);
    compass_msg.data.push_back(data->compass1_str[1]);
    compass_msg.data.push_back(data->compass1_str[2]);
    compass_msg.data.push_back(data->compass2_str[0]);
    compass_msg.data.push_back(data->compass2_str[1]);
    compass_msg.data.push_back(data->compass2_str[2]);
    compass_pub.publish(compass_msg);

//---------------------------发布到话题"/cti/fpga_serial/serial_status"
serial_status.recv_hall_turn_front_left = data->sw_status.bits.turn_front_left;
serial_status.recv_hall_turn_front_center = data->sw_status.bits.turn_front_center;
serial_status.recv_hall_turn_front_right = data->sw_status.bits.turn_front_right;
serial_status.recv_hall_turn_rear_left = data->sw_status.bits.turn_rear_left;
serial_status.recv_hall_turn_rear_center = data->sw_status.bits.turn_rear_center;
serial_status.recv_hall_turn_rear_right = data->sw_status.bits.turn_rear_right;
serial_status.recv_box_pos_reach1 = data->sw_status.bits.box_pos_reach1;
serial_status.recv_box_pos_reach2 = data->sw_status.bits.box_pos_reach2;
serial_status.recv_box_pos_reach2 = data->sw_status.bits.box_pos_reach2;
serial_status.recv_lift_pressure = !(data->sw_status.bits.lift_pressure);
//-------------------------接受到的所有信息全部发布出去,用于记log
cti_fpga_serial::vehiclestate chassis_info_msg;
chassis_info_msg.portIndex = data->portIndex;
chassis_info_msg.state_vehicle = data->state_vehicle;
chassis_info_msg.drivers_enable = data->drivers_enable;
chassis_info_msg.control_mode =  data->control_mode;
chassis_info_msg.state_brake = data->state_brake;
chassis_info_msg.vel_liner_x = data->vel_liner_x;
chassis_info_msg.vel_liner_y = data->vel_liner_y;
chassis_info_msg.vel_angular = data->vel_angular;
chassis_info_msg.angle_front_turn = data->angle_front_turn;
chassis_info_msg.angle_rear_turn = data->angle_rear_turn;
chassis_info_msg.sw_status_data = data->sw_status.data;
chassis_info_msg.odometer = data->odometer;
chassis_info_msg.pos_x = data->pos.x;
chassis_info_msg.pos_y = data->pos.y;
chassis_info_msg.angle_yaw_radian = data->angle_yaw_radian;
chassis_info_msg.lift_position = data->lift_position;
for(int i = 0 ; i <(sizeof(data->acc)/sizeof(data->acc[0]));i++){
    chassis_info_msg.acc.push_back(data->acc[i]);
}
for(int i = 0 ; i <(sizeof(data->gyro)/sizeof(data->gyro[0]));i++){
    chassis_info_msg.gyro.push_back(data->gyro[i]);
}
for(int i = 0 ; i <(sizeof(data->q)/sizeof(data->q[0]));i++){
    chassis_info_msg.q.push_back(data->q[i]);
}
for(int i = 0 ; i <(sizeof(data->position)/sizeof(data->position[0]));i++){
    chassis_info_msg.position.push_back(data->position[i]);
}
for(int i = 0 ; i <(sizeof(data->linear_velocity)/sizeof(data->linear_velocity[0]));i++){
    chassis_info_msg.linear_velocity.push_back(data->linear_velocity[i]);
}
chassis_info_msg.baro_raw = data->baro_raw;
chassis_info_msg.baro_height = data->baro_height;
for(int i = 0 ; i <(sizeof(data->compass1_str)/sizeof(data->compass1_str[0]));i++){
    chassis_info_msg.compass1_str.push_back(data->compass1_str[i]);
}
for(int i = 0 ; i <(sizeof(data->compass2_str)/sizeof(data->compass2_str[0]));i++){
    chassis_info_msg.compass2_str.push_back(data->compass2_str[i]);
}
chassis_info_msg.state_flag = data->state_flag;
for(int i = 0 ; i <(sizeof(data->laser_id)/sizeof(data->laser_id[0]));i++){
    chassis_info_msg.laser_id.push_back(data->laser_id[i]);
}
for(int i = 0 ; i <(sizeof(data->laser_data)/sizeof(data->laser_data[0]));i++){
    chassis_info_msg.laser_data.push_back(data->laser_data[i]);
}
recv_chassis_info_pub.publish(chassis_info_msg);
}

//************************************** 接收到电池状态后发布话题 不带无线充电
void pub_battery(const recv_battery_4_to_1_active_report_status_type *data)
{
    if(!data){
        return;
    }
    batteryState.header.stamp = ros::Time::now();
    batteryState.voltage_all = cmax(cmax(data->Bat_1_Volt,data->Bat_2_Volt), data->Bat_3_Volt);
    //batteryState.power = cmax(cmax(data->Bat_1_Soc,data->Bat_2_Soc),data->Bat_3_Soc);
    if(robot_type == 0){
        //阳光物流车
        batteryState.power = (data->Bat_1_Soc + data->Bat_2_Soc +data->Bat_3_Soc) / 3;
    }else if(robot_type == 1){
        //阳光环卫车
        batteryState.power = (data->Bat_1_Soc + data->Bat_2_Soc) / 2;
    }
    if(batteryState.power > 100){
        batteryState.power = 100;
    }
    batteryState.temperature = cmax(cmax(data->Bat_1_temp,data->Bat_2_temp),data->Bat_3_temp);
    batteryState.power_supply_status = data->ChargeStatus;
    batteryState.wireless_install_state = 0;
    batteryState.wireless_voltage = 0;
    batteryState.wireless_current = 0;
    batteryState.wireless_reserve = 0;
    batteryState.wireless_state = 0;
    batteryState.wireless_temp = 0;
    batteryState.wireless_changer = 0;
    battery_pub.publish(batteryState);

    BatCellsState.BatCells.clear();
    BatCellsState.header.stamp = ros::Time::now();
    BatCellsState.Soc_All = batteryState.power;
    BatCellsState.Volt_All = cmax(cmax(data->Bat_1_Volt,data->Bat_2_Volt), data->Bat_3_Volt);
    cti_msgs::BatteryCell BatCell;
    if( data->Bat_1_Volt > 5){
        BatCell.Bat_Volt = data->Bat_1_Volt;
        BatCell.Bat_temp = data->Bat_1_temp;
        BatCell.Bat_curr = data->Bat_1_curr;
        BatCell.Bat_Soc  = data->Bat_1_Soc;
        BatCell.Bat_comm_state = data->Bat_1_comm_state;
        BatCell.Bat_cell_num   = data->Bat_1_cell_num;
        BatCellsState.BatCells.push_back(BatCell);
    }
    if(data->Bat_2_Volt > 5){
        BatCell.Bat_Volt = data->Bat_2_Volt;
        BatCell.Bat_temp = data->Bat_2_temp;
        BatCell.Bat_curr = data->Bat_2_curr;
        BatCell.Bat_Soc  = data->Bat_2_Soc;
        BatCell.Bat_comm_state = data->Bat_2_comm_state;
        BatCell.Bat_cell_num   = data->Bat_2_cell_num;
        BatCellsState.BatCells.push_back(BatCell);
    }
    if(data->Bat_3_Volt > 5){
        BatCell.Bat_Volt = data->Bat_3_Volt;
        BatCell.Bat_temp = data->Bat_3_temp;
        BatCell.Bat_curr = data->Bat_3_curr;
        BatCell.Bat_Soc  = data->Bat_3_Soc;
        BatCell.Bat_comm_state = data->Bat_3_comm_state;
        BatCell.Bat_cell_num   = data->Bat_3_cell_num;
        BatCellsState.BatCells.push_back(BatCell);
    }
    //--
    BatCellsState.Bat_backup_Volt= data->Bat_backup_Volt;
    BatCellsState.Bus_Volt       = data->Bus_Volt;
    BatCellsState.Charger_Volt   = data->Charger_Volt;

    BatCellsState.Board_Temp.clear();
    BatCellsState.Board_Temp.push_back(data->Board_temp1);
    BatCellsState.Board_Temp.push_back(data->Board_temp2);
    BatCellsState.Board_Temp.push_back(data->Board_temp3);
    BatCellsState.Board_Temp.push_back(data->Board_temp4);
    BatCellsState.Board_Temp.push_back(data->Board_temp5);
    BatCellsState.Board_Temp.push_back(data->Board_temp6);
    BatCellsState.Board_Temp.push_back(data->Board_temp7);
    BatCellsState.Board_Temp.push_back(data->Board_temp8);
    BatCellsState.Board_Curr.clear();
    BatCellsState.Board_Curr.push_back(data->Board_curr1);
    BatCellsState.Board_Curr.push_back(data->Board_curr2);
    BatCellsState.Board_Curr.push_back(data->Board_curr3);
    BatCellsState.Board_Curr.push_back(data->Board_curr4);
    BatCellsState.Board_Curr.push_back(data->Board_curr5);
    BatCellsState.Board_Curr.push_back(data->Board_curr6);
    BatCellsState.Board_Curr.push_back(data->Board_curr7);
    BatCellsState.Board_Curr.push_back(data->Board_curr8);
    //--
    BatCellsState.charge_reverse = data->charge_reverse;
    BatCellsState.v_leakage = data->v_leakage;
    BatCellsState.ChargeStatus = data->ChargeStatus;
    BatCellsState.Lock_status = data->Lock_status;
    BatCellsState.errorInfo = data->errorInfo;
    //充电状态
    BatCellsState.charge_type = 0; //0:undefined 1:wireless_charge 2:wired_charge
    batcell_pub.publish(BatCellsState);
    Info("R_WC_IS: "<<(int)batteryState.wireless_install_state
    <<" R_BA_VT: "<<batteryState.voltage_all
    <<" R_BA_PW: "<<(int)batteryState.power
    <<" R_BA_1_VO: "<<data->Bat_1_Volt
    <<" R_BA_2_VO: "<<data->Bat_2_Volt
    <<" R_BA_3_VO: "<<data->Bat_3_Volt
    <<" R_BA_1_SOC: "<<(int)data->Bat_1_Soc
    <<" R_BA_2_SOC: "<<(int)data->Bat_2_Soc
    <<" R_BA_3_SOC: "<<(int)data->Bat_3_Soc
    <<" R_BA_VO: "<<BatCellsState.Volt_All
    <<" R_BA_SOC: "<<(int)BatCellsState.Soc_All
    );
}

//************************************** 接收到电池状态后发布话题,带有无线充电
void pub_battery_with_wireless(const recv_battery_4_to_1_active_report_status_with_wireless_type *data)
{
    if(!data){
        return;
    }
    batteryState.header.stamp = ros::Time::now();
    batteryState.voltage_all = cmax(cmax(data->Bat_1_Volt,data->Bat_2_Volt), data->Bat_3_Volt);
    //batteryState.power = cmax(cmax(data->Bat_1_Soc,data->Bat_2_Soc),data->Bat_3_Soc);
    if(robot_type == 0){
        //阳光物流车
        batteryState.power = (data->Bat_1_Soc + data->Bat_2_Soc +data->Bat_3_Soc) / 3;
    }else if(robot_type == 1){
        //阳光环卫车
        batteryState.power = (data->Bat_1_Soc + data->Bat_2_Soc) / 2;
    }
    if(batteryState.power > 100){
        batteryState.power = 100;
    }
    batteryState.temperature = cmax(cmax(data->Bat_1_temp,data->Bat_2_temp),data->Bat_3_temp);
    batteryState.power_supply_status = data->ChargeStatus;
    batteryState.wireless_install_state = 1;
    batteryState.wireless_voltage = data->wireless_voltage;
    batteryState.wireless_current = data->wireless_current;
    batteryState.wireless_reserve = data->wireless_reserve;
    batteryState.wireless_state = data->wireless_state;
    batteryState.wireless_temp = data->wireless_temp;
    batteryState.wireless_changer = data->wireless_changer;
    battery_pub.publish(batteryState);

    //新的电池信息发布
    BatCellsState.BatCells.clear();
    BatCellsState.header.stamp = ros::Time::now();
    BatCellsState.Soc_All = batteryState.power;
    BatCellsState.Volt_All = cmax(cmax(data->Bat_1_Volt,data->Bat_2_Volt), data->Bat_3_Volt);
    cti_msgs::BatteryCell BatCell;
    if(data->Bat_1_Volt > 5){
        BatCell.Bat_Volt = data->Bat_1_Volt;
        BatCell.Bat_temp = data->Bat_1_temp;
        BatCell.Bat_curr = data->Bat_1_curr;
        BatCell.Bat_Soc  = data->Bat_1_Soc;
        BatCell.Bat_comm_state = data->Bat_1_comm_state;
        BatCell.Bat_cell_num   = data->Bat_1_cell_num;
        BatCellsState.BatCells.push_back(BatCell);
    }
    if(data->Bat_2_Volt > 5){
        BatCell.Bat_Volt = data->Bat_2_Volt;
        BatCell.Bat_temp = data->Bat_2_temp;
        BatCell.Bat_curr = data->Bat_2_curr;
        BatCell.Bat_Soc  = data->Bat_2_Soc;
        BatCell.Bat_comm_state = data->Bat_2_comm_state;
        BatCell.Bat_cell_num   = data->Bat_2_cell_num;
        BatCellsState.BatCells.push_back(BatCell);
    }
    if(data->Bat_3_Volt > 5){
        BatCell.Bat_Volt = data->Bat_3_Volt;
        BatCell.Bat_temp = data->Bat_3_temp;
        BatCell.Bat_curr = data->Bat_3_curr;
        BatCell.Bat_Soc  = data->Bat_3_Soc;
        BatCell.Bat_comm_state = data->Bat_3_comm_state;
        BatCell.Bat_cell_num   = data->Bat_3_cell_num;
        BatCellsState.BatCells.push_back(BatCell);
    }
    //--
    BatCellsState.Bat_backup_Volt= data->Bat_backup_Volt;
    BatCellsState.Bus_Volt       = data->Bus_Volt;
    BatCellsState.Charger_Volt   = data->Charger_Volt;

    BatCellsState.Board_Temp.clear();
    BatCellsState.Board_Temp.push_back(data->Board_temp1);
    BatCellsState.Board_Temp.push_back(data->Board_temp2);
    BatCellsState.Board_Temp.push_back(data->Board_temp3);
    BatCellsState.Board_Temp.push_back(data->Board_temp4);
    BatCellsState.Board_Temp.push_back(data->Board_temp5);
    BatCellsState.Board_Temp.push_back(data->Board_temp6);
    BatCellsState.Board_Temp.push_back(data->Board_temp7);
    BatCellsState.Board_Temp.push_back(data->Board_temp8);
    BatCellsState.Board_Curr.clear();
    BatCellsState.Board_Curr.push_back(data->Board_curr1);
    BatCellsState.Board_Curr.push_back(data->Board_curr2);
    BatCellsState.Board_Curr.push_back(data->Board_curr3);
    BatCellsState.Board_Curr.push_back(data->Board_curr4);
    BatCellsState.Board_Curr.push_back(data->Board_curr5);
    BatCellsState.Board_Curr.push_back(data->Board_curr6);
    BatCellsState.Board_Curr.push_back(data->Board_curr7);
    BatCellsState.Board_Curr.push_back(data->Board_curr8);
    //--
    BatCellsState.charge_reverse = data->charge_reverse;
    BatCellsState.v_leakage = data->v_leakage;
    BatCellsState.ChargeStatus = data->ChargeStatus;
    BatCellsState.Lock_status = data->Lock_status;
    BatCellsState.errorInfo = data->errorInfo;
    //充电状态
    BatCellsState.charge_type = 1; //0:undefined 1:wireless_charge 2:wired_charge
    BatCellsState.charge_voltage = data->wireless_voltage;
    BatCellsState.charge_current = data->wireless_current;
    BatCellsState.charge_reserve = data->wireless_reserve;
    BatCellsState.charge_state = data->wireless_state;
    BatCellsState.charge_temp = data->wireless_temp;
    //对充电标志位进行窗口滤波
    static std::deque<uint8_t> vehicle_charger_queue_;
    vehicle_charger_queue_.push_back(data->wireless_changer);
    if(vehicle_charger_queue_.size() > 3)
        vehicle_charger_queue_.pop_front();
    uint8_t wireless_changer = 1;
    if(vehicle_charger_queue_[0] == 0 && vehicle_charger_queue_[1] == 0 && vehicle_charger_queue_[2] == 0 )
        wireless_changer = 0;
    BatCellsState.charge_charger = wireless_changer;

    batcell_pub.publish(BatCellsState);

    Info("R_WC_IS: "<<(int)batteryState.wireless_install_state
        <<" R_WC_VT: "<< (int)data->wireless_voltage
        <<" R_WC_CU: "<< (int)data->wireless_current
        <<" R_WC_RE: "<< (int)data->wireless_reserve
        <<" R_WC_ST: "<< (int)data->wireless_state
        <<" R_WC_TP: "<< (int)data->wireless_temp
        <<" R_WC_CH: " << (int)data->wireless_changer
        <<" R_WC_CH_RE: " << (int)wireless_changer
        <<" R_BA_VT: "<<batteryState.voltage_all
        <<" R_BA_PW: "<<(int)batteryState.power
        <<" R_BA_1_VO: "<<data->Bat_1_Volt
        <<" R_BA_2_VO: "<<data->Bat_2_Volt
        <<" R_BA_3_VO: "<<data->Bat_3_Volt
        <<" R_BA_1_SOC: "<<(int)data->Bat_1_Soc
        <<" R_BA_2_SOC: "<<(int)data->Bat_2_Soc
        <<" R_BA_3_SOC: "<<(int)data->Bat_3_Soc
        <<" R_BA_SOC: "<<(int)BatCellsState.Soc_All
        <<" R_BK_VOTWO: "<<(int)data->Bat_backup_Volt
        <<" R_ER_IFTWO: "<<(int)data->errorInfo
        <<" R_BA_VO: "<<BatCellsState.Volt_All
        <<" R_BK_VO: "<<(int)data->Bat_backup_Volt
        <<" R_ER_IF: "<<(int)data->errorInfo
    );
}

//************************************** 接收到电机驱动状态后发布话题<无用>
void pub_driverstatus(recv_from_driver_status_type *data)
{
    if(!data){
        return;
    }
}

//************************************** 接收到超声波数据后发布话题
void pub_ultrasonicdata(recv_from_ultrasonic_data *data)
{
    if(!data){
        return;
    }
    const float min_range=0.12;
    const float max_range=2.5;
    rangeData.header.stamp = rangeDatas.header.stamp = ros::Time::now();
    rangeData.radiation_type = rangeDatas.radiation_type = 0;//ult
    rangeData.min_range = rangeDatas.min_range = min_range;
    rangeData.max_range = rangeDatas.max_range = max_range;
    rangeData.field_of_view = 24*M_PI/180.0f;
    rangeDatas.range.clear();
    for(int i=0;i<max_type_ult;i++){
        float ult_data = (float)data->ult_data[i]/1000.f;
        rangeDatas.range.push_back(ult_data);
        rangeData.range = ult_data;
        rangeData.header.frame_id = ult_name[i];
        range_pub[i].publish(rangeData);
    }
    ranges_pub.publish(rangeDatas);
}

//************************************** 接收到超声波数据后发布话题--lin通信
void pub_ultrasonicdata_lin(recv_from_ultrasonic_data_lin *data)
{
    if(!data){
        return;
    }
    Info("R_LIN_ULT: "<<data->ult_data[0]
    <<" "<<data->ult_data[1]
    <<" "<<data->ult_data[2]
    <<" "<<data->ult_data[3]
    <<" "<<data->ult_data[4]
    <<" "<<data->ult_data[5]
    <<" "<<data->ult_data[6]
    <<" "<<data->ult_data[7]
    <<" "<<data->ult_data[8]
    <<" "<<data->ult_data[9]
    <<" "<<data->ult_data[10] 
    <<" R_LIN_SQE1: "<<data->sqe1
    <<" R_LIN_SQE2: "<<data->sqe1
    )
    //超声波处理方式变更,将源数据发布出去,由另一个节点来专门处理
    std_msgs::UInt16MultiArray lin_ult_data_msgs;
    int data_size = sizeof(data->ult_data) / sizeof(data->ult_data[0]);
    //根据指令将数据过滤一遍，指定关闭的探头数据发2500，探头顺序对应
    //int8_t linult_order_to_control[] = {2, -1, 1, -1, -1, 7, 6, -1, 10, 9, 4, 3, 8, 5};
    for(int i = 0; i < lin_max_type_ult; i++){
        if((vehicle_ult_cmd & 0x00000001 << i) == 0){
            int8_t ult_order = linult_order_to_control[i]; //取得探头编号
            if(ult_order > 0 && ult_order <= data_size){
                 data->ult_data[ult_order -1] = 2500; //对应的探头数据设置为2500
            }
        }
    }

    //数据放入消息
    for(int i = 0; i <  data_size; i++){
        lin_ult_data_msgs.data.push_back(data->ult_data[i]);
    }
    // for(int i = 0; i <  data_size; i++){
    //     lin_ult_data_msgs.data.push_back(data->ult_data[i]);
    //     std::cout<<"cnt"<<(int)i+1<<":  "<<(int)data->ult_data[i]<<std::endl;
    // }
    // std::cout<<"---------------------"<<std::endl;
    lin_ult_data_pub.publish(lin_ult_data_msgs);
}
//**************************************对比两个输入的vector,返回对比结果
bool less_equa_compare(std::vector<int> vec1,std::vector<int> vec2)
{
    if(vec1[0] < vec2[0]){
        return true;
    }
    if((vec1[0] == vec2[0]) && (vec1[1] < vec2[1])){
        return true;
    }
    if((vec1[0] == vec2[0]) && (vec1[1] == vec2[1]) && (vec1[2] <= vec2[2]) ){
        return true;
    }
    return false;
}
//************************************** 提取版本号中的数字 并和版本限制进行对比，如果超出版本限制，导航的运动控制命令不会下发
void getNumInString(std::string str)
{
    int str_len;
    int start_itr;
    std::vector<int> version;

    std::string num_str;

    str_len = str.length();
    for(int i = 0;i<str_len;i++){
        if (str[i]== 'V' ){
            start_itr = i +1;
            break;
        }
    }
    for(int i = start_itr;i<str_len;i++)
    {
        std::string temp_str = "";
        temp_str = str[i];
        if(str[i] >= '0' && str[i] <= '9')
            num_str.push_back(str[i]);
        else if (str[i] == '.'){
            version.push_back(atoi(num_str.c_str()));
            num_str.clear();
            continue;}
        else
            break;   
    }
    version.push_back(atoi(num_str.c_str()));
    num_str.clear();
    // printf("version0: %d\n",version[0]);
    // printf("version1: %d\n",version[1]);
    // printf("version2: %d\n",version[2]);
    // compare with the min_version
    
    std::vector<int> max_version;
    std::vector<int> min_version;
    max_version.push_back(max_control_board_version_head_);
    max_version.push_back(max_control_board_version_mid_);
    max_version.push_back(max_control_board_version_end_);
    min_version.push_back(min_control_board_version_head_);
    min_version.push_back(min_control_board_version_mid_);
    min_version.push_back(min_control_board_version_end_);

    bool less_equa_than_max_version = less_equa_compare(version,max_version);
    bool more_equa_than_min_version = less_equa_compare(min_version,version);
   
    if(less_equa_than_max_version && more_equa_than_min_version)
    {
        control_version_right = 0;
    }
    if(!less_equa_than_max_version && more_equa_than_min_version)
    {
        control_version_right = 1;
    }
    if(less_equa_than_max_version && !more_equa_than_min_version)
    {
        control_version_right = -1;
    }
    firmwareVersionCheck.data = control_version_right;
    firmware_version_status_pub.publish(firmwareVersionCheck);

}
//************************************** 接收到固件版本号后发布话题
void pub_firmwareversion(recv_from_firmware_version_type *data)
{
    if(!data){
        return;
    }    
    if(data->app_ver != NULL && data->app_ver != " ")
	{
	    get_version_flag = true;
	}
    //发送收到的原始数据
    cti_fpga_serial::firmwareinfo  fm_info_msg;
    fm_info_msg.src = data->upd_info.src;
    fm_info_msg.dest = data->upd_info.dest;
    fm_info_msg.run_area = data->run_area;
    fm_info_msg.update_status = data->update_status;
    fm_info_msg.boot_ver = data->boot_ver;
    fm_info_msg.app_ver = data->app_ver;
    fm_info_msg.update_lib_ver = data->update_lib_ver;
    firmware_version_check_pub.publish(fm_info_msg);

    //判断收到的状态：
    Info("R_210_SRC: "<<data->upd_info.src
    <<"R_210_DEST: "<<data->upd_info.dest
    <<"R_210_BOOT: "<<data->boot_ver
    <<"R_210_APP: "<<data->app_ver
    <<"R_210_LIB: "<<data->update_lib_ver);

    fw_ver_check_state.minor_id_recv = data->upd_info.src;
    fw_ver_check_state.recv_210_src = data->upd_info.src;
    fw_ver_check_state.recv_210_dest = data->upd_info.dest;
    fw_ver_check_state.recv_210_run_area = data->run_area;
    fw_ver_check_state.recv_210_update_status = data->update_status;
    fw_ver_check_state.recv_210_boot_ver = data->boot_ver;
    fw_ver_check_state.recv_210_app_ver = data->app_ver;
    fw_ver_check_state.recv_210_update_lib_ver = data->update_lib_ver;
    if(fw_ver_check_state.state != CH_WAIT_210_RESPOND){
        fw_ver_check_state.state_msg = fw_ver_check_msg_map.at(CH_RECV_210_IN_WRONG_STATE);
        fw_ver_check_state.state_msg = fw_ver_check_state.state_msg + "in state: ";
        fw_ver_check_state.state_msg = fw_ver_check_state.state_msg + fw_ver_check_msg_map.at(fw_ver_check_state.state);
        fw_ver_check_state.state = CH_RECV_210_IN_WRONG_STATE;
        return;
    }
    if(fw_ver_check_state.minor_id_recv == fw_ver_check_state.minor_id_send){
        fw_ver_check_state.state = CH_210_RESPOND_SUCCESS;
        fw_ver_check_state.state_msg = fw_ver_check_msg_map.at(CH_210_RESPOND_SUCCESS);
    }else{
        fw_ver_check_state.state = CH_210_RESPOND_ERROR;
        fw_ver_check_state.state_msg = fw_ver_check_msg_map.at(CH_210_RESPOND_ERROR);
    }

    //发布版本
    switch (data->upd_info.src)
    {
    case MODULE_DEBUG_BOARD:
        RobotVersionDisplay.debugInterfaceVersion = data->app_ver;
        break;       
    case MODULE_MOVE_CONTROL_BOARD:
        {
        RobotVersionDisplay.operationControlVersion = data->app_ver;
        std::string version_info;
        version_info = data->app_ver;
        getNumInString(version_info);
        break;
        }
    case MODULE_POWER_INTEGRATE_BOARD:
        RobotVersionDisplay.batteryTandemVersion = data->app_ver;
        break;
    case MODULE_MOTOR_DRIVER_FRONT_LEFT:
        RobotVersionDisplay.leftFrontDiverVersion = data->app_ver;
        break;
    case MODULE_MOTOR_DRIVER_FRONT_RIGHT:
        RobotVersionDisplay.rightFrontDiverVersion = data->app_ver;
        break;        
    case MODULE_MOTOR_DRIVER_BACK_LEFT:
        RobotVersionDisplay.leftBackDiverVersion = data->app_ver;
        break;      
    case MODULE_MOTOR_DRIVER_BACK_RIGHT:
        RobotVersionDisplay.rightBackDiverVersion = data->app_ver;
        break;            
    case MODULE_MOTOR_TURN_FRONT:
        RobotVersionDisplay.forwardSwayDiverVersion = data->app_ver;
        break;            
    case MODULE_MOTOR_TURN_BACK:
        RobotVersionDisplay.backwardSwayDiverVersion = data->app_ver;
        break;            
    case MODULE_MOTOR_BRAKE_FRONT:
        RobotVersionDisplay.frontBrakeDiverVersion = data->app_ver;
        break;    
    case MODULE_MOTOR_BREAK_BACK:
        RobotVersionDisplay.backBrakeDiverVersion = data->app_ver;
        break;
    case MODULE_LIGHT_START:
        RobotVersionDisplay.lightControlVersion = data->app_ver;
        break;
    case MODULE_ULTRASONIC_1:
        RobotVersionDisplay.ultrasonicVersion[0] = data->app_ver;
        break;
    case MODULE_ULTRASONIC_2:
        RobotVersionDisplay.ultrasonicVersion[1] = data->app_ver;
        break;
    case MODULE_ULTRASONIC_3:
        RobotVersionDisplay.ultrasonicVersion[2] = data->app_ver;
        break;
    case MODULE_ULTRASONIC_4:
        RobotVersionDisplay.ultrasonicVersion[3] = data->app_ver;
        break;
    case MODULE_ULTRASONIC_5:
        RobotVersionDisplay.ultrasonicVersion[4] = data->app_ver;
        break;
    case MODULE_ULTRASONIC_6:
        RobotVersionDisplay.ultrasonicVersion[5] = data->app_ver;
        break;
    case MODULE_ULTRASONIC_7:
        RobotVersionDisplay.ultrasonicVersion[6] = data->app_ver;
        break;
    case MODULE_ULTRASONIC_8:
        RobotVersionDisplay.ultrasonicVersion[7] = data->app_ver;
        break;
    case MODULE_ULTRASONIC_9:
        RobotVersionDisplay.ultrasonicVersion[8] = data->app_ver;
        break;
    case MODULE_ULTRASONIC_10:
        RobotVersionDisplay.ultrasonicVersion[9] = data->app_ver;
        break;
    case MODULE_ULTRASONIC_11:
        RobotVersionDisplay.ultrasonicVersion[10] = data->app_ver;
        break;
    case MODULE_ULTRASONIC_12:
        RobotVersionDisplay.ultrasonicVersion[11] = data->app_ver;
        break;
    case MODULE_ULTRASONIC_13:
        RobotVersionDisplay.ultrasonicVersion[12] = data->app_ver;
        break;
    case MODULE_ULTRASONIC_14:
        RobotVersionDisplay.ultrasonicVersion[13] = data->app_ver;
        break;
    case MODULE_ULTRASONIC_15:
        RobotVersionDisplay.ultrasonicVersion[14] = data->app_ver;
        break;
    default:
        break;
    }
    firmvion_pub.publish(RobotVersionDisplay);
}

//************************************** 接收到命令应答之后计数
void compar_id_recv_send(recv_from_cmd_answer_type *data)
{
    if (!data)
    {
        return;
    }
        cmd_send_map.clear();
        cmd_answer_success_cnt++;
}

//************************************** rfid成组发布缓存容器push_back
void oldTabstate_Pushback(cti_msgs::TabState *tabstate_ptr,uint8_t id)
{
    switch(id)
    {
        case 1:
        oldtabstate_rfid1.push_back(*tabstate_ptr);
        break;
        case 2:
        oldtabstate_rfid2.push_back(*tabstate_ptr);
        break;
        case 3:
        tabstate_ptr->id = 2;
        oldtabstate_rfid3.push_back(*tabstate_ptr);
        break;
        case 4:
        tabstate_ptr->id = 3;
        oldtabstate_rfid4.push_back(*tabstate_ptr);
        break;
        default:
        break;
    }  
}

//************************************** rfid单独发布缓存容器push_back
void rfid_single_Pushback(cti_msgs::TabState *tabstate_ptr,uint8_t id)
{
    
    switch(id)
    {
        case 1:
        rfid_single.states.push_back(*tabstate_ptr);
        break;
        case 2:
        rfid_single.states.push_back(*tabstate_ptr);
        break;
        case 3:
        //tabstate_ptr->id = 2;
        rfid_single.states.push_back(*tabstate_ptr);
        break;
        case 4:
        //tabstate_ptr->id = 3;
        rfid_single.states.push_back(*tabstate_ptr);
        break;
        default:
        break;
    } 
}

//************************************** rfid成组发布缓存容器clear
void oldTabstate_clear(uint8_t id)
{
    switch(id)
    {
        case 1:
        oldtabstate_rfid1.clear();
        break;
        case 2:
        oldtabstate_rfid2.clear();
        break;
        case 3:
        oldtabstate_rfid3.clear();
        break;
        case 4:
        oldtabstate_rfid4.clear();
        break;
        default:
        break;
    }           
}

//************************************** rfid接收超时检测
void recv_rfid_timeout(uint8_t id)
{
     switch(id)
    {
        case 1:
        oldtime_rfid1 = ros::Time::now().toSec();
        break;
        case 2:
        oldtime_rfid2 = ros::Time::now().toSec();
        break;
        case 3:
        oldtime_rfid3 = ros::Time::now().toSec();
        break;
        case 4:
        oldtime_rfid4 = ros::Time::now().toSec();
        break;
        default:
        break;
    }  
	double nowtime_rfid = ros::Time::now().toSec(); 
    if (RFIDTIMEOUT_DUR < (nowtime_rfid - oldtime_rfid1))
    {
        recv_rfid1_timeout = true;
		oldtabstate_rfid1.clear();
        cti_msgs::TabState tabstate;
		tabstate.id = 1;
        tabstate.status = 140;//140 means recv timeout!
		tabstate.message = "";
        oldTabstate_Pushback(&tabstate,tabstate.id);
		oldtime_rfid1 = ros::Time::now().toSec();
        //Info("RECV RFID: module_id_1 recv timeout!");   
	}
	else
	{
		recv_rfid1_timeout = false;
	}
    if (RFIDTIMEOUT_DUR < (nowtime_rfid - oldtime_rfid2))
    {
		recv_rfid2_timeout = true;                
		oldtabstate_rfid2.clear();
        cti_msgs::TabState tabstate;
		tabstate.id = 2;
        tabstate.status = 140;//140 means recv timeout!
		tabstate.message = "";
        oldTabstate_Pushback(&tabstate,tabstate.id);
        oldtime_rfid2 = ros::Time::now().toSec();
        //Info("RECV RFID: module_id_2 recv timeout!");
	} 
	else
	{
		recv_rfid2_timeout = false;
	}   
    if (RFIDTIMEOUT_DUR < (nowtime_rfid - oldtime_rfid3))
    {
        recv_rfid3_timeout = true; 
		oldtabstate_rfid3.clear();
        cti_msgs::TabState tabstate;
		tabstate.id = 3;
        tabstate.status = 140;//140 means recv timeout!
		tabstate.message = "";
        oldTabstate_Pushback(&tabstate,tabstate.id);
        oldtime_rfid3 = ros::Time::now().toSec();
        //Info("RECV RFID: module_id_3 recv timeout!");
	}  
	else
	{
		recv_rfid3_timeout = false;
	}
    if (RFIDTIMEOUT_DUR < (nowtime_rfid - oldtime_rfid4))
    {
        recv_rfid4_timeout = true; 
		oldtabstate_rfid4.clear();
        cti_msgs::TabState tabstate;
		tabstate.id = 4;
        tabstate.status = 140;//140 means recv timeout!
		tabstate.message = "";
        oldTabstate_Pushback(&tabstate,tabstate.id);
        oldtime_rfid4 = ros::Time::now().toSec();
        //Info("RECV RFID: module_id_4 recv timeout!");
	}  
	else
	{
		recv_rfid4_timeout = false;
	}      
}

//************************************** rfid接收到信息后发布
void pub_rfidinfo(recv_from_rfid_info_type *data)
{
    if(!data)
    {
        return;
       // printf("error!reveive empty data!\n");
    }  

    rfid_single.header.frame_id="rfid_single";
    rfid_all.header.frame_id="rfid_all";
    cti_msgs::TabState tabstate;
    tabstate.id = data->module_id;
    tabstate.status=data->status;
    tabstate.name = "rfid";
    oldTabstate_clear(data->module_id);
    recv_rfid_timeout(data->module_id);
    if(data->status == 1)
    {
        tabstate.status = data->read_num;
        //std::cout<<"NO RFID"<<tabstate.message<<std_endl;
        rfid_single_Pushback(&tabstate,data->module_id);
        boxrfid_pub_single.publish(rfid_single);
        rfid_single.states.clear();
        oldTabstate_Pushback(&tabstate,data->module_id);
    }
    if(data->status == 0)
    {
        switch(data->read_num)
        {
        case 1:
             {
            tabstate.message = hex2string(&(data->data[2]),8);
            rfid_single_Pushback(&tabstate,data->module_id);
            //std:: cout<<"GET RFID NUM 1: "<<tabstate.message<<std::endl;
            oldTabstate_Pushback(&tabstate,data->module_id);
            break;
             }
        case 2:
             {
            tabstate.message = hex2string(&(data->data[2]),8);
            rfid_single_Pushback(&tabstate,data->module_id);
            //std:: cout<<"GET RFID NUM 2_1: "<<tabstate.message<<std::endl;
            oldTabstate_Pushback(&tabstate,data->module_id);
            tabstate.message = hex2string(&(data->data[12]),8);
            rfid_single_Pushback(&tabstate,data->module_id);
            oldTabstate_Pushback(&tabstate,data->module_id);
            //std:: cout<<"GET RFID NUM 2_2: "<<tabstate.message<<std::endl;
            break;
             }
        case 3:
             {
            tabstate.message = hex2string(&(data->data[2]),8);
            rfid_single_Pushback(&tabstate,data->module_id);
            oldTabstate_Pushback(&tabstate,data->module_id);
            tabstate.message = hex2string(&(data->data[12]),8);
	        rfid_single_Pushback(&tabstate,data->module_id);
            oldTabstate_Pushback(&tabstate,data->module_id);
            tabstate.message = hex2string(&(data->data[22]),8);
	        rfid_single_Pushback(&tabstate,data->module_id);
            oldTabstate_Pushback(&tabstate,data->module_id);
            break;
             }
        case 4:
             {
            tabstate.message = hex2string(&(data->data[2]),8);
	        rfid_single_Pushback(&tabstate,data->module_id);
            oldTabstate_Pushback(&tabstate,data->module_id);
            tabstate.message = hex2string(&(data->data[12]),8);
	        rfid_single_Pushback(&tabstate,data->module_id);
            oldTabstate_Pushback(&tabstate,data->module_id);
            tabstate.message = hex2string(&(data->data[22]),8);
	        rfid_single_Pushback(&tabstate,data->module_id);
            oldTabstate_Pushback(&tabstate,data->module_id);
            tabstate.message = hex2string(&(data->data[32]),8);
	        rfid_single_Pushback(&tabstate,data->module_id);
            oldTabstate_Pushback(&tabstate,data->module_id);
            break;
             }
        default:
            break;     
        }
    rfid_single.header.stamp = ros::Time::now();
    rfid_single.header.frame_id = "/fpga_serial/rfid_single";
    boxrfid_pub_single.publish(rfid_single);
    rfid_single.states.clear();
    }
    if(4 ==data->module_id || recv_rfid4_timeout == true)
    {
    rfid_all.states.clear();
    rfid_all.header.stamp = ros::Time::now();
    rfid_all.header.frame_id = "/fpga_serial/rfid_all";
    rfid_all.states.insert(rfid_all.states.end(),oldtabstate_rfid1.begin(),oldtabstate_rfid1.end());
    rfid_all.states.insert(rfid_all.states.end(),oldtabstate_rfid2.begin(),oldtabstate_rfid2.end());
    rfid_all.states.insert(rfid_all.states.end(),oldtabstate_rfid3.begin(),oldtabstate_rfid3.end()); 
    rfid_all.states.insert(rfid_all.states.end(),oldtabstate_rfid4.begin(),oldtabstate_rfid4.end()); 
    boxrfid_pub_all.publish(rfid_all);
    rfid_all.states.clear();
    }  
    std::string log_data = hex2string(&(data->data[2]),8); 
    /*
    Info("RECV_RFID:" << " module_id:" << (int)data->module_id << " data_length:" << (int)data->data_length << " status:" << (int)data->status 
    << " read_num:" << int(data->read_num)<< " data:" << log_data );   
    */
}
//************************************** 接收到底盘错误码发布
void pub_chassiserror(recv_chassis_error_report_type* data)
{
        if(!data)
    {
        return;
    }
    std_msgs::UInt32MultiArray chassis_error;
    chassis_error.data.push_back((uint32_t)data->module_type);
    chassis_error.data.push_back(data->module_error_code);
    chassis_error.data.push_back((uint32_t)data->module_error_level);
    chassis_error_pub.publish(chassis_error);

    module_type_global = data->module_type;
    module_error_code_global = data->module_error_code;
    module_error_level_global = data->module_error_level;

}
//*************************************Timer5定时器，发布地盘错误码
void timer5Callback(const ros::TimerEvent& event)
{
    std_msgs::UInt32MultiArray chassis_error;
  
    if(0 == module_type_global)
    {
        chassis_error.data.push_back(0);
    }
    else
    {
        chassis_error.data.push_back((uint32_t)module_type_global);
        module_type_global = 0;
    }

    if(0 == module_error_code_global)
    {
        chassis_error.data.push_back(0);
    }
    else
    {
        chassis_error.data.push_back(module_error_code_global);
        module_error_code_global = 0;
    }

    if(0 == module_error_level_global)
    {
        chassis_error.data.push_back(0);
    }
    else
    {
        chassis_error.data.push_back((uint32_t)module_error_level_global);
        module_error_level_global = 0;
    }
    chassis_error_pub.publish(chassis_error);    
}
//************************************** 接收到底盘导航重要信息
void pub_navigationlog(recv_navigation_log_status_type* data)
{
    if(!data)
    {
        return;
    }
    navigation_log.header.stamp = ros::Time::now();
    navigation_log.header.frame_id = "/fpga_serial/navigationlog";
    navigation_log.liner_speed = data->liner_speed;
    navigation_log.turn_angle = data->turn_angle;
    navigation_log.break_torque = data->break_torque; //刹车力度，直接向驱动下发力矩值
    navigation_log.enable_flag = data->enable_flag;
    navigation_log.actual_body_linear_vel_x = data->actual_body_linear_vel_x;
    navigation_log.actual_speed_base_on_left_front_wheel = data->actual_speed_base_on_left_front_wheel; //左前轮折算的车体中心速度
    navigation_log.actual_speed_base_on_right_front_wheel = data->actual_speed_base_on_right_front_wheel;
    navigation_log.actual_speed_base_on_left_rear_wheel = data->actual_speed_base_on_left_rear_wheel;
    navigation_log.actual_speed_base_on_right_rear_wheel = data->actual_speed_base_on_right_rear_wheel; 
    navigation_log.actual_turn_front_angle = data->actual_turn_front_angle;//unit:degree 前后转向电机的角度
    navigation_log.actual_turn_rear_angle = data->actual_turn_rear_angle;//unit:degree
    for(int i =0 ; i < (sizeof(data->set_torque)/sizeof(int16_t));i++)
    {
        navigation_log.set_torque.push_back(data->set_torque[i]);
    }
    for(int i =0 ; i < (sizeof(data->now_encoder)/sizeof(uint16_t));i++)
    {
        navigation_log.now_encoder.push_back(data->set_torque[i]);
    }
    navigation_log_pub.publish(navigation_log);
    navigation_log.set_torque.clear();
    navigation_log.now_encoder.clear();

    //发布四轮轮速
    std_msgs::Float32MultiArray wheels_speed;
    wheels_speed.data.push_back(data->actual_speed_base_on_left_front_wheel);  //左前轮速度
    wheels_speed.data.push_back(data->actual_speed_base_on_right_front_wheel); //右前轮速度
    wheels_speed.data.push_back(data->actual_speed_base_on_left_rear_wheel);   //左后轮速度
    wheels_speed.data.push_back(data->actual_speed_base_on_right_rear_wheel);  //右后轮速度
    wheel_speed_pub.publish(wheels_speed);
}
//************************************** 接收sd卡格式化结果后发布话题
void pub_formatsdcard(send_format_sd_card_cmd_type* data)
{
    if(!data)
    {
        return;
    }
    formatsdcard_result.data = data->portIndex;
    formatsdcard_pub.publish(formatsdcard_result);
}
//************************************** 经纬度数据转化,度.分->度.度
double gps_data_trans(double data){
    double data_temp = data;;
    if(data < 0){
        data_temp = -data_temp;
    }
    int int_part = (int)data_temp;
    double dec_part = data_temp - int_part;
    dec_part = dec_part * 100 / 60.0;
    double ret = dec_part + int_part;
    if(data < 0){
        ret = -ret;
        return ret;
    }else{
        return ret;
    }
}
//************************************** 接收gps数据并发布
void pub_gps(recv_gps_data_type* data)
{
    if(!data)
    {
        return;
    }
    gps_data.header.stamp = ros::Time::now();
    gps_data.header.frame_id = "/chassis_gps";
    //gps utc time
    gps_data.utc_seconds = data->utc_seconds;     //utc时间,00:00:00至今的秒数,当前未使用

    //position
    switch (data->position_stat) //位置解状态,NONE=无定位 SINGLE=单点定位 PSFDIFF=伪距差分 NARROW_FLOAT=浮点解 NARROW_INT=固定解
    {
        case 0:
        case 48:
        gps_data.position_stat = "NONE";
        break;
        case 49:
        gps_data.position_stat = "SINGLE";
        break;
        case 50:
        gps_data.position_stat = "PSFDIFF";
        break;
        case 51:
        gps_data.position_stat = "NONE";
        break;
        case 52:
        gps_data.position_stat = "NONE";
        break;
        case 53:
        gps_data.position_stat = "NARROW_FLOAT";
        break;
        case 54:
        gps_data.position_stat = "NARROW_INT";
        break;
        default:
        gps_data.position_stat = "UNDEFINED";
        break;
    }  
    gps_data.lat = (data->lat)/100;        //负数表示南半球,单位(度)
    gps_data.lat = gps_data_trans(gps_data.lat);
    gps_data.lon = (data->lon)/100;          //负数表示西半球,单位(度)
    gps_data.lon = gps_data_trans(gps_data.lon);
    gps_data.alt = data->alt;           //WGS84 椭球高,若使用海拔高,则需根据undulation计算转换.
    gps_data.lat_err = data->lat_err;         //纬度标准差,单位(m)
    gps_data.lon_err = data->lon_err;       // 经度标准差,单位(m)
    gps_data.alt_err = data->alt_err;        //高程标准差,单位(m)
    gps_data.diff_age = data->diff_age;        // 差分龄,单位(s)
    gps_data.undulation = data->undulation;     // 海拔高 = alt - undulation
    gps_data.sats_tracked = data->sats_tracked;   // 跟踪到的卫星数
    gps_data.sats_used = data->sats_used;      // 解算中使用的卫星数

    // angle 主天线(moving)到副天线(heading)所形成的向量,真北即经线北向
        //position
    switch (data->heading_stat)  // 定向解状态,仅 SOL_COMPUTED 时表示定位成功
    {
        case 0:
        gps_data.heading_stat = "NONE";
        break;
        case 1:
        gps_data.heading_stat = "SOL_COMPUTED";
        break;
        default:
        gps_data.heading_stat = "UNDEFINED";
        break;
    }
    gps_data.heading = data->heading;        // 航向角(度),以为真北为起点,顺时针 0.0 - 359.99
    gps_data.pitch = data->pitch;         // 俯仰角(度),水平为0°,±90°
    gps_data.heading_err = data->heading_err;    //航向角标准差,单位(度)
    gps_data.pitch_err = data->pitch_err;      //俯仰角标准差,单位(度)
    gps_data.baselineLen = data->baselineLen;    // 两天线基线长

    //velocity
    switch (data->velocity_stat)  // 速度解状态,仅 SOL_COMPUTED 时表示定位成功
    {
        case 0:
        gps_data.velocity_stat = "NONE";
        break;
        case 1:
        gps_data.velocity_stat = "SOL_COMPUTED";
        break;
        default:
        gps_data.velocity_stat = "UNDEFINED";
        break;
    }  
    gps_data.speed_north  = (data->speed_north)/100;  // 东北天系下各速度分量,单位(m/s)
    gps_data.speed_east = (data->speed_east)/100;
    gps_data.speed_up = (data->speed_up)/100;
    gps_data.latency = data->latency;     // 速度延迟,单位(s)

    gps_pub.publish(gps_data);
}
//************************************** 定位状态命令话题回调函数
void localizerState_Callback(const cti_msgs::RobotLocalizerState::ConstPtr &msg)
{
    if(localization_limit){
        robotlocalizerstate_global = msg->id;
    }else{
        robotlocalizerstate_global = 2;
    }
}
 //************************************** 发送细时间同步命令
void send_fine_sync_cmd()
{
    //获取到微秒级别
    struct timeval tv;
    struct timezone tz;
    gettimeofday(&tv,&tz);

    time_sync_fine_msg_type timenow_cmd;

    //printf("tv_usec:%d\n",tv.tv_usec);
    timenow_cmd.sec = tv.tv_sec % 60;
    //printf("send_fine_sec:%d\n",timenow_cmd.sec);
    timenow_cmd.millisencond = (tv.tv_usec)/1000;
    //printf("send_fine_millisec:%d\n",timenow_cmd.millisencond);
    timenow_cmd.microsecondsec = (tv.tv_usec)%1000;
    //printf("send_fine_microsec:%d\n",timenow_cmd.microsecondsec);
    construct_serial_frame_ex(&user_frame, TIME_SYNC_FINE_CMD, sizeof(timenow_cmd), &timenow_cmd);
    // serial_frame_include_id_type user_frame_include_id_0;
    // user_frame_include_id_0.id = 0;
    // user_frame_include_id_0.cmd = TIME_SYNC_FINE_CMD;
    // user_frame_include_id_0.need_id = 0;
    // user_frame_include_id_0.frame = user_frame;
    // pushData(&user_frame_include_id_0);
    pushData(construct_frame_include_id(0,TIME_SYNC_FINE_CMD,0,user_frame));
    Info("S_SYNC_FINE: "<<1);
    //printf("________________________________________\n");
    //printf("Send_fine_sync_req::33>>>>>>>>>>>>>>>>>>>>>>\n");
}  
 //************************************** 发送延时时间同步命令
void send_delay_sync_cmd()
{
    //获取到微秒级别
    struct timeval tv;
    struct timezone tz;
    gettimeofday(&tv,&tz);

    time_sync_fine_msg_type timenow_cmd;

    timenow_cmd.sec = tv.tv_sec % 60;
    timenow_cmd.millisencond = (tv.tv_usec)/1000;
    timenow_cmd.microsecondsec = (tv.tv_usec)%1000;
    construct_serial_frame_ex(&user_frame, TIME_SYNC_DELAY_CMD, sizeof(timenow_cmd), &timenow_cmd);
    // serial_frame_include_id_type user_frame_include_id_0;
    // user_frame_include_id_0.id = 0;
    // user_frame_include_id_0.cmd = TIME_SYNC_DELAY_CMD;
    // user_frame_include_id_0.need_id = 0;
    // user_frame_include_id_0.frame = user_frame;
    // pushData(&user_frame_include_id_0);
    pushData(construct_frame_include_id(0,TIME_SYNC_DELAY_CMD,0,user_frame));
    Info("S_SYNC_DELAY: "<<1);
    //printf("Send_delay_sync_req::34>>>>>>>>>>>>>>>>>>>>>>\n");
}
//************************************** 粗时间同步回应处理函数
void check_rough_res(time_sync_rough_msg_type * data)
{
    if(data->year == sync_rough_msg_saved.year && data->mon == sync_rough_msg_saved.mon 
        && data->day == sync_rough_msg_saved.day && data->hour == sync_rough_msg_saved.hour 
        && data->min == sync_rough_msg_saved.min && data->sec == sync_rough_msg_saved.sec)
    {
        //发送细时间同步
        need_send_fine_sync = true;
        //取消粗同步回应超时检测
        check_rough_sync_res_timeoout = false;
        check_rough_sync_res_cnt = 0;
        need_send_rough = false;
       //printf("rough_sync_successed@@@@@@@@@@@@@@@\n");
    }
    else
    {
        need_resend_rough = true;
    }
}

//************************************** 清扫箱状态发布
void pub_dustbin_state(dustbin_to_motion_t *data)
{
    
    cti_msgs::DustbinState dustbin_state_msg;
    dustbin_state_msg.header.stamp = ros::Time::now();
    dustbin_state_msg.header.frame_id = "/chassis_dustbin";

	dustbin_state_msg.water_tank_top= data->water_tank_top;
	dustbin_state_msg.water_tank_bottom= data->water_tank_bottom;
	dustbin_state_msg.dustbin_tail_cover= data->dustbin_tail_cover;
	dustbin_state_msg.dustbin_distance= data->dustbin_distance;
    dustbin_state_msg.motor_status = data->motor_status.data;
    
    for(int i = 0;i<sizeof(data->ultrasonic_distance)/sizeof(uint16_t);i++)
    {
        dustbin_state_msg.ultrasonic_distance.push_back(data->ultrasonic_distance[i]);
    }
    dustbin_state_msg.voltage1= data->voltage1;
    dustbin_state_msg.voltage2= data->voltage2;
    dustbin_state_msg.voltage3= data->voltage3;

    dustbin_state_pub.publish(dustbin_state_msg);


   //使用cti_msgs/DataArray消息类型发布消息
    cti_msgs::DataArray dustVehicleInfos;
    dustVehicleInfos.header.stamp = ros::Time::now();
    dustVehicleInfos.header.frame_id = "sanitation_vehicle";
    //水箱高水位
    cti_msgs::Data dustVehicleInfo;
    dustVehicleInfo.name = "water_tank_top";
    dustVehicleInfo.data = std::to_string(data->water_tank_top);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo); 
    //水箱低水位
    dustVehicleInfo.name = "water_tank_bottom";
    dustVehicleInfo.data = std::to_string(data->water_tank_bottom);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo); 
    //电机状态
    dustVehicleInfo.name = "motor_status";
    dustVehicleInfo.data = std::to_string(data->motor_status.data);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo);
    dust_vehicle_state_info_pub.publish(dustVehicleInfos); //发布新的消息类型


    //发布超声波数据
    if(data->ultrasonic_distance[0] == 0xffff){//新版超声波
        sensor_msgs::Range new_sweeperRangeData;
        int index = (data->ultrasonic_distance[1])-1;
        const float sweeper_min_range=0.12;
        const float sweeper_max_range=2.5;
        new_sweeperRangeData.header.stamp = ros::Time::now();
        new_sweeperRangeData.radiation_type = 0;//ult
        new_sweeperRangeData.min_range = sweeper_min_range;
        new_sweeperRangeData.max_range = msg_max_range;
        new_sweeperRangeData.field_of_view = 24*M_PI/180.0f;
        for(int i=0;i<new_sweeper_max_type_ult;i++){
            float ult_data = (float)data->ultrasonic_distance[i+2]/1000.f;
            new_sweeperRangeData.range = ult_data; 
            new_sweeperRangeData.header.frame_id = new_sweeper_ult_name[index][i];
            new_sweeper_range_pub[index][i].publish(new_sweeperRangeData);
        }
    }
    else{//旧版超声波
        const float sweeper_min_range=0.12;
        const float sweeper_max_range=2.5;
        sweeperRangeData.header.stamp = ros::Time::now();
        sweeperRangeData.radiation_type = 0;//ult
        sweeperRangeData.min_range = sweeper_min_range;
        sweeperRangeData.max_range = msg_max_range;
        sweeperRangeData.field_of_view = 24*M_PI/180.0f;
        for(int i=0;i<sweeper_max_type_ult;i++){
            float ult_data = (float)data->ultrasonic_distance[i]/1000.f;
            sweeperRangeData.range = ult_data; 
            sweeperRangeData.header.frame_id = sweeper_ult_name[i];
            sweeper_range_pub[i].publish(sweeperRangeData);
        }
    }
    Info("R_DBI_ES: "<<(int)data->motor_status.data
    <<" R_DBI_WT: "<<(int)data->water_tank_top
    <<" R_DBI_WB: "<<(int)data->water_tank_bottom
    <<" R_DBI_TC: "<<(int)data->dustbin_tail_cover
    <<" R_DBI_DS: "<<(int)data->dustbin_distance
    <<" R_DBI_DOC: "<<(int)data->recv_dustbin_on_the_car
    );

    //判断清扫车在车上的状态判断
    if(set_dustbin_on_car){ 
        if((data->recv_dustbin_on_the_car == 1 && box_type == 4) || (data->recv_dustbin_on_the_car == 0 && box_type != 4)){
            set_dustbin_on_car = false;
        }
    }
    //-------------------通讯检测------------------
    box_chat_state.module_id = 1;
    box_chat_state.recv_state = true;
    box_chat_state.time_recv = ros::Time::now().toSec(); 
}

//************************************** 环卫车清扫状态发布_旧的,没有加边刷伸缩<暂时保留，防止嵌入式有没有更新的版本>
void pub_clean_state(clean_to_motion_t *data)
{
    
    cti_msgs::DustbinState clean_state_msg;
    clean_state_msg.header.stamp = ros::Time::now();
    clean_state_msg.header.frame_id = "/chassis_dustbin";

	clean_state_msg.water_tank_top= data->water_tank_top;
	clean_state_msg.water_tank_bottom= data->water_tank_bottom;
	clean_state_msg.dustbin_tail_cover= data->dustbin_tail_cover;
	clean_state_msg.dustbin_distance= data->dustbin_distance;
    clean_state_msg.motor_status = data->motor_status.data;
    
    for(int i = 0;i<sizeof(data->ultrasonic_distance)/sizeof(uint16_t);i++)
    {
        clean_state_msg.ultrasonic_distance.push_back(data->ultrasonic_distance[i]);
    }
    clean_state_msg.voltage1= data->voltage1;
    clean_state_msg.voltage2= data->voltage2;
    clean_state_msg.voltage3= data->voltage3;

    dustbin_state_pub.publish(clean_state_msg);
    

    //用cti_msgs::dataArray 类型发布消息
    //使用cti_msgs/DataArray消息类型发布消息
    cti_msgs::DataArray dustVehicleInfos;
    dustVehicleInfos.header.stamp = ros::Time::now();
    dustVehicleInfos.header.frame_id = "sanitation_vehicle";
    //水箱高水位
    cti_msgs::Data dustVehicleInfo;
    dustVehicleInfo.name = "water_tank_top";
    dustVehicleInfo.data = std::to_string(data->water_tank_top);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo); 
    //水箱低水位
    dustVehicleInfo.name = "water_tank_bottom";
    dustVehicleInfo.data = std::to_string(data->water_tank_bottom);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo); 
    //电机状态
    dustVehicleInfo.name = "motor_status";
    dustVehicleInfo.data = std::to_string(data->motor_status.data);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo);
    //挡板状态
    dustVehicleInfo.name = "dam_board_status";
    dustVehicleInfo.data = std::to_string(data->dam_board_status);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo);


     //存储上水位传感器数据
    vehicle_water_tank_top.push_back(data->water_tank_top);
    vehicle_water_tank_bottom.push_back(data->water_tank_bottom);
    if (vehicle_water_tank_top.size() > max_vehicle_water_tank_restore){
        vehicle_water_tank_top.pop_front();
    }
    if (vehicle_water_tank_bottom.size() > max_vehicle_water_tank_restore){
        vehicle_water_tank_bottom.pop_front();
    }
    bool water_full = true;//默认水满
    bool water_empty = true;//默认水空
    for(int i = 0;i < max_vehicle_water_tank_restore -1; i++){
        if(1 == vehicle_water_tank_top[i]){
            water_full = false; //队列中有一个 == 1,就认为水没有满
            break;
        }
    }
    for(int i = 0;i < max_vehicle_water_tank_restore -1; i++){
        if(1 == vehicle_water_tank_bottom[i]){
            water_empty = false; //队列中有一个 == 1,就认为水没有空
            break;
        }
    }
    //计算水量百分比
    if(31 == data->motor_status.data){
        if(false == vehicle_water_status.is_out){
            vehicle_water_status.is_out = true;
            vehicle_water_status.out_pre_time = ros::Time::now().toSec();
        }
    }else if(27 == data->motor_status.data || 0 == data->motor_status.data){
        if(true == vehicle_water_status.is_out){
            vehicle_water_status.is_out = false;
            vehicle_water_status.out_pre_time = ros::Time::now().toSec();
        }
    }
    if(water_full){
        vehicle_water_status.water_percnt_now = 100.0;
    }else if(water_empty){
        vehicle_water_status.water_percnt_now = 0.0;
    }else{
        if(true == vehicle_water_status.is_out){
            double time_now = ros::Time::now().toSec();
            double duration = time_now - vehicle_water_status.out_pre_time;
            vehicle_water_status.water_percnt_now =  vehicle_water_status.water_percnt_now - duration * vehicle_water_status.out_per_sec;
            if (vehicle_water_status.water_percnt_now > 90.0){
                vehicle_water_status.water_percnt_now = 90.0;
            }
            if (vehicle_water_status.water_percnt_now < 10.0){
                vehicle_water_status.water_percnt_now = 10.0;
            }
            vehicle_water_status.out_pre_time = time_now;
        }
    }
    //水量百分比
    dustVehicleInfo.name = "water_percent";
    dustVehicleInfo.data = std::to_string((int)vehicle_water_status.water_percnt_now);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo); 

    std::string modify_file = "echo "+ std::to_string(vehicle_water_status.water_percnt_now) + " > " + config_file_path;
    FILE *fpr = NULL;
    fpr = popen(modify_file.c_str(),"w");
    pclose(fpr);

    //消息发布
    dust_vehicle_state_info_pub.publish(dustVehicleInfos);

    //发布超声波数据
    if(data->ultrasonic_distance[0] == 0xffff){//新版超声波
        sensor_msgs::Range new_sweeperRangeData;
        int index = (data->ultrasonic_distance[1])-1;
        const float sweeper_min_range=0.12;
        const float sweeper_max_range=2.5;
        new_sweeperRangeData.header.stamp = ros::Time::now();
        new_sweeperRangeData.radiation_type = 0;//ult
        new_sweeperRangeData.min_range = sweeper_min_range;
        new_sweeperRangeData.max_range = msg_max_range;
        new_sweeperRangeData.field_of_view = 24*M_PI/180.0f;
        for(int i=0;i<new_sweeper_max_type_ult;i++){
            float ult_data = (float)data->ultrasonic_distance[i+2]/1000.f;
            new_sweeperRangeData.range = ult_data; 
            new_sweeperRangeData.header.frame_id = new_sweeper_ult_name[index][i];
            new_sweeper_range_pub[index][i].publish(new_sweeperRangeData);
        }
    }
    else{//旧版超声波
        const float sweeper_min_range=0.12;
        const float sweeper_max_range=2.5;
        sweeperRangeData.header.stamp = ros::Time::now();
        sweeperRangeData.radiation_type = 0;//ult
        sweeperRangeData.min_range = sweeper_min_range;
        sweeperRangeData.max_range = msg_max_range;
        sweeperRangeData.field_of_view = 24*M_PI/180.0f;
        for(int i=0;i<sweeper_max_type_ult;i++){
            float ult_data = (float)data->ultrasonic_distance[i]/1000.f;
            sweeperRangeData.range = ult_data; 
            sweeperRangeData.header.frame_id = sweeper_ult_name[i];
            sweeper_range_pub[i].publish(sweeperRangeData);
        }
    }
    Info("R_CC_ES: "<<(int)data->motor_status.data
    <<" R_CC_WT: "<<(int)data->water_tank_top
    <<" R_CC_WB: "<<(int)data->water_tank_bottom
    <<" R_CC_TC: "<<(int)data->dustbin_tail_cover
    <<" R_CC_DS: "<<(int)data->dustbin_distance
    <<" R_CC_DOC: "<<(int)data->recv_dustbin_on_the_car
    <<" R_CC_DB: "<<(int)data->dam_board_status
    );
    recv_clean_mechine_motor_status = data->motor_status.data;

    //判断清扫车在车上的状态判断
    if(set_dustbin_on_car){ 
        if((data->recv_dustbin_on_the_car == 1 && box_type == 4) || (data->recv_dustbin_on_the_car == 0 && box_type != 4)){
            set_dustbin_on_car = false;
        }
    }
    //发布挡板状态
    std_msgs::UInt8 dustbin_damboard_msg;
    dustbin_damboard_msg.data = data->dam_board_status;
    dustbin_damboard_pub.publish(dustbin_damboard_msg);
}

//************************************** 环卫车清扫状态发布_新的,加边刷伸缩
void pub_clean_state_new(clean_to_motion_t_new *data)
{
    //发布旧的消息类型
    cti_msgs::DustbinState clean_state_msg;
    clean_state_msg.header.stamp = ros::Time::now();
    clean_state_msg.header.frame_id = "/chassis_dustbin";

	clean_state_msg.water_tank_top= data->water_tank_top;
	clean_state_msg.water_tank_bottom= data->water_tank_bottom;
	clean_state_msg.dustbin_tail_cover= data->dustbin_tail_cover;
	clean_state_msg.dustbin_distance= data->dustbin_distance;
    clean_state_msg.motor_status = data->motor_status.data;
    
    for(int i = 0;i<sizeof(data->ultrasonic_distance)/sizeof(uint16_t);i++)
    {
        clean_state_msg.ultrasonic_distance.push_back(data->ultrasonic_distance[i]);
    }
    clean_state_msg.voltage1= data->voltage1;
    clean_state_msg.voltage2= data->voltage2;
    clean_state_msg.voltage3= data->voltage3;

    dustbin_state_pub.publish(clean_state_msg);
    
    //发布新的消息类型
    cti_msgs::DustbinStateNew clean_state_msg_new;
    clean_state_msg_new.header.stamp = ros::Time::now();
    clean_state_msg_new.header.frame_id = "/chassis_dustbin";

	clean_state_msg_new.water_tank_top= data->water_tank_top;
	clean_state_msg_new.water_tank_bottom= data->water_tank_bottom;
	clean_state_msg_new.dustbin_tail_cover= data->dustbin_tail_cover;
	clean_state_msg_new.dustbin_distance= data->dustbin_distance;
    clean_state_msg_new.motor_status = data->motor_status.data;
    
    for(int i = 0;i<sizeof(data->ultrasonic_distance)/sizeof(uint16_t);i++)
    {
        clean_state_msg_new.ultrasonic_distance.push_back(data->ultrasonic_distance[i]);
    }
    clean_state_msg_new.voltage1= data->voltage1;
    clean_state_msg_new.voltage2= data->voltage2;
    clean_state_msg_new.voltage3= data->voltage3;
    clean_state_msg_new.recv_dustbin_on_the_car = data->recv_dustbin_on_the_car;
    clean_state_msg_new.dam_board_status = data->dam_board_status;
    clean_state_msg_new.side_brush_transform_state = data->side_brush_transform_state;
    clean_state_msg_new.side_brush_speed = data->side_brush_speed;
    clean_state_msg_new.error_code =  data->error_code;
    clean_state_msg_new.unused0 = data->side_brush_transform_error;
    clean_state_msg_new.unused1 = data->left_side_brush_error;
    clean_state_msg_new.unused2 = data->right_side_brush_error;
    clean_state_msg_new.unused3 = data->multi_status.data;

    dustbin_state_pub_new.publish(clean_state_msg_new);


    //使用cti_msgs/DataArray消息类型发布消息
    cti_msgs::DataArray dustVehicleInfos;
    dustVehicleInfos.header.stamp = ros::Time::now();
    dustVehicleInfos.header.frame_id = "sanitation_vehicle";
    //水箱高水位
    cti_msgs::Data dustVehicleInfo;
    dustVehicleInfo.name = "water_tank_top";
    dustVehicleInfo.data = std::to_string(data->water_tank_top);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo); 
    //水箱低水位
    dustVehicleInfo.name = "water_tank_bottom";
    dustVehicleInfo.data = std::to_string(data->water_tank_bottom);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo); 
    //电机状态
    dustVehicleInfo.name = "motor_status";
    dustVehicleInfo.data = std::to_string(data->motor_status.data);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo);
    //挡板状态
    dustVehicleInfo.name = "dam_board_status";
    dustVehicleInfo.data = std::to_string(data->dam_board_status);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo);
    //边刷伸展
    dustVehicleInfo.name = "sidebrush_state";
    dustVehicleInfo.data = std::to_string(data->side_brush_transform_state);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo);
    //边刷旋转速度
    dustVehicleInfo.name = "sidebrush_speed";
    dustVehicleInfo.data = std::to_string(data->side_brush_speed);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo);
    //清扫功能错误码
    dustVehicleInfo.name = "error_code";
    dustVehicleInfo.data = std::to_string(data->error_code);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT32;
    dustVehicleInfos.datas.push_back(dustVehicleInfo);
    //边刷伸展错误码
    dustVehicleInfo.name = "sidebrush_transform_error";
    dustVehicleInfo.data = std::to_string(data->side_brush_transform_error);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo);
    //左边刷错误码
    dustVehicleInfo.name = "left_sidebrush_error";
    dustVehicleInfo.data = std::to_string(data->left_side_brush_error);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo);
    //右边刷错误码
    dustVehicleInfo.name = "right_sidebrush_error";
    dustVehicleInfo.data = std::to_string(data->right_side_brush_error);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo);
    //加水水泵状态
    dustVehicleInfo.name = "increase_water_pump";
    uint8_t increase_water_pump_state = data->multi_status.bits.increase_water_pump;
    dustVehicleInfo.data = std::to_string(increase_water_pump_state);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo);

    //存储上水位传感器数据
    vehicle_water_tank_top.push_back(data->water_tank_top);
    vehicle_water_tank_bottom.push_back(data->water_tank_bottom);
    if (vehicle_water_tank_top.size() > max_vehicle_water_tank_restore){
        vehicle_water_tank_top.pop_front();
    }
    if (vehicle_water_tank_bottom.size() > max_vehicle_water_tank_restore){
        vehicle_water_tank_bottom.pop_front();
    }
    //判断水箱是否满了。
    uint8_t water_full = 2;//水满状态标志位 0: 水箱未满 1：水箱满 2：无效数据
    //tank_top传感器定义：  全是 0 ：代表水满 全是1：代表水没有满  有0有1：无效数据
    uint8_t tank_top_sum = 0;
    for(int i = 0;i < max_vehicle_water_tank_restore; i++){
        tank_top_sum += vehicle_water_tank_top[i];
    }
    if(0 == tank_top_sum){
        water_full = 1; //传感器数据全为0时，代表水箱满了
    }else if( max_vehicle_water_tank_restore == tank_top_sum ){
        water_full = 0; //传感器数据全为1时，代表水箱未满
    }else{
        water_full = 2; //无效数据
    }

    //p判断水箱是否为空
    uint8_t water_empty = 2;//水空状态标志位： 0： 水箱未空 1：水箱空 2：无效数据
    //tank_top传感器定义 全是 0 ：代表水空 全是1：代表水没有空  有0有1：无效数据
    uint8_t tank_bottom_sum = 0;
    for(int i = 0;i < max_vehicle_water_tank_restore; i++){
        tank_bottom_sum +=  vehicle_water_tank_bottom[i];
    }
    if(0 == tank_bottom_sum){
        water_empty = 1; //全部数据为0代表水空
    }else if(max_vehicle_water_tank_restore == tank_bottom_sum){
        water_empty = 0; //全部数据为1代表水未空
    }else{
        water_empty = 2; //无效数据
    }

    //水量计算修改： 改为4档水量 0%  5% 50% 100%
    if(1 == water_full && 0 == water_empty){     //水满
        vehicle_water_status.water_percnt_now = 100.0;
    }else if(1 == water_empty && 0 == water_full){ //水空
        vehicle_water_status.water_percnt_now = 0.0;
    }else if(0 == water_empty && 0 == water_full){//有水未满
        vehicle_water_status.water_percnt_now = 50.0;
    }else if( 1 == water_full && 1 == water_empty){//水空和水满都被触发，水量报5%，然后报错
        vehicle_water_status.water_percnt_now = 5.0;
        pubErrorMsg("SENSOR", 1, 11, "水箱水位传感器状态异常");
    }else{
        ;
    }

    //计算水量百分比
    //排水状态
    // if(31 == data->motor_status.data){
    //     if(false == vehicle_water_status.is_out){
    //         vehicle_water_status.is_out = true;
    //         vehicle_water_status.out_pre_time = ros::Time::now().toSec();
    //     }
    // }else if(27 == data->motor_status.data || 0 == data->motor_status.data){
    //     if(true == vehicle_water_status.is_out){
    //         vehicle_water_status.is_out = false;
    //         vehicle_water_status.out_pre_time = ros::Time::now().toSec();
    //     }
    // }
    // //进水状态
    // if(1 == increase_water_pump_state){
    //     if(false == vehicle_water_status.is_in){
    //         vehicle_water_status.is_in = true;
    //         vehicle_water_status.in_pre_time = ros::Time::now().toSec();
    //     }
    // }else if(0 == increase_water_pump_state){
    //     if(true == vehicle_water_status.is_in){
    //         vehicle_water_status.is_out = false;
    //         vehicle_water_status.out_pre_time = ros::Time::now().toSec();
    //     }
    // }

    // if(1 == water_full && 0 == water_empty){
    //     vehicle_water_status.water_percnt_now = 100.0;
    // }else if(1 == water_empty && 0 == water_full){
    //     vehicle_water_status.water_percnt_now = 0.0;
    // }else{
    //     //排水计算
    //     if(true == vehicle_water_status.is_out){
    //         double time_now = ros::Time::now().toSec();
    //         double duration = time_now - vehicle_water_status.out_pre_time;
    //         vehicle_water_status.water_percnt_now =  vehicle_water_status.water_percnt_now - duration * vehicle_water_status.out_per_sec;
    //         if (vehicle_water_status.water_percnt_now > 90.0){
    //             vehicle_water_status.water_percnt_now = 90.0;
    //         }
    //         if (vehicle_water_status.water_percnt_now < 10.0){
    //             vehicle_water_status.water_percnt_now = 10.0;
    //         }
    //         vehicle_water_status.out_pre_time = time_now;
    //     }
    //     //加水计算
    //     if(true == vehicle_water_status.is_in){
    //         double time_now = ros::Time::now().toSec();
    //         double duration = time_now - vehicle_water_status.in_pre_time;
    //         vehicle_water_status.water_percnt_now =  vehicle_water_status.water_percnt_now + duration * vehicle_water_status.in_per_sec;
    //         if (vehicle_water_status.water_percnt_now > 90.0){
    //             vehicle_water_status.water_percnt_now = 90.0;
    //         }
    //         if (vehicle_water_status.water_percnt_now < 10.0){
    //             vehicle_water_status.water_percnt_now = 10.0;
    //         }
    //         vehicle_water_status.out_pre_time = time_now;
    //     }
    // }
    
    //水量百分比
    dustVehicleInfo.name = "water_percent";
    dustVehicleInfo.data = std::to_string(int(vehicle_water_status.water_percnt_now));
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo); 
    // std::string modify_file = "echo "+ std::to_string(vehicle_water_status.water_percnt_now) + " > " + config_file_path;
    // FILE *fpr = NULL;
    // fpr = popen(modify_file.c_str(),"w");
    // pclose(fpr);
    //装饰灯状态发布
    dustVehicleInfo.name = "decorate_light";
    uint8_t decorate_light_state = data->multi_status.bits.decorate_light;
    dustVehicleInfo.data = std::to_string(decorate_light_state);
    dustVehicleInfo.type = cti_msgs::Data::TYPE_UINT8;
    dustVehicleInfos.datas.push_back(dustVehicleInfo);

    //消息发布
    dust_vehicle_state_info_pub.publish(dustVehicleInfos);

    //储存状态
    clean_function_test_state.recv_damboard_status = data->dam_board_status;
    clean_function_test_state.recv_side_brush_transform_state = data->side_brush_transform_state;
    clean_function_test_state.recv_side_brush_speed = data->side_brush_speed;
    clean_function_test_state.recv_spray_motor = (data->motor_status.data == 27 || data->motor_status.data == 0)?0:1;

    //发布超声波数据
    if(data->ultrasonic_distance[0] == 0xffff){//新版超声波
        sensor_msgs::Range new_sweeperRangeData;
        int index = (data->ultrasonic_distance[1])-1;
        const float sweeper_min_range=0.12;
        const float sweeper_max_range=2.5;
        new_sweeperRangeData.header.stamp = ros::Time::now();
        new_sweeperRangeData.radiation_type = 0;//ult
        new_sweeperRangeData.min_range = sweeper_min_range;
        new_sweeperRangeData.max_range = sweeper_max_range;
        new_sweeperRangeData.field_of_view = 24*M_PI/180.0f;
        for(int i=0;i<new_sweeper_max_type_ult;i++){
            float ult_data = (float)data->ultrasonic_distance[i+2]/1000.f;
            new_sweeperRangeData.range = ult_data; 
            new_sweeperRangeData.header.frame_id = new_sweeper_ult_name[index][i];
            new_sweeper_range_pub[index][i].publish(new_sweeperRangeData);
        }
    }
    else{//旧版超声波
        const float sweeper_min_range=0.12;
        const float sweeper_max_range=2.5;
        sweeperRangeData.header.stamp = ros::Time::now();
        sweeperRangeData.radiation_type = 0;//ult
        sweeperRangeData.min_range = sweeper_min_range;
        sweeperRangeData.max_range = sweeper_max_range;
        sweeperRangeData.field_of_view = 24*M_PI/180.0f;
        for(int i=0;i<sweeper_max_type_ult;i++){
            float ult_data = (float)data->ultrasonic_distance[i]/1000.f;
            sweeperRangeData.range = ult_data; 
            sweeperRangeData.header.frame_id = sweeper_ult_name[i];
            sweeper_range_pub[i].publish(sweeperRangeData);
        }
    }
    Info("R_CC_ES: "<<(int)data->motor_status.data
    <<" R_CC_WT: "<<(int)data->water_tank_top
    <<" R_CC_WB: "<<(int)data->water_tank_bottom
    <<" R_CC_TC: "<<(int)data->dustbin_tail_cover
    <<" R_CC_DS: "<<(int)data->dustbin_distance
    <<" R_CC_DOC: "<<(int)data->recv_dustbin_on_the_car
    <<" R_CC_DB: "<<(int)data->dam_board_status
    <<" R_CC_SBT: "<<(int)data->side_brush_transform_state
    <<" R_CC_SBS: "<<(int)data->side_brush_speed
    <<" R_CC_ERR: "<<(int)data->error_code
    <<" R_CC_SBTE: "<<(int)data->side_brush_transform_error
    <<" R_CC_LSBE: "<<(int)data->left_side_brush_error
    <<" R_CC_RSBE: "<<(int)data->right_side_brush_error
    <<" R_CC_WP: "<<(int)vehicle_water_status.water_percnt_now
    <<" R_CC_DE_LG: "<<(int)decorate_light_state
    <<" R_CC_IN_PU: "<<(int)increase_water_pump_state
    );
    recv_clean_mechine_motor_status = data->motor_status.data;

    //判断清扫车在车上的状态判断
    if(set_dustbin_on_car){ 
        if((data->recv_dustbin_on_the_car == 1 && box_type == 4) || (data->recv_dustbin_on_the_car == 0 && box_type != 4)){
            set_dustbin_on_car = false;
        }
    }
    //发布挡板状态
    std_msgs::UInt8 dustbin_damboard_msg;
    dustbin_damboard_msg.data = data->dam_board_status;
    dustbin_damboard_pub.publish(dustbin_damboard_msg);
}
//************************************** 集尘箱状态发布(旧的,没有电池,风机力矩等)
void pub_dust_box_state_old(dust_box_to_motion_t_old *data)
{
    cti_msgs::DustbinState dust_box_state_msg;
    dust_box_state_msg.header.stamp = ros::Time::now();
    dust_box_state_msg.header.frame_id = "/chassis_dust_box";

	dust_box_state_msg.water_tank_top= data->water_tank_top;
	dust_box_state_msg.water_tank_bottom= data->water_tank_bottom;
	dust_box_state_msg.dustbin_distance= data->dustbin_distance;
    int ultrasonic_distance_length = sizeof(data->ultrasonic_distance) / sizeof(data->ultrasonic_distance[0]);
    for(int i = 0; i < ultrasonic_distance_length;i++){
        dust_box_state_msg.ultrasonic_distance.push_back(data->ultrasonic_distance[i]);
    }
    dust_box_state_msg.voltage1 = data->voltage1;
    dust_box_state_msg.voltage2 = data->voltage2;
    dust_box_state_msg.voltage3 = data->voltage3;
    dust_box_state_msg.motor_status = data->motor_status.data;
    dust_box_state_pub.publish(dust_box_state_msg); 


     //使用cti_msgs/DataArray消息类型发布消息
    cti_msgs::DataArray dustBoxInfos;
    dustBoxInfos.header.stamp = ros::Time::now();
    dustBoxInfos.header.frame_id = "sanitation_dust_box";

    cti_msgs::Data dustboxinfo;
    //垃圾到顶盖的距离
    dustboxinfo.name = "dust_distance";
    dustboxinfo.data = std::to_string(data->dustbin_distance);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT16;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //电池1的电压
    dustboxinfo.name = "bat1_vol";
    dustboxinfo.data = std::to_string(data->voltage1);
    dustboxinfo.type = cti_msgs::Data::TYPE_FLOAT;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //电池2的电压
    dustboxinfo.name = "bat2_vol";
    dustboxinfo.data = std::to_string(data->voltage2);
    dustboxinfo.type = cti_msgs::Data::TYPE_FLOAT;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //电池3的电压
    dustboxinfo.name = "bat3_vol";
    dustboxinfo.data = std::to_string(data->voltage3);
    dustboxinfo.type = cti_msgs::Data::TYPE_FLOAT;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //吸尘箱启动状态
    dustboxinfo.name = "motor_status";
    dustboxinfo.data = std::to_string(data->motor_status.data);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //自动倾倒垃圾的状态
    dustboxinfo.name = "dust_autopush";
    dustboxinfo.data = std::to_string(data->auto_push);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //风机速度,控制反馈
    dustboxinfo.name = "fan_Speed";
    dustboxinfo.data = std::to_string(data->fan_speed);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //垃圾量百分比
    dustboxinfo.name = "trash_percent";
    dustboxinfo.data = std::to_string(data->trash_per);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //消息发布
    dust_box_state_info_pub.publish(dustBoxInfos);


    //箱尾超声波发布  //v6.0 v6.1为四个超声波都在箱子后面 v6.2 1-3号超声波在箱子后面 4号在垃圾仓里
    //位于后方的原始数据发布
    std_msgs::UInt16MultiArray dustbox_rear_ult;
    int size = sizeof(data->ultrasonic_distance) / sizeof(data->ultrasonic_distance[0]);
    for(int i=0;i<size;i++){
        dustbox_rear_ult.data.push_back(data->ultrasonic_distance[i]);
    }
    dustbox_rear_range_pub.publish(dustbox_rear_ult);
    if(cti_run_ver != "v6.0" && cti_run_ver != "v6.1"){
        sensor_msgs::Range dustboxrRangeData;
        const float dustbox_min_range=0.12;
        const float dustbox_max_range=2.5;
        dustboxrRangeData.header.stamp = ros::Time::now();
        dustboxrRangeData.radiation_type = 0;//ult
        dustboxrRangeData.min_range = dustbox_min_range;
        dustboxrRangeData.max_range = msg_max_range;
        dustboxrRangeData.field_of_view = 24*M_PI/180.0f;
        for(int i=0;i<dustbox_max_type_ult;i++){
            float ult_data;
            if(0 == data->auto_push){
                ult_data = (float)data->ultrasonic_distance[i+2]/1000.f;
            }else{
                ult_data = 8.88; //8.88表示此时在倾倒垃圾,超声波数据给一个超出正常范围的值
            }
            dustboxrRangeData.range = ult_data; 
            dustboxrRangeData.header.frame_id = dustbox_ult_name[i];
            dustbox_range_pub[i].publish(dustboxrRangeData);
        }
        float ult_data;
        ult_data = (float)data->ultrasonic_distance[5]/1000.f;
        dustboxrRangeData.range = ult_data; 
        dustboxrRangeData.header.frame_id = "bottom";
        dustbox_bottom_range_pub.publish(dustboxrRangeData);
    }else{
        sensor_msgs::Range dustboxrRangeData;
        const float dustbox_min_range=0.12;
        const float dustbox_max_range=2.5;
        dustboxrRangeData.header.stamp = ros::Time::now();
        dustboxrRangeData.radiation_type = 0;//ult
        dustboxrRangeData.min_range = dustbox_min_range;
        dustboxrRangeData.max_range = msg_max_range;
        dustboxrRangeData.field_of_view = 24*M_PI/180.0f;
        for(int i=0;i<dustbox_max_type_ult;i++){
            float ult_data;
            if(0 == data->auto_push){
                ult_data = (float)data->ultrasonic_distance[i+2]/1000.f;
            }else{
                ult_data = 8.88; //8.88表示此时在倾倒垃圾,超声波数据给一个超出正常范围的值
            }
            dustboxrRangeData.range = ult_data; 
            dustboxrRangeData.header.frame_id = dustbox_ult_name[i];
            dustbox_range_pub[i].publish(dustboxrRangeData);
        }
    }
    //自动倒垃圾状态
    std_msgs::UInt8 dust_box_autopush_msg;
    dust_box_autopush_msg.data = data->auto_push;
    dust_box_autopush_pub.publish(dust_box_autopush_msg);

    //风机速度
    std_msgs::UInt8 dust_box_fanspeed_msg;
    dust_box_fanspeed_msg.data = data->fan_speed;
    dust_box_fanspeed_pub.publish(dust_box_fanspeed_msg);
        
    //无线充电
    cti_msgs::BatteryState wireless_charge_state_msg;
    wireless_charge_state_msg.header.stamp = ros::Time::now();
    wireless_charge_state_msg.wireless_install_state = 1;
    wireless_charge_state_msg.wireless_voltage = data->wireless_voltage;
    wireless_charge_state_msg.wireless_current = data->wireless_current;
    wireless_charge_state_msg.wireless_reserve = data->wireless_reserve;
    wireless_charge_state_msg.wireless_state = data->wireless_state;
    wireless_charge_state_msg.wireless_temp = data->wireless_temp;
    wireless_charge_state_msg.wireless_changer = data->wireless_changer;
    dustbox_wireless_charge_state_pub.publish(wireless_charge_state_msg);

    //使用json发布状态<无用，保留代码做参考>
    Json::Value dustboxStateJson;
    Json::Value wirelessChargeJson;
    Json::FastWriter writer;
    dustboxStateJson["stamp"] = std::to_string(ros::Time::now().toSec());
    dustboxStateJson["portIndex"] = std::to_string(data->portIndex);
    dustboxStateJson["water_tank_top"] = std::to_string(data->water_tank_top);
    dustboxStateJson["dustbin_distance"] = std::to_string(data->dustbin_distance);
    dustboxStateJson["voltage1"] = std::to_string(data->voltage1);
    dustboxStateJson["voltage2"] = std::to_string(data->voltage2);
    dustboxStateJson["voltage3"] = std::to_string(data->voltage3);
    dustboxStateJson["motor_status"] = std::to_string(data->motor_status.data); 
    dustboxStateJson["recv_dustbin_on_the_car"] = std::to_string(data->recv_dustbin_on_the_car);
    dustboxStateJson["auto_push"] = std::to_string(data->auto_push); 
    dustboxStateJson["fan_speed"] = std::to_string(data->fan_speed);  
    //无线充电状态start
    wirelessChargeJson["wireless_voltage"] = std::to_string(data->wireless_voltage);
    wirelessChargeJson["wireless_current"] = std::to_string(data->wireless_current);
    wirelessChargeJson["wireless_reserve"] = std::to_string(data->wireless_reserve);
    wirelessChargeJson["wireless_state"] = std::to_string(data->wireless_state);
    wirelessChargeJson["wireless_temp"] = std::to_string(data->wireless_temp);
    wirelessChargeJson["wireless_charger"] = std::to_string(data->wireless_changer);
    dustboxStateJson["wireless_charge"] = wirelessChargeJson;
    //无线充电状态 end
    std_msgs::String dustboxStateStr;
    dustboxStateStr.data = Json::FastWriter().write(dustboxStateJson);
    dust_box_state_pub_json.publish(dustboxStateStr);

    Info("R_DBO_ES: "<<(int)data->motor_status.data
    <<" R_DBO_WT: "<<(int)data->water_tank_top
    <<" R_DBO_WB: "<<(int)data->water_tank_bottom
    <<" R_DBO_DS: "<<(int)data->dustbin_distance
    <<" R_DBO_DOC: "<<(int)data->recv_dustbin_on_the_car
    <<" R_DBO_AP: "<<(int)data->auto_push
    <<" R_DBO_FS: "<<(int)data->fan_speed
    <<" R_DBO_WC_IS: "<<(int)wireless_charge_state_msg.wireless_install_state
    <<" R_DBO_WC_VT: "<<data->wireless_voltage
    <<" R_DBO_WC_CU: "<<data->wireless_current
    <<" R_DBO_WC_RE: "<<(int)data->wireless_reserve
    <<" R_DBO_WC_ST: "<<(int)data->wireless_state
    <<" R_DBO_WC_TP: "<<(int)data->wireless_temp
    <<" R_DBO_WC_CH: "<<(int)data->wireless_changer
    <<" R_DBO_TRA_PER: "<<(int)data->trash_per
    );   

    //-------------------通讯检测------------------
    box_chat_state.module_id = 2;
    box_chat_state.recv_state = true;
    box_chat_state.time_recv = ros::Time::now().toSec();
}

//************************************** 集尘箱状态发布(新的,加了电池,风机力矩等)
void pub_dust_box_state_new(dust_box_to_motion_t_new *data)
{
    //发布旧的消息类型,用于兼容以前的版本
    cti_msgs::DustbinState dust_box_state_msg;
    dust_box_state_msg.header.stamp = ros::Time::now();
    dust_box_state_msg.header.frame_id = "/chassis_dust_box";

	dust_box_state_msg.water_tank_top= data->auto_work; //2021-09-09 这个数据修改为新的含义，但是为了兼容旧的，依旧发出 walter_tank_top
	dust_box_state_msg.water_tank_bottom= data->mannual_work;//2021-09-09 这个数据修改为新的含义，但是为了兼容旧的，依旧发出 walter_tank_bottom
	dust_box_state_msg.dustbin_distance= data->dustbin_distance;
    int ultrasonic_distance_length = sizeof(data->ultrasonic_distance) / sizeof(data->ultrasonic_distance[0]);
    for(int i = 0; i < ultrasonic_distance_length;i++){
        dust_box_state_msg.ultrasonic_distance.push_back(data->ultrasonic_distance[i]);
    }
    dust_box_state_msg.voltage1 = data->voltage1;
    dust_box_state_msg.voltage2 = data->voltage2;
    dust_box_state_msg.voltage3 = data->voltage3;
    dust_box_state_msg.motor_status = data->motor_status.data;
    dust_box_state_pub.publish(dust_box_state_msg); 
    
    //使用cti_msgs/DataArray消息类型发布消息
    cti_msgs::DataArray dustBoxInfos;
    dustBoxInfos.header.stamp = ros::Time::now();
    dustBoxInfos.header.frame_id = "sanitation_dust_box";
    cti_msgs::Data dustboxinfo;
    //垃圾到顶盖的距离
    dustboxinfo.name = "dust_distance";
    dustboxinfo.data = std::to_string(data->dustbin_distance);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT16;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //如果小于0x32保持10s,app上提醒垃圾仓超声波探头需要擦拭，持续3s正常提醒删除

    //电池1的电压
    dustboxinfo.name = "bat1_vol";
    dustboxinfo.data = std::to_string(data->voltage1);
    dustboxinfo.type = cti_msgs::Data::TYPE_FLOAT;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //电池1的电量
    dustboxinfo.name = "bat1_soc";
    dustboxinfo.data = std::to_string(data->Bat_1_Soc);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //电池2的电压
    dustboxinfo.name = "bat2_vol";
    dustboxinfo.data = std::to_string(data->voltage2);
    dustboxinfo.type = cti_msgs::Data::TYPE_FLOAT;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //电池2的电量
    dustboxinfo.name = "bat2_soc";
    dustboxinfo.data = std::to_string(data->Bat_2_Soc);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //电池3的电压
    dustboxinfo.name = "bat3_vol";
    dustboxinfo.data = std::to_string(data->voltage3);
    dustboxinfo.type = cti_msgs::Data::TYPE_FLOAT;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //电池3的电量
    dustboxinfo.name = "bat3_soc";
    dustboxinfo.data = std::to_string(data->Bat_3_Soc);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //电池4的电压
    dustboxinfo.name = "bat4_vol";
    dustboxinfo.data = std::to_string(data->voltage4);
    dustboxinfo.type = cti_msgs::Data::TYPE_FLOAT;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //电池4的电量
    dustboxinfo.name = "bat4_soc";
    dustboxinfo.data = std::to_string(data->Bat_4_Soc);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //箱子整体电量
    dustboxinfo.name = "dustbox_soc";
    uint8_t dustbox_soc = 0;
    if(data->voltage4 < 0){
        //箱子最多装3个电池
        dustbox_soc = (data->Bat_1_Soc + data->Bat_2_Soc + data->Bat_3_Soc) / 3;
    }else{
        //箱子最多装4个电池
        dustbox_soc = (data->Bat_1_Soc + data->Bat_2_Soc + data->Bat_3_Soc + data->Bat_4_Soc) / 4;
    }
    if(dustbox_soc > 100)
        dustbox_soc = 100;
    dustboxinfo.data = std::to_string(dustbox_soc);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //箱子整体电量，箱子上传
    dustboxinfo.name = "dustbox_soc_from_box";
    dustboxinfo.data = std::to_string(data->all_Soc);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //吸尘箱启动状态
    dustboxinfo.name = "motor_status";
    dustboxinfo.data = std::to_string(data->motor_status.data);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //自动倾倒垃圾的状态
    dustboxinfo.name = "dust_autopush";
    dustboxinfo.data = std::to_string(data->auto_push);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //风机速度,控制反馈
    dustboxinfo.name = "fan_Speed";
    dustboxinfo.data = std::to_string(data->fan_speed);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //风机转矩
    dustboxinfo.name = "fan_torque";
    dustboxinfo.data = std::to_string(data->fan_torque);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //风机转速,硬件反馈
    dustboxinfo.name = "fan_speed_hd";
    dustboxinfo.data = std::to_string(data->fan_speed_closedloop);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT16;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //风机电流
    dustboxinfo.name = "fan_current";
    dustboxinfo.data = std::to_string(data->fan_current);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //板子错误码
    dustboxinfo.name = "error_code";
    dustboxinfo.data = std::to_string(data->errorInfo);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT64;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //箱子4g状态
    dustboxinfo.name = "4g_state";
    dustboxinfo.data = std::to_string(data->net_state);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //箱子4g信号
    dustboxinfo.name = "4g_signal";
    dustboxinfo.data = std::to_string(data->net_signal);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //垃圾量百分比
    dustboxinfo.name = "trash_percent";
    dustboxinfo.data = std::to_string(data->trash_per);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //传感器数据 0: 无效,无此功能 1：关 2：开
    dustboxinfo.name = "auto_work";
    dustboxinfo.data = std::to_string(data->auto_work);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //传感器数据 0: 无效,无此功能 1：关 2：开
    dustboxinfo.name = "mannual_work";
    dustboxinfo.data = std::to_string(data->mannual_work);
    dustboxinfo.type = cti_msgs::Data::TYPE_UINT8;
    dustBoxInfos.datas.push_back(dustboxinfo);
    //--人机协作工作模式--
    //mannual_work  #0(null),1(手动),2(自动)
    //auto_work     #0(null),1(卸下),2(合上)
    int workmode;
    if(data->auto_work== 0 && data->mannual_work == 0){ //未识别到传感器
        workmode = -1;
    }else if(data->auto_work== 2 && data->mannual_work == 2){ //自动模式
        workmode = 0;
    }else if(data->auto_work== 1 && data->mannual_work == 1){ //人机模式
        workmode = 1;
    }else if(data->auto_work== 2 && data->mannual_work == 1){ //
        workmode = 2;
    }else if(data->auto_work== 1 && data->mannual_work == 2){ //
        workmode = 3;
    }else{ //状态异常
        workmode = 4;
    }
    dustboxinfo.name = "workmode";
    dustboxinfo.data = std::to_string(workmode);
    dustboxinfo.type = cti_msgs::Data::TYPE_INT64;
    dustBoxInfos.datas.push_back(dustboxinfo);

    //消息发布
    dust_box_state_info_pub.publish(dustBoxInfos);
    
    //状态储存
    clean_function_test_state.recv_fanspeed = data->fan_speed;

    //电池电量发布
    cti_msgs::BatteryCellsState BatCellsState;
    BatCellsState.header.stamp = ros::Time::now();
    BatCellsState.Soc_All = dustbox_soc;
    BatCellsState.Volt_All = cmax(cmax(cmax(data->voltage1,data->voltage2),data->voltage3),data->voltage4);
    cti_msgs::BatteryCell BatCell;
    if(data->voltage1 > 5){
        BatCell.Bat_Soc = data->Bat_1_Soc;
        BatCell.Bat_Volt = data->voltage1;
        BatCell.Bat_cell_num = 1;
        BatCellsState.BatCells.push_back(BatCell);
    }
    if(data->voltage2 > 5){
        BatCell.Bat_Soc = data->Bat_2_Soc;
        BatCell.Bat_Volt = data->voltage2;
        BatCell.Bat_cell_num = 2;
        BatCellsState.BatCells.push_back(BatCell);
    }
    if(data->voltage3 > 5){
        BatCell.Bat_Soc = data->Bat_3_Soc;
        BatCell.Bat_Volt = data->voltage3;
        BatCell.Bat_cell_num = 3;
        BatCellsState.BatCells.push_back(BatCell);
    }
    if(data->voltage4 > 5){
        BatCell.Bat_Soc = data->Bat_4_Soc;
        BatCell.Bat_Volt = data->voltage4;
        BatCell.Bat_cell_num = 4;
        BatCellsState.BatCells.push_back(BatCell);
    }
    BatCellsState.charge_type = 1;//0:undefined 1:wireless_charge 2:wired_charge
    BatCellsState.charge_voltage = data->wireless_voltage;
    BatCellsState.charge_current = data->wireless_current;
    BatCellsState.charge_reserve = data->wireless_reserve;
    BatCellsState.charge_state = data->wireless_state;
    BatCellsState.charge_temp = data->wireless_temp;
        //对充电标志位进行窗口滤波
    static std::deque<uint8_t> dustbox_charger_queue_;
    dustbox_charger_queue_.push_back(data->wireless_changer);
    if(dustbox_charger_queue_.size() > 3)
        dustbox_charger_queue_.pop_front();
    uint8_t wireless_changer = 1;
    if(dustbox_charger_queue_[0] == 0 && dustbox_charger_queue_[1] == 0 && dustbox_charger_queue_[2] == 0 )
        wireless_changer = 0;
    BatCellsState.charge_charger = wireless_changer;
    dustbox_batterycell_pub.publish(BatCellsState);


    //箱尾超声波发布  v6.0 v6.1为四个超声波都在箱子后面 v6.2  v6.3 1-3号超声波在箱子后面 4号在垃圾仓里 v6.5 安装的是lin通信超声波
    if(cti_run_ver == "v6.2" || cti_run_ver == "v6.3"){
        sensor_msgs::Range dustboxrRangeData;
        const float dustbox_min_range=0.12;
        const float dustbox_max_range=2.5;
        dustboxrRangeData.header.stamp = ros::Time::now();
        dustboxrRangeData.radiation_type = 0;//ult
        dustboxrRangeData.min_range = dustbox_min_range;
        dustboxrRangeData.max_range = msg_max_range;
        dustboxrRangeData.field_of_view = 24*M_PI/180.0f;
        for(int i=0;i<dustbox_max_type_ult;i++){
            float ult_data;
            if(0 == data->auto_push){
                ult_data = (float)data->ultrasonic_distance[i+2]/1000.f;
            }else{
                ult_data = 8.88; //8.88表示此时在倾倒垃圾,超声波数据给一个超出正常范围的值
            }
            dustboxrRangeData.range = ult_data; 
            dustboxrRangeData.header.frame_id = dustbox_ult_name[i];
            dustbox_range_pub[i].publish(dustboxrRangeData);
        }
        float ult_data;
        ult_data = (float)data->ultrasonic_distance[5]/1000.f;
        dustboxrRangeData.range = ult_data; 
        dustboxrRangeData.header.frame_id = "bottom";
        dustbox_bottom_range_pub.publish(dustboxrRangeData);
    }else if( cti_run_ver == "v6.0" || cti_run_ver == "v6.1" ){
        sensor_msgs::Range dustboxrRangeData;
        const float dustbox_min_range=0.12;
        const float dustbox_max_range=2.5;
        dustboxrRangeData.header.stamp = ros::Time::now();
        dustboxrRangeData.radiation_type = 0;
        dustboxrRangeData.min_range = dustbox_min_range;
        dustboxrRangeData.max_range = msg_max_range;
        dustboxrRangeData.field_of_view = 24*M_PI/180.0f;
        for(int i=0;i<dustbox_max_type_ult;i++){
            float ult_data;
            if(0 == data->auto_push){
                ult_data = (float)data->ultrasonic_distance[i+2]/1000.f;
            }else{
                ult_data = 8.88; //8.88表示此时在倾倒垃圾,超声波数据给一个超出正常范围的值
            }
            dustboxrRangeData.range = ult_data; 
            dustboxrRangeData.header.frame_id = dustbox_ult_name[i];
            dustbox_range_pub[i].publish(dustboxrRangeData);
        }
    }
    else if( cti_run_ver == "v6.5"){
        if(LIN_ult_installed){
            //所有的原始数据发布
            std_msgs::UInt16MultiArray dustbox_rear_ult;
            int size = sizeof(data->ultrasonic_distance) / sizeof(data->ultrasonic_distance[0]);
            for(int i=0;i<size;i++){
                dustbox_rear_ult.data.push_back(data->ultrasonic_distance[i]);
            }
            //根据超声波控制命令dustbox_ult_cmd，来进行数据更改
            if((dustbox_ult_cmd & 0x0000000000004000) == 0){
                //箱子后右超声波关闭
                dustbox_rear_ult.data[2] = 2500;
            }
            if((dustbox_ult_cmd & 0x0000000000008000) == 0){
                //箱子后左超声波关闭
                dustbox_rear_ult.data[3] = 2500;
            }
            if((dustbox_ult_cmd & 0x0000000000010000) == 0){
                //箱子底部超声波关闭
                dustbox_rear_ult.data[4] = 2500;
            }
            dustbox_rear_range_pub.publish(dustbox_rear_ult);

            //判断超声波数据是否中断
            static double box_ult_recv_time=0;
            static uint16_t box_ult_cnt_pre = 1;
            if(data->ultrasonic_distance[size-1] != 0){
                if(data->ultrasonic_distance[size-1] != box_ult_cnt_pre){
                    box_ult_recv_time = ros::Time::now().toSec();
                    box_ult_cnt_pre = data->ultrasonic_distance[size-1];
                }
                if(ros::Time::now().toSec() - box_ult_recv_time > 1.0){
                    pubErrorMsg("SENSOR", 1, 11, "清扫箱超声波数据停止更新");
                }
            }
        }
    }
    
    //自动倒垃圾状态
    std_msgs::UInt8 dust_box_autopush_msg;
    dust_box_autopush_msg.data = data->auto_push;
    dust_box_autopush_pub.publish(dust_box_autopush_msg);

    //风机速度
    std_msgs::UInt8 dust_box_fanspeed_msg;
    dust_box_fanspeed_msg.data = data->fan_speed;
    dust_box_fanspeed_pub.publish(dust_box_fanspeed_msg);
        
    //无线充电
    cti_msgs::BatteryState wireless_charge_state_msg;
    wireless_charge_state_msg.header.stamp = ros::Time::now();
    wireless_charge_state_msg.wireless_install_state = 1;
    wireless_charge_state_msg.wireless_voltage = data->wireless_voltage;
    wireless_charge_state_msg.wireless_current = data->wireless_current;
    wireless_charge_state_msg.wireless_reserve = data->wireless_reserve;
    wireless_charge_state_msg.wireless_state = data->wireless_state;
    wireless_charge_state_msg.wireless_temp = data->wireless_temp;
    wireless_charge_state_msg.wireless_changer = data->wireless_changer;
    dustbox_wireless_charge_state_pub.publish(wireless_charge_state_msg);

    Info("R_DBO_ES: "<<(int)data->motor_status.data
    <<" R_DBO_AW: "<<(int)data->auto_work
    <<" R_DBO_MW: "<<(int)data->mannual_work
    <<" R_DBO_DS: "<<(int)data->dustbin_distance
    <<" R_DBO_ALL_SOC: "<<(int)data->all_Soc
    <<" R_DBO_AP: "<<(int)data->auto_push
    <<" R_DBO_FS: "<<(int)data->fan_speed
    <<" R_DBO_WC_IS: "<<(int)wireless_charge_state_msg.wireless_install_state
    <<" R_DBO_WC_VT: "<<data->wireless_voltage
    <<" R_DBO_WC_CU: "<<data->wireless_current
    <<" R_DBO_WC_RE: "<<(int)data->wireless_reserve
    <<" R_DBO_WC_ST: "<<(int)data->wireless_state
    <<" R_DBO_WC_TP: "<<(int)data->wireless_temp
    <<" R_DBO_WC_CH: "<<(int)data->wireless_changer
    <<" R_DBO_WC_CH_RE: "<<(int)wireless_changer
    <<" R_DBO_BAT1: "<<data->voltage1
    <<" R_DBO_SOC1: "<<(int)data->Bat_1_Soc
    <<" R_DBO_BAT2: "<<data->voltage2
    <<" R_DBO_SOC2: "<<(int)data->Bat_2_Soc
    <<" R_DBO_BAT3: "<<data->voltage3
    <<" R_DBO_SOC3: "<<(int)data->Bat_3_Soc
    <<" R_DBO_BAT4: "<<data->voltage4
    <<" R_DBO_SOC4: "<<(int)data->Bat_4_Soc
    <<" R_DBO_FTQ: "<<(int)data->fan_torque
    <<" R_DBO_FSCL: "<<(int)data->fan_speed_closedloop
    <<" R_DBO_FCU: "<<(int)data->fan_current
    <<" R_DBO_ERR: "<<(int)data->errorInfo
    <<" R_DBO_TRA_PER: "<<(int)data->trash_per
    <<" R_DBO_4G_ST: "<<(int)data->net_state
    <<" R_DBO_4G_SI: "<<(int)data->net_signal
    <<" R_DBO_PW_SOC: "<<(int)dustbox_soc
     <<" R_DBO_WM: "<<(int)workmode
    );    
    //-------------------通讯检测------------------
    box_chat_state.module_id = 2;
    box_chat_state.recv_state = true;
    box_chat_state.time_recv = ros::Time::now().toSec();
}

//清扫箱id返回处理
void process_dustbin_id(dustbin_rf_set_cmd_t *data)
{
    //改变状态池
    dustbin_set_state.recv_id = data->id;
    //发布话题
    dustbinidstate.rw = data->rw;
    dustbinidstate.reset = data->reset;
    dustbinidstate.mode = data->mode;
    dustbinidstate.link = data->link;
    dustbinidstate.baud = data->baud;
    dustbinidstate.id = data->id;
    recv_dustbin_id_state_pub.publish(dustbinidstate);
    Info("R_DBI_ID: "<<(int)data->id
        <<" R_DBI_LN: "<<(int)data->link);
}

//************************************** 雨水传感器数据发布
void pub_rain_sensor(rain_sensor_t *data){
    cti_msgs::BoxState rain_sensor_msgs;
    rain_sensor_msgs.header.stamp = ros::Time::now();
    cti_msgs::TabState rain_sensor_msg;
    rain_sensor_msg.status = data->right;
    rain_sensor_msg.name = "right";
    rain_sensor_msgs.states.push_back(rain_sensor_msg);
    rain_sensor_msg.status = data->left;
    rain_sensor_msg.name = "left";
    rain_sensor_msgs.states.push_back(rain_sensor_msg);    
    rain_sensor_pub.publish(rain_sensor_msgs);
    Info("R_RS_R: "<<(int)data->right
        <<"R_RS_L: "<<(int)data->left);
}
//************************************** 雨水传感器数据发布--新的结构体<嵌入式有更新的有没有更新的，所以新结构体和旧结构体都要保留>
void pub_rain_sensor_new(rain_sensor_t_new *data){
    cti_msgs::BoxState rain_sensor_msgs;
    rain_sensor_msgs.header.stamp = ros::Time::now();
    cti_msgs::TabState rain_sensor_msg;
    rain_sensor_msg.status = data->flag;
    rain_sensor_msg.name = "flag";
    rain_sensor_msgs.states.push_back(rain_sensor_msg);
    rain_sensor_msg.status = data->right;
    rain_sensor_msg.name = "right";
    rain_sensor_msgs.states.push_back(rain_sensor_msg);
    rain_sensor_msg.status = data->left;
    rain_sensor_msg.name = "left";
    rain_sensor_msgs.states.push_back(rain_sensor_msg);    
    rain_sensor_pub.publish(rain_sensor_msgs);
    Info("R_RS_R: "<<(int)data->right
        <<"R_RS_L: "<<(int)data->left
        <<"R_RS_F: "<<(int)data->flag);
}

//*********************************** 智能垃圾箱状态发布
void pub_smart_trash(smart_trash_report *data){

    //使用cti_msgs/DataArray消息类型发布消息
    cti_msgs::DataArray smartTrashInfos;
    smartTrashInfos.header.stamp = ros::Time::now();
    smartTrashInfos.header.frame_id = "smart_trash";
    
    cti_msgs::Data smarttrashinfo;
    //当前位置 uint8_t 1:底部：2:顶部
    smarttrashinfo.name = "current_position";
    smarttrashinfo.data = std::to_string(data->current_position);
    smarttrashinfo.type = cti_msgs::Data::TYPE_UINT8;
    smartTrashInfos.datas.push_back(smarttrashinfo); 
    //运动状态 uint8_t 0：静止 1:向上，2向下
    smarttrashinfo.name = "kinestate";
    smarttrashinfo.data = std::to_string(data->kinestate);
    smarttrashinfo.type = cti_msgs::Data::TYPE_UINT8;
    smartTrashInfos.datas.push_back(smarttrashinfo); 
    //具体位置 uint16_t 
    smarttrashinfo.name = "location";
    smarttrashinfo.data = std::to_string(data->location);
    smarttrashinfo.type = cti_msgs::Data::TYPE_UINT16;
    smartTrashInfos.datas.push_back(smarttrashinfo); 
    //升降报错 uint16_t 
    smarttrashinfo.name = "errorinfo";
    smarttrashinfo.data = std::to_string(data->errorinfo);
    smarttrashinfo.type = cti_msgs::Data::TYPE_UINT16;
    smartTrashInfos.datas.push_back(smarttrashinfo); 
    //当前连接lora的id号
    smarttrashinfo.name = "lora_id";
    smarttrashinfo.data = std::to_string(dustbin_set_state.recv_id);
    smarttrashinfo.type = cti_msgs::Data::TYPE_INT64;
    smartTrashInfos.datas.push_back(smarttrashinfo); 
    //感应开关是否开关 1:关闭感应开盖功能 0: 打开感应开盖功能
    smarttrashinfo.name = "reaction_state";
    smarttrashinfo.data = std::to_string(data->reaction_state);
    smarttrashinfo.type = cti_msgs::Data::TYPE_UINT16;
    smartTrashInfos.datas.push_back(smarttrashinfo); 
    
    smart_trash_state_pub.publish(smartTrashInfos);

    Info("R_ST_PO: "<<(int)data->current_position
        <<"R_ST_ST: "<<(int)data->kinestate
        <<"R_ST_LC: "<<(int)data->location
        <<"R_ST_ERR: "<<(int) data->errorinfo    
    );
    //-------------------通讯检测------------------
    box_chat_state.module_id = 3;
    box_chat_state.recv_state = true;
    box_chat_state.time_recv = ros::Time::now().toSec();
}

//*********************************** 吸尘箱5g状态发布<转为箱子卡号id发布>
void pub_dustbox_5g_state(dustbox_5g_recv_cmd_t *data){

    cti_msgs::DataArray dustbox5GStates;
    dustbox5GStates.header.stamp = ros::Time::now();
    dustbox5GStates.header.frame_id = "smart_trash";

    cti_msgs::Data dustbox5GState;
    dustbox5GState.name = "port_index";
    dustbox5GState.data = std::to_string(data->portIndex);
    dustbox5GState.type = cti_msgs::Data::TYPE_UINT8;
    dustbox5GStates.datas.push_back(dustbox5GState); 
    
    dustbox5GState.name = "msg1";
    int array_size = sizeof(data->msg1) / sizeof(data->msg1[0]);
    dustbox5GState.data.clear();
    for(int i = 0; i < array_size ; i++){
        dustbox5GState.data.push_back((char)data->msg1[i]);
    }
    std::string msg1 = dustbox5GState.data;
    dustbox5GState.type = cti_msgs::Data::TYPE_UINT8;
    dustbox5GStates.datas.push_back(dustbox5GState); 

    dustbox5GState.name = "unused1";
    array_size = sizeof(data->unused1) / sizeof(data->unused1[0]);
    dustbox5GState.data.clear();
    for(int i = 0; i < array_size ; i++){
        dustbox5GState.data.push_back((char)data->unused1[i]);
    }
    std::string unused1 = dustbox5GState.data;
    dustbox5GState.type = cti_msgs::Data::TYPE_UINT8;
    dustbox5GStates.datas.push_back(dustbox5GState); 

    dustbox5GState.name = "unused2";
    array_size = sizeof(data->unused2) / sizeof(data->unused2[0]);
    dustbox5GState.data.clear();
    for(int i = 0; i < array_size ; i++){
        dustbox5GState.data.push_back((char)data->unused2[i]);
    }
    std::string unused2 = dustbox5GState.data;
    dustbox5GState.type = cti_msgs::Data::TYPE_UINT8;
    dustbox5GStates.datas.push_back(dustbox5GState); 
    dustbox_5g_state_pub.publish(dustbox5GStates);
    Info("R_DOB_5G_PI: "<<(int)data->portIndex
        <<"R_DOB_5G_MSG1: "<<msg1.c_str()
        <<"R_DOB_5G_UN1: "<<unused1.c_str()
        <<"R_DOB_5G_UN2: "<<unused2.c_str()
    );
}
//--超声波模式接收处理
void process_ult_mode_report(report_lin_ult_mode_type *data){
    //车身超声波处理
    // printf("*****************recv_ult_mode_report******************\n");
    // printf("data->src: %d\n",data->src);
    // printf("data->dest: %d\n",data->dest);  
    // printf("data->mode: 0x%016lx\n",data->mode.data);
    // printf("*****************recv_ult_mode_report******************\n");
  
    if(data->src == MODULE_ULTRASONIC_MASTER){ //车身超声波模式
        vehicle_ult_set_state.recv_mode = data->mode.data;
        if(vehicle_ult_set_state.state == 2 ){
            printf("vehicle_ult_set_state.set_mode: 0x%016lx\n",vehicle_ult_set_state.set_mode);
            printf("vehicle_ult_set_state.recv_mode: 0x%016lx\n",vehicle_ult_set_state.recv_mode);
         
            if(vehicle_ult_set_state.set_mode == vehicle_ult_set_state.recv_mode){
                vehicle_ult_set_state.state = 0;
                vehicle_ult_check_resend_time = 0;
            }else{
                vehicle_ult_set_state.state = 3;
                vehicle_ult_check_resend_time = 0;
            }
        }
        else if(vehicle_ult_set_state.state == 4){
            if(vehicle_ult_set_state.set_mode == vehicle_ult_set_state.recv_mode){
                vehicle_ult_set_state.state = 0;
                vehicle_ult_set_resend_time = 0;
            }else{
                vehicle_ult_set_state.state = -3;
                vehicle_ult_set_resend_time = 0;
            } 
        }
        else{
            vehicle_ult_set_state.state = 0;
        }
        Info("R_V_ULT_MD: "<<data->mode.data);
    }

    //吸尘箱超声波处理
    if(data->src == MODULE_DUST_BOX_BOARD){//吸尘箱超声波模式
        dustbox_ult_set_state.recv_mode = data->mode.data;
        if(dustbox_ult_set_state.state == 2 ){
            if(dustbox_ult_set_state.set_mode == dustbox_ult_set_state.recv_mode){
                dustbox_ult_set_state.state = 0;
                dustbox_ult_check_resend_time = 0;
            }else{
                dustbox_ult_set_state.state = 3;
                dustbox_ult_check_resend_time = 0;
            }
        }
        else if(dustbox_ult_set_state.state == 4){
            if(dustbox_ult_set_state.set_mode == dustbox_ult_set_state.recv_mode){
                dustbox_ult_set_state.state = 0;
                dustbox_ult_set_resend_time = 0;
            }else{
                dustbox_ult_set_state.state = -3;
                dustbox_ult_set_resend_time = 0;
            } 
        }
        else{
            dustbox_ult_set_state.state = 0;
        }
        Info("R_D_ULT_MD: "<<data->mode.data);
    }



#if DEBUG_PRINT
    printf("recv ult mode report+++++++++++++++++++ %08x\n",data->mode.data);
#endif
}

//212命令接收处理（为查询版本时使用）
void process_212_respond(upd_cmd_212_type* data){
    Info("R_212_SRC: "<<data->upd_info.src
    <<"R_212_DEST: "<<data->upd_info.dest
    <<"R_212_ID: "<<data->update_node);
    fw_ver_check_state.major_id_recv = data->update_node;
    if(fw_ver_check_state.state != CH_WAIT_212_RESPOND){
        fw_ver_check_state.state_msg = fw_ver_check_msg_map.at(CH_RECV_212_IN_WRONG_STATE);
        fw_ver_check_state.state_msg = fw_ver_check_state.state_msg + "in state: ";
        fw_ver_check_state.state_msg = fw_ver_check_state.state_msg + fw_ver_check_msg_map.at(fw_ver_check_state.state);
        fw_ver_check_state.state = CH_RECV_212_IN_WRONG_STATE;
        return;
    }
    if(fw_ver_check_state.major_id_recv == fw_ver_check_state.major_id_send){
        fw_ver_check_state.state = CH_212_RESPOND_SUCCESS;
        fw_ver_check_state.state_msg = fw_ver_check_msg_map.at(CH_212_RESPOND_SUCCESS);
    }else{
        fw_ver_check_state.state = CH_212_RESPOND_ERROR;
        fw_ver_check_state.state_msg = fw_ver_check_msg_map.at(CH_212_RESPOND_ERROR);
    }
}

//************************************** 非升级命令接收处理
int process_nomal_cmd_ex(unsigned char cmd, unsigned char *data, unsigned int length)
{
  // printf("nomal cmd=%d\n",cmd);
   switch(cmd){
   case RECV_FROM_CONTROL_STATUS:
       recv_from_control_status_type *recv_statusdata;
       if(sizeof(recv_from_control_status_type) != length){
           printf("recv control length:%d error! sizeof struct length:%d\n",length,(int)sizeof(recv_from_control_status_type));
           return -1;
       }
       recv_ctrl_cnt++;
       recv_ctrl_cnt %= 65535;
       recv_ctrl_rate_cnt++;
       recv_ctrl_rate_cnt %= 65535;
       recv_statusdata = (recv_from_control_status_type *)data;
       pub_odom(recv_statusdata);
       break;
   case RECV_FROM_CONTROL_BATTERY_4_TO_1_STATUS:
       recv_battery_4_to_1_active_report_status_type *recv_batterydata;
       recv_battery_4_to_1_active_report_status_with_wireless_type *recv_batterydata_with_wireless;
       if(sizeof(recv_battery_4_to_1_active_report_status_type) != length && sizeof(recv_battery_4_to_1_active_report_status_with_wireless_type) != length){
           printf("recv battery length:%d error! sizeof struct length:%d, sizeof struct_with_wireless:%d\n",length,(int)sizeof(recv_battery_4_to_1_active_report_status_type),(int)sizeof(recv_battery_4_to_1_active_report_status_with_wireless_type));
           return -1;
       }
       if(length == sizeof(recv_battery_4_to_1_active_report_status_type)){
            recv_batterydata = (recv_battery_4_to_1_active_report_status_type *)data;
            pub_battery(recv_batterydata);
       }
        if(length == sizeof(recv_battery_4_to_1_active_report_status_with_wireless_type)){
            recv_batterydata_with_wireless = (recv_battery_4_to_1_active_report_status_with_wireless_type *)data;
            pub_battery_with_wireless(recv_batterydata_with_wireless);
       }
       break;
   case RECV_FROM_DRIVER_STATUS:
       recv_from_driver_status_type *recv_driverdata;
       if(sizeof(recv_from_driver_status_type) != length){
           printf("recv driver status length:%d error! sizeof struct length:%d\n",length,(int)sizeof(recv_from_driver_status_type));
           return -1;
       }
       recv_driverdata = (recv_from_driver_status_type *)data;
       pub_driverstatus(recv_driverdata);
       break;
   case RECV_FROM_ULTRASONIC_DATA:
        recv_ult_cnt++;
        recv_ult_cnt %= 65535; 
        if(0 == LIN_ult_installed){
            recv_from_ultrasonic_data *recv_ultdata;
            if(sizeof(recv_from_ultrasonic_data) != length){
                printf("recv ult data length:%d error! sizeof struct length:%d\n",length,(int)sizeof(recv_from_ultrasonic_data));
                return -1;
            }
            recv_ultdata = (recv_from_ultrasonic_data *)data;
            pub_ultrasonicdata(recv_ultdata);
        }else if(1 == LIN_ult_installed){
            recv_from_ultrasonic_data_lin *recv_ultdata;
            if(sizeof(recv_from_ultrasonic_data_lin) != length){
                printf("recv ult data length:%d error! sizeof struct length:%d\n",length,(int)sizeof(recv_from_ultrasonic_data_lin));
                return -1;
            }
            recv_ultdata = (recv_from_ultrasonic_data_lin *)data;
            pub_ultrasonicdata_lin(recv_ultdata);
        }else{
            printf("error:  LIN_ult_installed: %d  is wrong value, the right value is 0 or 1! ",LIN_ult_installed);
            return -1;
        }
       break;
   case UPD_CMD_210:
   case RECV_FROM_FIRMWARE_VERSION:
        Info("R_CMD_210: "<<1);
        recv_from_firmware_version_type *recv_version;
        if(sizeof(recv_from_firmware_version_type) != length){
           printf("recv version length:%d error! sizeof struct length:%d\n",length,(int)sizeof(recv_from_firmware_version_type));
           return -1;
       }
       recv_version=(recv_from_firmware_version_type *)data;
       pub_firmwareversion(recv_version);
       break;
   case RECV_FROM_CMD_ANSWER:
        recv_from_cmd_answer_type *recv_cmdanswer;  
        if(sizeof(recv_from_cmd_answer_type) != length){
            printf("recv command answer length:%d error! sizeof struct length:%d\n",length,(int)sizeof(recv_from_cmd_answer_type));
            return -1;
        }
        cmd_withid_recv_cnt++;
        recv_cmd_cnt++;
        recv_cmd_cnt %= 65535;
        recv_cmdanswer=(recv_from_cmd_answer_type *)data;
        compar_id_recv_send(recv_cmdanswer);  
        break;
   case RECV_FROM_RFID_INFO:
        recv_from_rfid_info_type *recv_rfidinfo; 
        if(sizeof(recv_from_rfid_info_type) != length){
        printf("recv from rfid info length:%d error! sizeof struct length:%d\n",length,(int)sizeof(recv_from_rfid_info_type));
        return -1;
        }
        recv_rfid_cnt++;
        recv_rfid_cnt %= 65535;  
        recv_rfidinfo = (recv_from_rfid_info_type *)data;
        pub_rfidinfo(recv_rfidinfo);
        break;
    case RECV_CHASSIS_ERROE_REPORT:
        recv_chassis_error_report_type *recv_chassiserror;
        if(sizeof(recv_chassis_error_report_type) != length){
            printf("recv from chassis error length:%d error! sizeof struct length:%d\n",length,(int)sizeof(recv_chassis_error_report_type));
            return -1;
        }
        recv_chassiserror = (recv_chassis_error_report_type *)data;
        pub_chassiserror(recv_chassiserror);
        break;  
    case RECV_NAVIGATION_LOG_STATUS:
        recv_navigation_log_status_type *recv_navigationlog;
        if(sizeof(recv_navigation_log_status_type) != length){
            printf("recv from navigation log  length:%d error! sizeof struct length:%d\n",length,(int)sizeof(recv_navigation_log_status_type));
            return -1;
        }
        recv_navigationlog = (recv_navigation_log_status_type *)data;
        pub_navigationlog(recv_navigationlog);
        break;    
    //SDK格式化是否成功的回复。0：成功
    case SEND_FORMAT_SD_CARD_CMD:
        send_format_sd_card_cmd_type *recv_formatsdcard;
        if(sizeof(send_format_sd_card_cmd_type) != length){
            printf("recv from navigation log length:%d error! sizeof struct length:%d\n",length,(int)sizeof(send_format_sd_card_cmd_type));
            return -1;
        }
        recv_formatsdcard = (send_format_sd_card_cmd_type *)data;
        pub_formatsdcard(recv_formatsdcard);
        break;
    case RECV_FROM_GPS:
        recv_gps_data_type *recv_gps;
        if(sizeof(recv_gps_data_type) != length){
            printf("recv from gps_data length:%d error! sizeof struct length:%d\n",length,(int)sizeof(recv_gps_data_type));
            return -1;
        }
        recv_gps = (recv_gps_data_type *)data;
        pub_gps(recv_gps);
        break;
    case UPD_CMD_206:
        get_version_flag = false;
        need_send_rough = true;//运运控重启之后，重新进行时间粗同步
        Info("R_CMD_206: "<<1);
        break;
    case TIME_SYNC_ROUGH_CMD:
        time_sync_rough_msg_type *recv_rough_res;
        if(sizeof(time_sync_rough_msg_type) != length){
            printf("recv from rough_time_response length:%d error! sizeof struct length:%d\n",length,(int)sizeof(time_sync_rough_msg_type));
            return -1;
        }
        recv_rough_res = (time_sync_rough_msg_type *)data;
        check_rough_res(recv_rough_res);
        break;
    case TIME_SYNC_FINE_CMD:
        //Info(" time_sync_successd: "<<1);
        if(sizeof(time_sync_fine_msg_type) != length){
            printf("recv from fine_time_response length:%d error! sizeof struct length:%d\n",length,(int)sizeof(time_sync_fine_msg_type));
            return -1;
        }
        break;
    case TIME_SYNC_DELAY_CMD:
        if(sizeof(time_sync_fine_msg_type) != length){
            printf("recv from delay_time_response length:%d error! sizeof struct length:%d\n",length,(int)sizeof(time_sync_fine_msg_type));
            return -1;
        }
        send_delay_sync_cmd();
        break;
    case RECV_FROM_DUSTBIN_INFO:
        if(robot_type == 0){
            dustbin_to_motion_t *recv_dustbin_state;
            if(sizeof(dustbin_to_motion_t) != length){
                printf("recv from dustbin_to_motion_t length:%d error! sizeof struct length:%d\n",length,(int)sizeof(dustbin_to_motion_t));
                return -1;
            }
            recv_dustbin_state = (dustbin_to_motion_t *)data;
            pub_dustbin_state(recv_dustbin_state);  
        }else if(robot_type == 1){
            clean_to_motion_t *recv_clean_state;
            clean_to_motion_t_new *recv_clean_state_new;
            serial_status.dustbin_state_recv_size = length;
            if(sizeof(clean_to_motion_t) == length){
                serial_status.recv_from_dustbin_state_old_cnt ++;
                serial_status.recv_from_dustbin_state_old_cnt %= 32000;
                recv_clean_state = (clean_to_motion_t *)data;
                pub_clean_state(recv_clean_state);
            }else if(sizeof(clean_to_motion_t_new) == length){
                serial_status.recv_from_dustbin_state_new_cnt++;
                serial_status.recv_from_dustbin_state_new_cnt %= 32000;
                recv_clean_state_new = (clean_to_motion_t_new *)data;
                pub_clean_state_new(recv_clean_state_new);
            }else{
                printf("recv from clean_to_motion_t length:%d error! sizeof struct length:old: %d or new: %d\n",length,(int)sizeof(clean_to_motion_t),(int)sizeof(clean_to_motion_t_new));
                return -1;
            }  
        }else{
            dustbin_to_motion_t *recv_dustbin_state;
            if(sizeof(dustbin_to_motion_t) != length){
                printf("recv from dustbin_to_motion_t length:%d error! sizeof struct length:%d\n",length,(int)sizeof(dustbin_to_motion_t));
                return -1;
            } 
            recv_dustbin_state = (dustbin_to_motion_t *)data;
            pub_dustbin_state(recv_dustbin_state);  
        }
        break;
    case DUSBIN_RF_SET_READ_CMD:
        dustbin_rf_set_cmd_t *recv_dustbin_id;
        if(sizeof(dustbin_rf_set_cmd_t) != length){
            printf("recv from dustbin_rf_set_cmd_t length:%d error! sizeof struct length:%d\n",length,(int)sizeof(dustbin_rf_set_cmd_t));
            return -1;
        }
        recv_dustbin_id = (dustbin_rf_set_cmd_t *)data;
        process_dustbin_id(recv_dustbin_id);
        break;
    case RECV_FROM_DUST_BOX_INFO:
        dust_box_to_motion_t_old *recv_dust_box_state_old;
        dust_box_to_motion_t_new *recv_dust_box_state_new;
        serial_status.dustbox_state_recv_size = length;
        if(sizeof(dust_box_to_motion_t_old) == length){
            serial_status.recv_from_dustbox_state_old_cnt++;
            serial_status.recv_from_dustbox_state_old_cnt %= 32000;
            recv_dust_box_state_old = (dust_box_to_motion_t_old*)data;
            pub_dust_box_state_old(recv_dust_box_state_old);
        }
        else if(sizeof(dust_box_to_motion_t_new) == length){
            serial_status.recv_from_dustbox_state_new_cnt++;
            serial_status.recv_from_dustbox_state_new_cnt %= 32000;
            recv_dust_box_state_new = (dust_box_to_motion_t_new*)data;
            pub_dust_box_state_new(recv_dust_box_state_new);
        }
        else{
            recv_dust_box_state_new = (dust_box_to_motion_t_new*)data;
            pub_dust_box_state_new(recv_dust_box_state_new);
            printf("recv from dust_box_to_motion_t length:%d error! sizeof struct length: old: %d or new: %d\n",length,(int)sizeof(dust_box_to_motion_t_old),(int)sizeof(dust_box_to_motion_t_new));
            return -1;
        }
        break;
    case RAIN_SENSOR:
        rain_sensor_t *recv_rain_sensor;
        rain_sensor_t_new *recv_rain_sensor_new;
        if(sizeof(rain_sensor_t) == length){
            recv_rain_sensor = (rain_sensor_t *)data;
            pub_rain_sensor(recv_rain_sensor);
        }else if(sizeof(rain_sensor_t_new) == length){
            recv_rain_sensor_new = (rain_sensor_t_new *)data;
            pub_rain_sensor_new(recv_rain_sensor_new);
        }else{
            printf("recv from rain_sensor_t length:%d error! sizeof struct old length:%d, new length:%d\n",length,(int)sizeof(rain_sensor_t),(int)sizeof(rain_sensor_t_new));
            return -1;
        }
        break;
    case SMART_TRASH_RECV:
        smart_trash_report *recv_smart_trash;
        if(sizeof(smart_trash_report) != length){
            printf("recv from smart_trash_report length:%d error! sizeof struct length:%d\n",length,(int)sizeof(smart_trash_report));
            return -1;
        }
        recv_smart_trash = (smart_trash_report *)data;
        pub_smart_trash(recv_smart_trash);
        break;
    case DUST_BOX_5G_RECV_CMD:
        dustbox_5g_recv_cmd_t *dustbox_5g_recv;
        if(sizeof(dustbox_5g_recv_cmd_t) != length){
            printf("recv from dustbox_5g_recv_cmd_t length:%d error! sizeof struct length:%d\n",length,(int)sizeof(dustbox_5g_recv_cmd_t));
            return -1;
        }
        dustbox_5g_recv = (dustbox_5g_recv_cmd_t *)data;
        pub_dustbox_5g_state(dustbox_5g_recv);
        break;
    case RECV_LIN_ULT_MODE:
        report_lin_ult_mode_type  *recv_ult_mode;
        if(sizeof(report_lin_ult_mode_type) != length){
            printf("recv from report_lin_ult_mode_type length:%d error! sizeof struct length:%d\n",length,(int)sizeof(report_lin_ult_mode_type));
            return -1;
        }
        recv_ult_mode = (report_lin_ult_mode_type*)data;
        process_ult_mode_report(recv_ult_mode);
        break;
    case UPD_CMD_212:
        upd_cmd_212_type *recv_212_respond;
        if(sizeof(upd_cmd_212_type) != length){
            printf("recv from upd_cmd_212_type length:%d error! sizeof struct length:%d\n",length,(int)sizeof(upd_cmd_212_type));
            return -1;
        }
        recv_212_respond = (upd_cmd_212_type*)data;
        process_212_respond(recv_212_respond);
        break;
    default:
       break;
   }
   return 0;
}

//************************************** 升级命令接收处理
int process_update_cmd_ex(unsigned char cmd, unsigned char *data, unsigned int length)
{
    if( (NULL == data) || (length > 510))
    {
        return -1;
    }
    cti_fpga_serial::updateinfo  upd_data;
    seq_num++;
    upd_data.seq_num = seq_num;
    for(int i=0;i<length+2;i++){
        upd_data.data.push_back(*(data+i));
    }
    stm32_pub.publish(upd_data);
}

//************************************** 定时器 代码更新超时处理
void timerCallback(const ros::TimerEvent& event)
{
    //--代码更新超时处理--
    if(stm32_update_flag && (++TIMEOUT) >= 5)
    {
        TIMEOUT = 0;
        stm32_update_flag = false;
    }
}

//************************************** 定时器2 查询版本号和发送时间同步
void timer2Callback(const ros::TimerEvent& event)
{
	//--查询版本号--
	if(get_version_flag == false)
	{
		send_to_check_version_type check_version_cmd;
		update_info_type update_info_struct;
		update_info_struct.src = MODULE_CHECK_UPD;
		update_info_struct.dest = MODULE_MOVE_CONTROL_BOARD;
		check_version_cmd.upd_info = update_info_struct;
		check_version_cmd.check = 01;
		construct_serial_frame_ex(&user_frame, SEND_TO_CHECK_PROGRAM_VERSION, sizeof(check_version_cmd), &check_version_cmd);
		// serial_frame_include_id_type user_frame_include_id_1;
		// user_frame_include_id_1.id = 0;
		// user_frame_include_id_1.cmd = SEND_TO_CHECK_PROGRAM_VERSION;
		// user_frame_include_id_1.need_id = 0;
		// user_frame_include_id_1.frame = user_frame;
		// pushData(&user_frame_include_id_1);
        pushData(construct_frame_include_id(0,SEND_TO_CHECK_PROGRAM_VERSION,0,user_frame));
        Info("S_CH_VE: "<<(int)MODULE_MOVE_CONTROL_BOARD);
	}

    if ((sendtime_loop_num >= 60 || need_resend_rough) && need_send_rough)
	{
        //获取年月日时分秒
        struct tm* local;
		time_t t;
		t = time(NULL);
		local = localtime(&t);
        //获取到微秒级别
        struct timeval tv;
	    struct timezone tz;
	    gettimeofday(&tv,&tz);

        time_sync_rough_msg_type timenow_cmd;

        timenow_cmd.year = local->tm_year;
        timenow_cmd.mon = local->tm_mon;
        timenow_cmd.day = local->tm_mday;
		timenow_cmd.hour = local->tm_hour;
		timenow_cmd.min = local->tm_min;
		timenow_cmd.sec = local->tm_sec;
		construct_serial_frame_ex(&user_frame, TIME_SYNC_ROUGH_CMD, sizeof(timenow_cmd), &timenow_cmd);
		// serial_frame_include_id_type user_frame_include_id_0;
		// user_frame_include_id_0.id = 0;
		// user_frame_include_id_0.cmd = TIME_SYNC_ROUGH_CMD;
		// user_frame_include_id_0.need_id = 0;
		// user_frame_include_id_0.frame = user_frame;
		// pushData(&user_frame_include_id_0);
        pushData(construct_frame_include_id(0,TIME_SYNC_ROUGH_CMD,0,user_frame));
		sendtime_loop_num= 0;
        need_resend_rough = false;
        sync_rough_msg_saved = timenow_cmd;
        check_rough_sync_res_timeoout = true;
        check_rough_sync_res_cnt = 0;
        Info("S_SYNC_ROUGH: "<<1);
	}
	sendtime_loop_num++;

    if(check_rough_sync_res_timeoout)
    {
        if(check_rough_sync_res_cnt >= 3 )
        {
            //Info(" recv_rough_sync_res_timeout: "<<1);
            need_resend_rough = true;
            check_rough_sync_res_timeoout = false;
            check_rough_sync_res_cnt = 0;
        }
        check_rough_sync_res_cnt++;
    }

    if(need_send_fine_sync)
    {
        if(send_fine_sync_cnt >= 60 )
        {
            send_fine_sync_cnt = 0;
            send_fine_sync_cmd();
        }
        send_fine_sync_cnt++;
    }
    
    cmd_answer_cnt.cmd_answer_success_num = cmd_answer_success_cnt;
    cmd_answer_cnt.cmd_answer_fail_num = cmd_withid_send_cnt-cmd_answer_success_cnt;
    cmd_answer_cnt.cmd_withid_send_num = cmd_withid_send_cnt;
    cmd_answer_pub.publish(cmd_answer_cnt);
    
  }

//************************************** 清扫箱命令重发函数<无用，本来设计为app控制清扫发送>
void resend_dustbin_ctrl_cmd()
{ 
    construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD, sizeof(resend_to_dustbin_cmd), &resend_to_dustbin_cmd);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,SEND_TO_DUSTBIN_CMD,0,user_frame));
    Info("RS_DBI_ES: " << (int)resend_to_dustbin_cmd.engin_start )
}

//************************************** 环卫车清扫命令重发函数<无用，本来设计为app控制清扫发送>
void resend_clean_ctrl_cmd()
{ 
    construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD, sizeof(resend_to_clean_cmd), &resend_to_clean_cmd);
    // serial_frame_include_id_type user_frame_include_id_1;
    // user_frame_include_id_1.id = 0;
    // user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
    // user_frame_include_id_1.need_id = 0;
    // user_frame_include_id_1.frame = user_frame;
    // pushData(&user_frame_include_id_1);
    pushData(construct_frame_include_id(0,SEND_TO_DUSTBIN_CMD,0,user_frame));
    Info("RS_CC_ES: " << (int)resend_to_clean_cmd.engin_start )
}

//************************************** 定时器3 命令应答超时处理
void timer3Callback(const ros::TimerEvent& event)
{
    //--命令应答超时处理
    if(!cmd_send_map.empty())
    {
        double endtime=ros::Time::now().toSec();
        double cmd_time_duration=0;
        std::map<uint16_t, double>::iterator it;
        it = cmd_send_map.begin();
        double starttime=it->second;
        cmd_time_duration = endtime - starttime;
        if (cmd_time_duration >= cmd_answer_timeout)
        {
            cmd_answer_fail_cnt++;
            //Info("RECV_ANSWER: "<<" id:"<<int(it->first)<<" cnt:"<<cmd_answer_fail_cnt);
            //cmd_send_map.erase(it);
            cmd_send_map.clear();
        }
    }
    //--清扫车命令应答处理 ，发送清扫车命令后，检查状态来判断命令是否执行。(0.02*50) 秒检查一次。如果状态不对，重发，最多重发三次。超过三次则超时
    static uint8_t check_clean_machine_status_cnt = 0;
    static uint8_t resend_clean_machine_cmd_cnt = 0;
    if(check_clean_mechine_state && check_clean_machine_status_cnt >= 20){
        if((resend_to_dustbin_cmd.engin_start == 1 && recv_clean_mechine_motor_status ==31) || 
        (resend_to_dustbin_cmd.engin_start == 0 && recv_clean_mechine_motor_status ==0)){
            check_clean_mechine_state = false;
            serial_status.control_clean_mechine = 0;
        }
        if((resend_to_dustbin_cmd.engin_start == 1 && recv_clean_mechine_motor_status !=31) || 
        (resend_to_dustbin_cmd.engin_start == 0 && recv_clean_mechine_motor_status !=0)){
            if(robot_type == 0){
                resend_dustbin_ctrl_cmd();
            }else if(robot_type == 1){
                resend_clean_ctrl_cmd();
            }else{
                resend_dustbin_ctrl_cmd();
            }
            resend_clean_machine_cmd_cnt++;
            if(resend_clean_machine_cmd_cnt>=5){
                check_clean_mechine_state = false;
                serial_status.control_clean_mechine = -1;
                resend_clean_machine_cmd_cnt = 0;
            }
        }
        check_clean_machine_status_cnt = 0;
    }
    check_clean_machine_status_cnt++;

}

//************************************** 定时器4 rfid接收超时，监控串口接收状态并发布话题
void timer4Callback(const ros::TimerEvent& event)
{
    //--rfid全部接收超时处理
    if(200 <= recvrfid_loop_num)
    {
        if(recv_rfid_cnt == recv_rfid_cnt_old)
        {
            rfid_all.states.clear();
            recv_rfid1_timeout = true;
            recv_rfid2_timeout = true;
            recv_rfid3_timeout = true;
            recv_rfid4_timeout = true;
            oldtabstate_rfid1.clear();
            cti_msgs::TabState tabstate;
            tabstate.id = 1;
            tabstate.status = 140;//140 means recv timeout!
            tabstate.message = "";
            oldTabstate_Pushback(&tabstate,tabstate.id);

            oldtabstate_rfid2.clear();
            tabstate.id = 2;
            oldTabstate_Pushback(&tabstate,tabstate.id);

            oldtabstate_rfid3.clear();
            tabstate.id = 3;
            oldTabstate_Pushback(&tabstate,tabstate.id);

            oldtabstate_rfid4.clear();
            tabstate.id = 4;
            oldTabstate_Pushback(&tabstate,tabstate.id);
            
            rfid_all.header.frame_id="rfid_all";
            rfid_all.states.insert(rfid_all.states.end(),oldtabstate_rfid1.begin(),oldtabstate_rfid1.end());
            rfid_all.states.insert(rfid_all.states.end(),oldtabstate_rfid2.begin(),oldtabstate_rfid2.end());
            rfid_all.states.insert(rfid_all.states.end(),oldtabstate_rfid3.begin(),oldtabstate_rfid3.end());  
            rfid_all.states.insert(rfid_all.states.end(),oldtabstate_rfid4.begin(),oldtabstate_rfid4.end());   
            boxrfid_pub_all.publish(rfid_all);
            rfid_all.states.clear();        
            Info("R_RF_TMO: "<<1);
            serial_status.data = serial_status.data | 0x0002; //如果0.02*200 = 4秒內不能接收到rfid,状态位倒数第二位置1
            serial_status.recv_rfid = -1;
        }
        else
        {
            serial_status.data = serial_status.data & 0xFFFD;//如果rfid接收正常 ，状态位倒数第二位重置 为0
            serial_status.recv_rfid = 0;
        }
        recv_rfid_cnt_old = recv_rfid_cnt;
        recvrfid_loop_num = 0;
    }
    recvrfid_loop_num++;
    //--控制信息接收超时处理 0.1s
    if (5 <= recvctrl_loop_num)
    {
        if(recv_ctrl_cnt == recv_ctrl_cnt_old)
        {
            serial_status.data = serial_status.data | 0x0001; //如果控制信息上传 0.1s內没上传，状态位倒数第一位置1
            serial_status.recv_ctrl = -1;
        }
        else
        {   
            serial_status.data = serial_status.data & 0xFFFE;//如果控制信息正常，状态位倒数第一位置0
            serial_status.recv_ctrl = 0;
        }
        recv_ctrl_cnt_old = recv_ctrl_cnt;
        recvctrl_loop_num = 0;
    }
    recvctrl_loop_num++;
    //--控制信息频率降低处理 1s
    if (50 <= recvctrl_rate_loop_num)
    {
        float recv_ctrl_rate = abs(recv_ctrl_rate_cnt - recv_ctrl_rate_cnt_old) / 1 ;
        if(recv_ctrl_rate > 10000)
        {
            recv_ctrl_rate = 25;
        }
        node_status_publisher_ptr_->CHECK_MIN_VALUE("/value/cti_fpga_serial/recv_ctrl_rate",recv_ctrl_rate,20,15,"value recv_ctrl_rate is too low");
        Info("R_CT_RA: "<<recv_ctrl_rate);
        //printf("recv_ctrl_rate:%f\n",recv_ctrl_rate);
        if (recv_ctrl_rate <= RECV_CTRL_RATE_MIN)
        {
            serial_status.data = serial_status.data | 0x0010; //如果控制信息上传频率小于RECV_CTRL_RATE,状态位倒数第五位置1
            serial_status.recv_ctrl_rate = -1;
        }
        else
        {
            serial_status.data = serial_status.data & 0xFFEF;//如果控制信息上传频率大于RECV_CTRL_RATE，状态位倒数第五位置0
            serial_status.recv_ctrl_rate = 0;
        }
        recv_ctrl_rate_cnt_old = recv_ctrl_rate_cnt;
        recvctrl_rate_loop_num = 0;
    }
    recvctrl_rate_loop_num++;
    //--超声波接收超时处理 1s
    if (50 <= recvult_loop_num)
    {
        if(recv_ult_cnt == recv_ult_cnt_old)
        {    
            serial_status.data = serial_status.data | 0x0004;//如果超声波信息上传 1秒内没有上传，状态位倒数第3位重置1
            serial_status.recv_ult = -1;
        }
        else
        {
            serial_status.data = serial_status.data & 0xFFFB;//如果超声波信息上传正常，状态位倒数第三位重置0
            serial_status.recv_ult = 0;
        }
        recv_ult_cnt_old = recv_ult_cnt;
        recvult_loop_num = 0;
    }
    recvult_loop_num++;
    //--命令应答接收超时处理 0.5s
    if (25 <= recvcmd_loop_num)
    {
        if(recv_cmd_cnt == recv_cmd_cnt_old)
        {
            serial_status.data = serial_status.data | 0x0008;//如果命令应答0.5s内没有上传，状态位倒数第四位重置1
            serial_status.recv_cmd_answer = -1;
        }
        else
        {
            serial_status.data = serial_status.data & 0xFFF7;//如果命令应答正常，状态位倒数第四位重置0
            serial_status.recv_cmd_answer = 0;
        }
        recv_cmd_cnt_old = recv_cmd_cnt;
        recvcmd_loop_num = 0;
    }
    recvcmd_loop_num++;
    //--recv_pthread线程的状态信息处理 0.1s 
    if (5 <= recv_pthread_loop_num)
    {
        if(recv_pthread_cnt == recv_pthread_cnt_old)
        {
            serial_status.data = serial_status.data | 0x0040;//如果数据接收线程1s内没有接到新数据，状态倒数第七位置1
            serial_status.recv_pthread_recv = -1;
        }
        else
        {
            serial_status.data = serial_status.data & 0xFFBF;//如果数据接收线程正常，状态倒数第七位置0
            serial_status.recv_pthread_recv = 0;
        }
        recv_cmd_cnt_old = recv_cmd_cnt;
        recvcmd_loop_num = 0;
        
        if(recv_pthread_crc_status == 1)
        {
            serial_status.data = serial_status.data | 0x0080;//如果数据接受CRC错误，状态倒数第八位置1
            serial_status.recv_pthread_crc = -1;
        }
        else
        {
            serial_status.data = serial_status.data & 0xFF7F;;//如果数据接收线程CRC正确，状态倒数第八位置0
            serial_status.recv_pthread_crc = 0;
        }
    }
    recv_pthread_loop_num++;

    if (control_version_right != 0)
    {
        serial_status.data = serial_status.data | 0x0100;//如果运控版本错误，状态倒数第九位置1
        serial_status.control_board_version = control_version_right;
    }
    else
    {
        serial_status.data = serial_status.data & 0xFEFF;;//如果运控版本正确，状态倒数第九位置0
        serial_status.control_board_version = 0;
    }
    //--状态信息发布(状态改变时)
    if(old_serial_status != serial_status.data)
    {
        serial_status_pub.publish(serial_status);   
        old_serial_status = serial_status.data;
        Info("SE_ST: "<<(int)serial_status.data);
    } 
    //-- 状态信息发布(定时发布)
    static uint8_t pub_status_num_loop =0;
    if(20 <= pub_status_num_loop){
        serial_status_pub.publish(serial_status); 
        pub_status_num_loop = 0;
        Info("SE_ST: "<<(int)serial_status.data);
    }
    pub_status_num_loop++;
    //--顶升测试命令超时 1s
    static uint16_t recv_lift_test_num_old =0;
    static uint16_t recv_lift_test_num_loop = 0;
    if (50 <= recv_lift_test_num_loop)
    {
        if(recv_lift_test_num == recv_lift_test_num_old)
        {
            lift_test_flag = false;
        }
        recv_lift_test_num_old = recv_lift_test_num;
        recv_lift_test_num_loop = 0;
    }
    recv_lift_test_num_loop++;
}

void timer6Callback(const ros::TimerEvent& event)
{
    //节点心跳和数据接收状态记录
    std_msgs::UInt8 nodeStatus;
    nodeStatus.data = 1;
    node_status_pub.publish(nodeStatus);
    Info(" BEAT: "<<1
        <<" R_DATA_CNT: "<<recv_data_cnt
        <<" R_RET: "<<recv_ret
        <<" R_CRC_ST: "<<recv_crc_status);
    //printf("recv_data_cnt: [%d], recv_ret: [%d], R_CRC_ST:[%d]\n",recv_data_cnt,recv_ret,recv_crc_status);
}   

double get_duration(double in_time){
    return ros::Time::now().toSec() - in_time;
}

//lora_id设置定时器
void timer7Callback(const ros::TimerEvent& event)
{
    //lora id设置状态发布
    cti_fpga_serial::LoraIdSetState lora_id_set_state;
    lora_id_set_state.need_setting = dustbin_set_state.need_setting;
    lora_id_set_state.send_id = dustbin_set_state.send_id;
    lora_id_set_state.recv_id = dustbin_set_state.recv_id;
    lora_id_set_state.setting_start_time = dustbin_set_state.setting_start_time;
    lora_id_set_state.set_read_switch = dustbin_set_state.set_read_switch;
    lora_id_set_state.set_read_switch_time = dustbin_set_state.set_read_switch_time;
    lora_id_set_state.set_timeout = dustbin_set_state.set_timeout;
    lora_id_set_state_pub.publish(lora_id_set_state);
    int need_setting  = lora_id_set_state.need_setting?1:0;
    int set_read_switch = dustbin_set_state.set_read_switch?1:0;
    Info("LOID_ST: "<<need_setting
    <<" LOID_SID: "<<dustbin_set_state.send_id
    <<" LOID_RID: "<<dustbin_set_state.recv_id
    <<" LOID_STI: "<<dustbin_set_state.setting_start_time
    <<" LOID_SRS: "<<set_read_switch
    <<" LOID_SRST: "<<dustbin_set_state.set_read_switch_time
    <<" LOID_STIO: "<<dustbin_set_state.set_timeout);

    cti_msgs::TabState set_dustbin_id_state;
    set_dustbin_id_state.id = dustbin_set_state.send_id;
    set_dustbin_id_state.name = "set dustbin id state";

    //如果发送和接受相同，设置成功
    if(dustbin_set_state.need_setting && dustbin_set_state.send_id == dustbin_set_state.recv_id){
        dustbin_set_state.need_setting = false;
        set_dustbin_id_state.status = 1; //1: 设置成功
        set_dustbin_id_state.message = "set successed!";
        set_dustbin_id_state_pub.publish(set_dustbin_id_state);
        Info("设置设定rola模块成功");
    }
    //如果需要设定，且轮到发送设置,发送设定
    if( dustbin_set_state.need_setting  
        && dustbin_set_state.set_read_switch
        && get_duration(dustbin_set_state.set_read_switch_time) > 2){
        dustbin_set_state.set_read_switch = !dustbin_set_state.set_read_switch;
        dustbin_set_state.set_read_switch_time = ros::Time::now().toSec();
        dustbin_rf_set_cmd_t send_set_dustbin_id;
        send_set_dustbin_id.rw = 1;
        send_set_dustbin_id.baud = -1; //一定要设置成-1 否则会修改运控和lora通信的波特率
        send_set_dustbin_id.id = dustbin_set_state.send_id;
        
        construct_serial_frame_ex(&user_frame, DUSBIN_RF_SET_READ_CMD, sizeof(send_set_dustbin_id), &send_set_dustbin_id);
        // serial_frame_include_id_type user_frame_include_id_1;
        // user_frame_include_id_1.id = 0;
        // user_frame_include_id_1.cmd = DUSBIN_RF_SET_READ_CMD;
        // user_frame_include_id_1.need_id = 0;
        // user_frame_include_id_1.frame = user_frame;
        // pushData(&user_frame_include_id_1);
        pushData(construct_frame_include_id(0,DUSBIN_RF_SET_READ_CMD,0,user_frame));
        Info("S_SID: " << (int)dustbin_set_state.send_id);
    }
    //如果需要设定，且轮到发送查询,发送查询
    if(dustbin_set_state.need_setting  
        && !dustbin_set_state.set_read_switch
        && get_duration(dustbin_set_state.set_read_switch_time) > 2){
        dustbin_set_state.set_read_switch = !dustbin_set_state.set_read_switch;
        dustbin_set_state.set_read_switch_time = ros::Time::now().toSec();
        dustbin_rf_set_cmd_t send_set_dustbin_id;
        send_set_dustbin_id.rw = 0;
        send_set_dustbin_id.baud = -1; //一定要设置成-1 否则会修改运控和lora通信的波特率

        construct_serial_frame_ex(&user_frame, DUSBIN_RF_SET_READ_CMD, sizeof(send_set_dustbin_id), &send_set_dustbin_id);
        // serial_frame_include_id_type user_frame_include_id_1;
        // user_frame_include_id_1.id = 0;
        // user_frame_include_id_1.cmd = DUSBIN_RF_SET_READ_CMD;
        // user_frame_include_id_1.need_id = 0;
        // user_frame_include_id_1.frame = user_frame;
        // pushData(&user_frame_include_id_1);
        pushData(construct_frame_include_id(0,DUSBIN_RF_SET_READ_CMD,0,user_frame));
        Info("S_RID: "<<1);
    }
    //设置超时
    if(dustbin_set_state.need_setting && get_duration(dustbin_set_state.setting_start_time) > dustbin_set_state.set_timeout){ 
        dustbin_set_state.need_setting == false;
        Info("S_SID_TMO: " << (int)dustbin_set_state.send_id);
        //发布状态
        set_dustbin_id_state.status = -2; //-2: 超时
        set_dustbin_id_state.message = "set failed ,timeout!";
        set_dustbin_id_state_pub.publish(set_dustbin_id_state);
    }


    //箱子类型发生改变,则一秒发一次set_dustbin_on_car直到设置成功
    static uint8_t set_dustbin_on_car_times = 0; 
    if(set_dustbin_on_car && robot_type == 0){
        //切换箱子时发送指令
        if(robot_type == 0){
            if(4 == box_type){
                send_to_dustbin_cmd.dustbin_on_car = 1;//清扫箱在车上
            }else{
                send_to_dustbin_cmd.dustbin_on_car = 0;
            }
            construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD, sizeof(send_to_dustbin_cmd), &send_to_dustbin_cmd);  
        }
        construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD, sizeof(send_to_dustbin_cmd), &send_to_dustbin_cmd);
        // serial_frame_include_id_type user_frame_include_id_1;
        // user_frame_include_id_1.id = 0;
        // user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
        // user_frame_include_id_1.need_id = 0;
        // user_frame_include_id_1.frame = user_frame;
        // pushData(&user_frame_include_id_1);
        pushData(construct_frame_include_id(0,SEND_TO_DUSTBIN_CMD,0,user_frame));

        Info("S_BT: "<< (int)box_type);
        //printf("setting_box_type: %d, set_times: %d\n", box_type,set_dustbin_on_car_times);
        set_dustbin_on_car_times++;
        if (set_dustbin_on_car_times >= 5){
            set_dustbin_on_car_times = 0;
            set_dustbin_on_car = false;
            Info("SBT_TMO: "<<1);
        }
    }
}


//通讯超时检测定时器
void timer_chat_timeout_Callbak(const ros::TimerEvent& event){
    static uint8_t in_cnt = 0;
    //开机10s后才开始检测
    if(in_cnt < 10){
        in_cnt++;
        return;
    }

    cti_msgs::BoxState communicate_msg;
    communicate_msg.header.stamp = ros::Time::now();
    cti_msgs::TabState chassis_state;
    chassis_state.id = 0;
    cti_msgs::TabState dustbox_state;
    dustbox_state.id = 1;
    //含义定义:
    //id: 模块 0:底盘通讯 1:箱子通讯
    //status: 0:空闲 1:通讯正常 -1:通讯超时
    //message: free:空闲; ok:通讯正常; timeout:通讯超时;

    //底盘通讯检测
    if(get_duration(chassis_chat_state.time_recv) > chassis_chat_timeout_secs){
        chassis_chat_state.recv_state = false;
    }
    if(!chassis_chat_state.recv_state){
        //底盘通讯异常
        chassis_state.status = -1;
        chassis_state.message = "timeout";
    }else{
        //底盘通讯正常
        chassis_state.status = 1;
        chassis_state.message = "ok";
    } 
    //箱子通讯检测
    if(get_duration(box_chat_state.time_recv) > box_chat_timeout_secs){
        box_chat_state.recv_state = false;
    }
    //箱子通讯超过5s，重新设定箱子超时。
    if(get_duration(box_chat_state.time_recv) > box_chat_timeout_secs/2){
        dustbin_set_state.need_setting = true;
        dustbin_set_state.set_read_switch = true;
        Info("箱子通讯超时5s，重新设定rola模块");
    }
    if(dustbox_lora_id != -1){
        if(!box_chat_state.recv_state){
            //箱子通讯异常
            dustbox_state.status = -1;
            dustbox_state.message = "timeout";
        }else{
            //箱子通讯正常
            dustbox_state.status = 1;
            dustbox_state.message = "ok";
        }
    }else{
        //lora_id没有设置
        //发布通讯空闲状态
        box_chat_state.reset();
        dustbox_state.status = 0;
        dustbox_state.message = "free";
    }
    communicate_msg.states.push_back(chassis_state);
    Info("VEH_CHAT: "<< (int)chassis_state.status);
    communicate_msg.states.push_back(dustbox_state);
    Info("DBX_ID: "<< (int)box_chat_state.module_id);
    Info("DBX_CHAT: "<< (int)dustbox_state.status);
    chat_statue_pub.publish(communicate_msg);
}

//---------------检查清扫功能测试是否超时的定时器
void timer_clean_function_test_timeout_Callback(const ros::TimerEvent& event){
    //发布结构体，为了方便查看
    cti_fpga_serial::cleanFuncTestState clean_function_test_state_msg;
    clean_function_test_state_msg.clean_function_enable  = clean_function_test_state.clean_function_enable;
    clean_function_test_state_msg.time_recv  = clean_function_test_state.time_recv;
    clean_function_test_state_msg.timeout_secs  = clean_function_test_state.timeout_secs;
    if(clean_function_test_state.clean_function_enable){
        clean_function_test_state_msg.test_time_left = clean_function_test_state_msg.timeout_secs-(ros::Time::now().toSec() - clean_function_test_state_msg.time_recv);
    }else{
        clean_function_test_state_msg.test_time_left = clean_function_test_state_msg.timeout_secs;
    }
    clean_function_test_state_msg.damboard_status  = clean_function_test_state.damboard_status;
    clean_function_test_state_msg.side_brush_transform_state  = clean_function_test_state.side_brush_transform_state;
    clean_function_test_state_msg.side_brush_speed  = clean_function_test_state.side_brush_speed;
    clean_function_test_state_msg.spray_motor  = clean_function_test_state.spray_motor;
    clean_function_test_state_msg.fanspeed = clean_function_test_state.fanspeed;
    clean_function_test_state_msg.recv_damboard_status  = clean_function_test_state.recv_damboard_status;
    clean_function_test_state_msg.recv_side_brush_transform_state  = clean_function_test_state.recv_side_brush_transform_state;
    clean_function_test_state_msg.recv_side_brush_speed  = clean_function_test_state.recv_side_brush_speed;
    clean_function_test_state_msg.recv_spray_motor  = clean_function_test_state.recv_spray_motor;
    clean_function_test_state_msg.recv_fanspeed  = clean_function_test_state.recv_fanspeed;
    clean_function_test_state_pub.publish(clean_function_test_state_msg);

    if(clean_function_test_state.clean_function_enable &&
    get_duration(clean_function_test_state.time_recv) > clean_function_test_state.timeout_secs){
       //如果测试时间到达发布测试结果
        cti_msgs::DataArray cleanFuncTestReports;
        cleanFuncTestReports.header.stamp = ros::Time::now();
        cleanFuncTestReports.header.frame_id = "clean_function_test_report";
        cti_msgs::Data cleanFuncTestReport;

        //挡板状态数值
        cleanFuncTestReport.name = "damboard_state_value";
        cleanFuncTestReport.data = std::to_string(clean_function_test_state.recv_damboard_status);
        cleanFuncTestReport.type = cti_msgs::Data::TYPE_UINT8;
        cleanFuncTestReports.datas.push_back(cleanFuncTestReport); 
        //挡板测试结果
        cleanFuncTestReport.name = "damboard_state_result";
        cleanFuncTestReport.type = cti_msgs::Data::TYPE_STRING;
        if(clean_function_test_state.recv_damboard_status == clean_function_test_state.damboard_status){
            cleanFuncTestReport.data = "PASS";
        }else{
            cleanFuncTestReport.data = "NG";
        }
        cleanFuncTestReports.datas.push_back(cleanFuncTestReport); 

        //边刷伸展
        cleanFuncTestReport.name = "sidebrush_transform_value";
        cleanFuncTestReport.data = std::to_string( clean_function_test_state.recv_side_brush_transform_state);
        cleanFuncTestReport.type = cti_msgs::Data::TYPE_UINT8;
        cleanFuncTestReports.datas.push_back(cleanFuncTestReport); 
        //边刷伸展结果
        cleanFuncTestReport.name = "sidebrush_transform_result";
        cleanFuncTestReport.type = cti_msgs::Data::TYPE_STRING;
        if(abs(clean_function_test_state.recv_side_brush_transform_state - clean_function_test_state.side_brush_transform_state) <= 3){
            cleanFuncTestReport.data = "PASS";
        }else{
            cleanFuncTestReport.data = "NG";
        }
        cleanFuncTestReports.datas.push_back(cleanFuncTestReport); 

        //边刷转速
        cleanFuncTestReport.name = "side_brush_speed_value";
        cleanFuncTestReport.data = std::to_string( clean_function_test_state.recv_side_brush_speed);
        cleanFuncTestReport.type = cti_msgs::Data::TYPE_UINT8;
        cleanFuncTestReports.datas.push_back(cleanFuncTestReport); 
        //边刷转速结果
        cleanFuncTestReport.name = "side_brush_speed_report";
        cleanFuncTestReport.type = cti_msgs::Data::TYPE_STRING;
        if(clean_function_test_state.recv_side_brush_speed == clean_function_test_state.side_brush_speed){
            cleanFuncTestReport.data = "PASS";
        }else{
            cleanFuncTestReport.data = "NG";
        }
        cleanFuncTestReports.datas.push_back(cleanFuncTestReport); 
        
        //喷水电机
        cleanFuncTestReport.name = "spray_motor_value";
        cleanFuncTestReport.data = std::to_string(clean_function_test_state.recv_spray_motor);
        cleanFuncTestReport.type = cti_msgs::Data::TYPE_UINT8;
        cleanFuncTestReports.datas.push_back(cleanFuncTestReport); 
        ////喷水电机结果
        cleanFuncTestReport.name = "spray_motor_report";
        cleanFuncTestReport.type = cti_msgs::Data::TYPE_STRING;
        if(clean_function_test_state.recv_spray_motor == clean_function_test_state.spray_motor){
            cleanFuncTestReport.data = "PASS";
        }else{
            cleanFuncTestReport.data = "NG";
        }
        cleanFuncTestReports.datas.push_back(cleanFuncTestReport); 

        //风机速度
        cleanFuncTestReport.name = "fanspeed_value";
        cleanFuncTestReport.data = std::to_string(clean_function_test_state.recv_fanspeed);
        cleanFuncTestReport.type = cti_msgs::Data::TYPE_UINT8;
        cleanFuncTestReports.datas.push_back(cleanFuncTestReport); 
        //风机速度结果
        cleanFuncTestReport.name = "fanspeed_report";
        cleanFuncTestReport.type = cti_msgs::Data::TYPE_STRING;
        if(abs(clean_function_test_state.recv_fanspeed - clean_function_test_state.fanspeed) <= 3){
            cleanFuncTestReport.data = "PASS";
        }else{
            cleanFuncTestReport.data = "NG";
        }
        cleanFuncTestReports.datas.push_back(cleanFuncTestReport); 

        //发布测试结果
        clean_function_test_report_pub.publish(cleanFuncTestReports);
        //状态重置
        clean_function_test_state.reset();
    }
}

//超声波模式设置定时器
void timer_set_ult_mode_Callback(const ros::TimerEvent& event){

    // printf("--------------------vehicle ult_state--------------------\n");
    // printf("vehicle_ult_set_state.state: %d\n",vehicle_ult_set_state.state);
    // printf("vehicle_ult_set_state.set_mode: 0x%016lx\n",vehicle_ult_set_state.set_mode);
    // printf("vehicle_ult_set_state.set_start_time: %lf\n",vehicle_ult_set_state.set_start_time);
    // printf("vehicle_ult_set_state.recv_mode: 0x%016lx\n",vehicle_ult_set_state.recv_mode);
    // printf("vehicle_ult_set_state.check_start_time: %f\n",vehicle_ult_set_state.check_start_time);
    // printf("--------------------dustbox ult_state--------------------\n");
    // printf("dustbox_ult_set_state.state: %d\n",dustbox_ult_set_state.state);
    // printf("dustbox_ult_set_state.set_mode: 0x%016lx\n",dustbox_ult_set_state.set_mode);
    // printf("dustbox_ult_set_state.set_start_time: %lf\n",dustbox_ult_set_state.set_start_time);
    // printf("dustbox_ult_set_state.recv_mode: 0x%016lx\n",dustbox_ult_set_state.recv_mode);
    // printf("dustbox_ult_set_state.check_start_time: %f\n",dustbox_ult_set_state.check_start_time);
    cti_fpga_serial::ultModeSetState vehicle_ult_set_state_msg;
    vehicle_ult_set_state_msg.state = vehicle_ult_set_state.state;
    vehicle_ult_set_state_msg.set_mode = vehicle_ult_set_state.set_mode;
    vehicle_ult_set_state_msg.set_start_time = vehicle_ult_set_state.set_start_time;
    vehicle_ult_set_state_msg.recv_mode = vehicle_ult_set_state.recv_mode;
    vehicle_ult_set_state_msg.check_start_time = vehicle_ult_set_state.check_start_time;
    vehicle_ult_set_state_pub.publish(vehicle_ult_set_state_msg);

    cti_fpga_serial::ultModeSetState dustbox_ult_set_state_msg;
    dustbox_ult_set_state_msg.state = dustbox_ult_set_state.state;
    dustbox_ult_set_state_msg.set_mode = dustbox_ult_set_state.set_mode;
    dustbox_ult_set_state_msg.set_start_time = dustbox_ult_set_state.set_start_time;
    dustbox_ult_set_state_msg.recv_mode = dustbox_ult_set_state.recv_mode;
    dustbox_ult_set_state_msg.check_start_time = dustbox_ult_set_state.check_start_time;
    dustbox_ult_set_state_pub.publish(dustbox_ult_set_state_msg);

    //车身超声波处理
    if(vehicle_ult_set_state.state == 0 || vehicle_ult_set_state.state == 127){
        //空闲或初始化 do nothing
        vehicle_ult_set_state.check_start_time = 0.0;
        vehicle_ult_set_state.set_start_time = 0.0;
    }
    else if(vehicle_ult_set_state.state == 1){
        //发送查询命令
        check_lin_ult_mode_type  vehicle_send_check_ult_mode;
        vehicle_send_check_ult_mode.src = MODULE_CONTROL_PC;
        vehicle_send_check_ult_mode.dest = MODULE_ULTRASONIC_MASTER;
        vehicle_send_check_ult_mode.check_ult_Serial = 0xffffffffffffffff; //全部查询
        construct_serial_frame_ex(&user_frame, CHECK_LIN_ULT_MODE, sizeof(vehicle_send_check_ult_mode), &vehicle_send_check_ult_mode);
        // serial_frame_include_id_type user_frame_include_id_1;
        // user_frame_include_id_1.id = 0;
        // user_frame_include_id_1.cmd = CHECK_LIN_ULT_MODE;
        // user_frame_include_id_1.need_id = 0;
        // user_frame_include_id_1.frame = user_frame;
        // pushData(&user_frame_include_id_1);
        pushData(construct_frame_include_id(0,CHECK_LIN_ULT_MODE,0,user_frame));

        vehicle_ult_set_state.check_start_time = ros::Time::now().toSec();
        //更改状态
        vehicle_ult_set_state.state = 2;
    }
    else if(vehicle_ult_set_state.state == 2){ 
        //等待查询结果
        double time_now = ros::Time::now().toSec();
        if(time_now - vehicle_ult_set_state.check_start_time > 3){
            vehicle_ult_check_resend_time++;
            if(vehicle_ult_check_resend_time <= 5){
                //发送查询命令
                check_lin_ult_mode_type  vehicle_send_check_ult_mode;
                vehicle_send_check_ult_mode.src = MODULE_CONTROL_PC;
                vehicle_send_check_ult_mode.dest = MODULE_ULTRASONIC_MASTER;
                vehicle_send_check_ult_mode.check_ult_Serial = 0xffffffffffffffff; //全部查询
                // construct_serial_frame_ex(&user_frame, CHECK_LIN_ULT_MODE, sizeof(vehicle_send_check_ult_mode), &vehicle_send_check_ult_mode);
                // serial_frame_include_id_type user_frame_include_id_1;
                // user_frame_include_id_1.id = 0;
                // user_frame_include_id_1.cmd = CHECK_LIN_ULT_MODE;
                // user_frame_include_id_1.need_id = 0;
                // user_frame_include_id_1.frame = user_frame;
                // pushData(&user_frame_include_id_1);
                pushData(construct_frame_include_id(0,CHECK_LIN_ULT_MODE,0,user_frame));
                vehicle_ult_set_state.check_start_time = ros::Time::now().toSec();
            }else{
                vehicle_ult_set_state.state = -1; //查询超时
                vehicle_ult_check_resend_time = 0;
            }
        } 
    }
    else if(vehicle_ult_set_state.state == 3){
        vehicle_ult_check_resend_time = 0;
        //发送设置命令
        set_lin_ult_mode_type  vehicle_send_set_ult_mode;
        vehicle_send_set_ult_mode.src = MODULE_CONTROL_PC;
        vehicle_send_set_ult_mode.dest = MODULE_ULTRASONIC_MASTER;
        vehicle_send_set_ult_mode.mode.data = vehicle_ult_set_state.set_mode; //设置模式
        construct_serial_frame_ex(&user_frame, SET_LIN_ULT_MODE, sizeof(vehicle_send_set_ult_mode), &vehicle_send_set_ult_mode);
        // serial_frame_include_id_type user_frame_include_id_1;
        // user_frame_include_id_1.id = 0;
        // user_frame_include_id_1.cmd = SET_LIN_ULT_MODE;
        // user_frame_include_id_1.need_id = 0;
        // user_frame_include_id_1.frame = user_frame;
        // pushData(&user_frame_include_id_1);
        pushData(construct_frame_include_id(0,SET_LIN_ULT_MODE,0,user_frame));
        vehicle_ult_set_state.set_start_time = ros::Time::now().toSec();
        //更改状态
        vehicle_ult_set_state.state = 4;
    }
    else if(vehicle_ult_set_state.state == 4){
        //等待设置结果
        double time_now = ros::Time::now().toSec();
        if(time_now - vehicle_ult_set_state.set_start_time > 3){
            vehicle_ult_set_resend_time++;
            if(vehicle_ult_set_resend_time <= 5){
                //发送设置命令
                set_lin_ult_mode_type  vehicle_send_set_ult_mode;
                vehicle_send_set_ult_mode.src = MODULE_CONTROL_PC;
                vehicle_send_set_ult_mode.dest = MODULE_ULTRASONIC_MASTER;
                vehicle_send_set_ult_mode.mode.data = vehicle_ult_set_state.set_mode; //设置模式
                construct_serial_frame_ex(&user_frame, SET_LIN_ULT_MODE, sizeof(vehicle_send_set_ult_mode), &vehicle_send_set_ult_mode);
                // serial_frame_include_id_type user_frame_include_id_1;
                // user_frame_include_id_1.id = 0;
                // user_frame_include_id_1.cmd = SET_LIN_ULT_MODE;
                // user_frame_include_id_1.need_id = 0;
                // user_frame_include_id_1.frame = user_frame;
                // pushData(&user_frame_include_id_1);
                pushData(construct_frame_include_id(0,SET_LIN_ULT_MODE,0,user_frame));
                vehicle_ult_set_state.set_start_time = ros::Time::now().toSec();
            }else{
                vehicle_ult_set_state.state = -2; //设置超时
                vehicle_ult_set_resend_time = 0;
            }
        } 
    }



    //吸尘箱超声波处理
    if(dustbox_ult_set_state.state == 0 || dustbox_ult_set_state.state == 127){
        //空闲或初始化 do nothing
        dustbox_ult_set_state.check_start_time = 0.0;
        dustbox_ult_set_state.set_start_time = 0.0;
    }
    else if(dustbox_ult_set_state.state == 1){
        //发送查询命令
        check_lin_ult_mode_type  dustbox_send_check_ult_mode;
        dustbox_send_check_ult_mode.src = MODULE_CONTROL_PC;
        dustbox_send_check_ult_mode.dest = MODULE_DUST_BOX_BOARD;
        dustbox_send_check_ult_mode.check_ult_Serial = 0xffffffffffffffff; //全部查询
        construct_serial_frame_ex(&user_frame, CHECK_LIN_ULT_MODE, sizeof(dustbox_send_check_ult_mode), &dustbox_send_check_ult_mode);
        // serial_frame_include_id_type user_frame_include_id_1;
        // user_frame_include_id_1.id = 0;
        // user_frame_include_id_1.cmd = CHECK_LIN_ULT_MODE;
        // user_frame_include_id_1.need_id = 0;
        // user_frame_include_id_1.frame = user_frame;
        // pushData(&user_frame_include_id_1);
        pushData(construct_frame_include_id(0,CHECK_LIN_ULT_MODE,0,user_frame));
        dustbox_ult_set_state.check_start_time = ros::Time::now().toSec();
        //更改状态
        dustbox_ult_set_state.state = 2;
    }
    else if(dustbox_ult_set_state.state == 2){ 
        //等待查询结果
        double time_now = ros::Time::now().toSec();
        if(time_now - dustbox_ult_set_state.check_start_time > 3){
            dustbox_ult_check_resend_time++;
            if(dustbox_ult_check_resend_time <= 5){
                //发送查询命令
                check_lin_ult_mode_type  dustbox_send_check_ult_mode;
                dustbox_send_check_ult_mode.src = MODULE_CONTROL_PC;
                dustbox_send_check_ult_mode.dest = MODULE_DUST_BOX_BOARD;
                dustbox_send_check_ult_mode.check_ult_Serial = 0xffffffffffffffff; //全部查询
                construct_serial_frame_ex(&user_frame, CHECK_LIN_ULT_MODE, sizeof(dustbox_send_check_ult_mode), &dustbox_send_check_ult_mode);
                // serial_frame_include_id_type user_frame_include_id_1;
                // user_frame_include_id_1.id = 0;
                // user_frame_include_id_1.cmd = CHECK_LIN_ULT_MODE;
                // user_frame_include_id_1.need_id = 0;
                // user_frame_include_id_1.frame = user_frame;
                // pushData(&user_frame_include_id_1);
                pushData(construct_frame_include_id(0,CHECK_LIN_ULT_MODE,0,user_frame));
                dustbox_ult_set_state.check_start_time = ros::Time::now().toSec();
            }else{
                dustbox_ult_set_state.state = -1; //查询超时
                dustbox_ult_check_resend_time = 0;
            }
        } 
    }
    else if(dustbox_ult_set_state.state == 3){
        dustbox_ult_check_resend_time = 0;
        //发送设置命令
        set_lin_ult_mode_type  dustbox_send_set_ult_mode;
        dustbox_send_set_ult_mode.src = MODULE_CONTROL_PC;
        dustbox_send_set_ult_mode.dest = MODULE_DUST_BOX_BOARD;
        dustbox_send_set_ult_mode.mode.data = dustbox_ult_set_state.set_mode; //设置模式
        construct_serial_frame_ex(&user_frame, SET_LIN_ULT_MODE, sizeof(dustbox_send_set_ult_mode), &dustbox_send_set_ult_mode);
        // serial_frame_include_id_type user_frame_include_id_1;
        // user_frame_include_id_1.id = 0;
        // user_frame_include_id_1.cmd = SET_LIN_ULT_MODE;
        // user_frame_include_id_1.need_id = 0;
        // user_frame_include_id_1.frame = user_frame;
        // pushData(&user_frame_include_id_1);
        pushData(construct_frame_include_id(0,SET_LIN_ULT_MODE,0,user_frame));
        dustbox_ult_set_state.set_start_time = ros::Time::now().toSec();
        //更改状态
        dustbox_ult_set_state.state = 4;
    }
    else if(dustbox_ult_set_state.state == 4){
        //等待设置结果
        double time_now = ros::Time::now().toSec();
        if(time_now - dustbox_ult_set_state.set_start_time > 3){
            dustbox_ult_set_resend_time++;
            if(dustbox_ult_set_resend_time <= 5){
                //发送设置命令
                set_lin_ult_mode_type  dustbox_send_set_ult_mode;
                dustbox_send_set_ult_mode.src = MODULE_CONTROL_PC;
                dustbox_send_set_ult_mode.dest = MODULE_DUST_BOX_BOARD;
                dustbox_send_set_ult_mode.mode.data = dustbox_ult_set_state.set_mode; //设置模式
                construct_serial_frame_ex(&user_frame, SET_LIN_ULT_MODE, sizeof(dustbox_send_set_ult_mode), &dustbox_send_set_ult_mode);
                // serial_frame_include_id_type user_frame_include_id_1;
                // user_frame_include_id_1.id = 0;
                // user_frame_include_id_1.cmd = SET_LIN_ULT_MODE;
                // user_frame_include_id_1.need_id = 0;
                // user_frame_include_id_1.frame = user_frame;
                // pushData(&user_frame_include_id_1);
                pushData(construct_frame_include_id(0,SET_LIN_ULT_MODE,0,user_frame));
                dustbox_ult_set_state.set_start_time = ros::Time::now().toSec();
            }else{
                dustbox_ult_set_state.state = -2; //设置超时
                dustbox_ult_set_resend_time = 0;
            }
        } 
    }
#if DEBUG_PRINT
    printf("--------------------vehicle ult_state--------------------\n");
    printf("vehicle_ult_set_state.state: %d\n",vehicle_ult_set_state.state);
    printf("vehicle_ult_set_state.set_mode: %08x\n",vehicle_ult_set_state.set_mode);
    printf("vehicle_ult_set_state.set_start_time: %lf\n",vehicle_ult_set_state.set_start_time);
    printf("vehicle_ult_set_state.recv_mode: %08x\n",vehicle_ult_set_state.recv_mode);
    printf("vehicle_ult_set_state.check_start_time: %f\n",vehicle_ult_set_state.check_start_time);
    printf("--------------------dustbox ult_state--------------------\n");
    printf("dustbox_ult_set_state.state: %d\n",dustbox_ult_set_state.state);
    printf("dustbox_ult_set_state.set_mode: %08x\n",dustbox_ult_set_state.set_mode);
    printf("dustbox_ult_set_state.set_start_time: %lf\n",dustbox_ult_set_state.set_start_time);
    printf("dustbox_ult_set_state.recv_mode: %08x\n",dustbox_ult_set_state.recv_mode);
    printf("dustbox_ult_set_state.check_start_time: %f\n",dustbox_ult_set_state.check_start_time);
#endif
    Info("ULT_ST: "
        <<" V_ULT_ST_ST: "<<(int)vehicle_ult_set_state.state
        <<" V_ULT_ST_SM: "<<vehicle_ult_set_state.set_mode
        <<" V_ULT_ST_SMT: "<<vehicle_ult_set_state.set_start_time
        <<" V_ULT_ST_RM: "<<vehicle_ult_set_state.recv_mode
        <<" V_ULT_ST_RMT: "<<vehicle_ult_set_state.check_start_time
        <<" D_ULT_ST_ST: "<<(int)dustbox_ult_set_state.state
        <<" D_ULT_ST_SM: "<<dustbox_ult_set_state.set_mode
        <<" D_ULT_ST_SMT: "<<dustbox_ult_set_state.set_start_time
        <<" D_ULT_ST_RM: "<<dustbox_ult_set_state.recv_mode
        <<" D_ULT_ST_RMT: "<<dustbox_ult_set_state.check_start_time
        )
}
//---发布嵌入式版本查询的状态
void pub_check_fw_state(){
    cti_fpga_serial::fwVerCheckState fw_check_state_msg; 
    fw_check_state_msg.state = fw_ver_check_state.state;
    fw_check_state_msg.state_msg = fw_ver_check_state.state_msg;
    fw_check_state_msg.major_id_send = fw_ver_check_state.major_id_send;
    fw_check_state_msg.major_id_send_time = fw_ver_check_state.major_id_send_time;
    fw_check_state_msg.major_id_recv = fw_ver_check_state.major_id_recv;
    fw_check_state_msg.minor_id_send = fw_ver_check_state.minor_id_send;
    fw_check_state_msg.minor_id_send_time = fw_ver_check_state.minor_id_send_time;
    fw_check_state_msg.minor_id_recv = fw_ver_check_state.minor_id_recv;
    fw_check_state_msg.wait_respon_timeout_secs = fw_ver_check_state.wait_respon_timeout_secs;
    fw_check_state_msg.recv_210_src = fw_ver_check_state.recv_210_src;
    fw_check_state_msg.recv_210_dest = fw_ver_check_state.recv_210_dest;
    fw_check_state_msg.recv_210_run_area = fw_ver_check_state.recv_210_run_area;
    fw_check_state_msg.recv_210_update_status = fw_ver_check_state.recv_210_update_status;
    fw_check_state_msg.recv_210_boot_ver = fw_ver_check_state.recv_210_boot_ver;
    fw_check_state_msg.recv_210_app_ver = fw_ver_check_state.recv_210_app_ver;
    fw_check_state_msg.recv_210_update_lib_ver = fw_ver_check_state.recv_210_update_lib_ver;
    fw_check_state_pub.publish(fw_check_state_msg);
}
//嵌入式版本查询定时器
void timer_fw_version_check_Callback(const ros::TimerEvent& event){
    //发布状态消息
    pub_check_fw_state();
    Info("FW_CH_ST: "<<fw_ver_check_state.state
    <<"FW_CH_MA_S: "<<fw_ver_check_state.major_id_send
    <<"FW_CH_MA_R: "<<fw_ver_check_state.major_id_recv
    <<"FW_CH_MI_S: "<<fw_ver_check_state.minor_id_send
    <<"FW_CH_MI_R: "<<fw_ver_check_state.minor_id_recv
    );
    if(stm32_update_flag){
        return;
    }
    switch(fw_ver_check_state.state){
        case CH_FREE:
            break;
        case CH_START_CHECK://发送212
            upd_cmd_212_type upd_cmd_212;
            upd_cmd_212.update_node = fw_ver_check_state.major_id_send;
            upd_cmd_212.upd_info.src = MODULE_CONTROL_PC;
            upd_cmd_212.upd_info.dest = MODULE_MOVE_CONTROL_BOARD;
            construct_serial_frame_ex(&user_frame, UPD_CMD_212, sizeof(upd_cmd_212_type), &upd_cmd_212);
            pushData(construct_frame_include_id(0,UPD_CMD_212,0,user_frame));
            fw_ver_check_state.state = CH_WAIT_212_RESPOND;
            fw_ver_check_state.state_msg = fw_ver_check_msg_map.at(CH_WAIT_212_RESPOND);
            fw_ver_check_state.major_id_send_time = ros::Time::now().toSec();
            break;
        case CH_WAIT_212_RESPOND://查询212回复是否超时
            if(ros::Time::now().toSec() - fw_ver_check_state.major_id_send_time  > fw_ver_check_state.wait_respon_timeout_secs){
                fw_ver_check_state.state = CH_WAIT_212_TIMEOUT;
                fw_ver_check_state.state_msg = fw_ver_check_msg_map.at(CH_WAIT_212_TIMEOUT);
            }
            break;
        case CH_212_RESPOND_SUCCESS: //212回复成功，发送209  
            send_to_check_version_type check_version_cmd;
            check_version_cmd.upd_info.src = MODULE_CONTROL_PC;
            check_version_cmd.upd_info.dest = fw_ver_check_state.minor_id_send;
            check_version_cmd.check = 01;
            construct_serial_frame_ex(&user_frame, SEND_TO_CHECK_PROGRAM_VERSION, sizeof(check_version_cmd), &check_version_cmd);
            pushData(construct_frame_include_id(0,SEND_TO_CHECK_PROGRAM_VERSION,0,user_frame));
            fw_ver_check_state.state = CH_WAIT_210_RESPOND;
            fw_ver_check_state.state_msg = fw_ver_check_msg_map.at(CH_WAIT_210_RESPOND);
            fw_ver_check_state.minor_id_send_time = ros::Time::now().toSec();
            break;
        case CH_WAIT_210_RESPOND:  //等待210回复，并查询是否超时
            if(ros::Time::now().toSec() - fw_ver_check_state.minor_id_send_time  > fw_ver_check_state.wait_respon_timeout_secs){
                fw_ver_check_state.state = CH_WAIT_210_TIMEOUT;
                fw_ver_check_state.state_msg = fw_ver_check_msg_map.at(CH_WAIT_210_TIMEOUT);
            }
            break;
        case CH_210_RESPOND_SUCCESS: //210回复成功
            // fw_check_state_msg.state = CH_FREE;
            // fw_check_state_msg.state_msg = fw_ver_check_msg_map.at(CH_FREE);
            break;
        case CH_WAIT_212_TIMEOUT: //212回复超时
            break;
        case CH_RECV_212_IN_WRONG_STATE: //212在错误的状态下回复的
            break;
        case CH_212_RESPOND_ERROR://212回复错误
            break;
        case CH_WAIT_210_TIMEOUT://210回复超时
            break;
        case CH_RECV_210_IN_WRONG_STATE://210在错误的状态下回复的
            break;
        case CH_210_RESPOND_ERROR://210回复错误
            break;
        default:
            break;
    }
    pub_check_fw_state();
}
//************************************** 主函数
int main(int argc, char **argv)
{
//----------------------初始化
    ros::init(argc, argv, "cti_fpga_serial_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    init_signal();
//--------------------testchat_state chassis_chat_state;
  chat_state box_chat_state;
  node_status_publisher_ptr_ = std::make_shared<cti_monitor::NodeStatusPublisher>(nh,pnh);
  node_status_publisher_ptr_->ENABLE();
  node_status_publisher_ptr_->NODE_ACTIVATE();
//----------------------参数变量定义
    std::string filelog;
    std::string cmd_answer_topic;
    std::string version_topic;
    double timer2_duration;
    int pub_hz;
    bool broadcast_tf;
    int send_to_dustbox_fanspeed;
    int send_to_damboard_control;

//----------------------获取参数
    pnh.param<int>("pub_hz", pub_hz, 50);
    pnh.param<bool>("broadcast_tf", broadcast_tf, false);
    pnh.param<double>("timer2_duration", timer2_duration, 1);
    pnh.param("version_topic", version_topic, std::string("/cti/fpga_serial/operationControlVersion"));
    pnh.param<std::string>("cmd_answer_topic", cmd_answer_topic, "/cti/fpga_serial/cmd_answer_cnt");
    pnh.param<std::string>("port_name", serial_port, std::string("/dev/cti_fpga"));
    pnh.param<double>("car_wheel_base", car_wheel_base, 0.8);
    pnh.param<double>("cmd_answer_timeout", cmd_answer_timeout, 0.01);
    pnh.param<int>("max_control_board_version_head", max_control_board_version_head_, 0);
    pnh.param<int>("min_control_board_version_head", min_control_board_version_head_, 0);
    pnh.param<int>("max_control_board_version_mid", max_control_board_version_mid_, 0);
    pnh.param<int>("min_control_board_version_mid", min_control_board_version_mid_, 0);
    pnh.param<int>("max_control_board_version_end", max_control_board_version_end_, 0);
    pnh.param<int>("min_control_board_version_end", min_control_board_version_end_, 0);
    pnh.param<bool>("lift_test_switch", lift_test_switch_, false);
    pnh.param<int>("default_sweeper_id", default_sweeper_id, 0);
    pnh.param<int>("default_dustbox_fanspeed", send_to_dustbox_fanspeed, 80);
    send_to_dust_box_cmd.fan_speed = send_to_dustbox_fanspeed;
    pnh.param<int>("default_damboard_control", send_to_damboard_control, 80);
    send_to_clean_cmd.dam_board = send_to_damboard_control;
    pnh.param<int>("robot_type", robot_type, 0);
    pnh.param<std::string>("/CTI_RUN_VER",cti_run_ver, "");
    pnh.param<int>("default_side_brush_transform", default_side_brush_transform, 0);
    pnh.param<int>("default_side_brush_speed", default_side_brush_speed, 0);
    pnh.param<int>("LIN_ult_installed", LIN_ult_installed, 0);
    printf("LIN_ult_installed: %d\n",LIN_ult_installed);
    pnh.param<std::string>("config_file_path", config_file_path, "/opt/cti/kinetic/share/cti_fpga_serial");
    config_file_path = config_file_path + "/config.yaml";
    printf("config_file_path: %s\n",config_file_path.c_str());
    pnh.param<double>("water_out_per_sec", vehicle_water_status.out_per_sec, 0.0139);
    pnh.param<double>("water_in_per_sec", vehicle_water_status.in_per_sec, 0.1);
    pnh.param<bool>("localization_limit", localization_limit, true);
    printf("localization_limit: %s\n",localization_limit?"true":"false");
    pnh.param<bool>("obstatle_disobs_limit", obstatle_disobs_limit, true);
    printf("obstatle_disobs_limit: %s\n",obstatle_disobs_limit?"true":"false");
    pnh.param<double>("chassis_chat_timeout_secs", chassis_chat_timeout_secs, 10);
    std::cout<<"chassis_chat_timeout_secs: "<<(double)chassis_chat_timeout_secs<<std::endl;
    pnh.param<double>("box_chat_timeout_secs", box_chat_timeout_secs, 10);
    std::cout<<"box_chat_timeout_secs: "<<(double)box_chat_timeout_secs<<std::endl;
    pnh.param<double>("msg_max_range", msg_max_range, 10);
    std::cout<<"msg_max_range: "<<(double)msg_max_range<<std::endl;
    pnh.param<double>("clean_function_test_timeout_secs", clean_function_test_state.timeout_secs, 30);
    std::cout<<"clean_function_test_timeout_secs: "<<(double)(clean_function_test_state.timeout_secs)<<std::endl;
    pnh.param<double>("lora_set_timeout", dustbin_set_state.set_timeout, 15.f);
    std::cout<<"lora_set_timeout: "<<(double)(dustbin_set_state.set_timeout)<<std::endl;

//----------------------ros话题订阅定义
    ros::Subscriber sub_cmd_vel = nh.subscribe("/cmd_vel", 10, cmd_vel_callback);
    ros::Subscriber sub_contraposition = nh.subscribe("/box_pose", 10, boxContraposition_Callback);
    ros::Subscriber box_sub = nh.subscribe("cti/fpga_serial/boxcmd",10,boxUpDown_Callback);
    ros::Subscriber boxlock_sub = nh.subscribe("cti/fpga_serial/boxlockcmd",10,boxLock_Callback);
    ros::Subscriber poweroff_sub = nh.subscribe("cti/fpga_serial/poweroffcmd",10,powerOff_Callback);
    ros::Subscriber stm32_sub = nh.subscribe("cti/fpga_serial/stmupdate",10,stmUpdate_Callback);
    ros::Subscriber light_sub = nh.subscribe("cti/fpga_serial/light_type",2,lightType_Callback);
    ros::Subscriber pose_sub = nh.subscribe("/ndt_pose",10,position_Callback);
    ros::Subscriber disObs_sub = nh.subscribe("/cti/obstacle/disObs",10,disObs_Callback);
    ros::Subscriber checkversion_sub = nh.subscribe("/cti/fpga_serial/checkversion",1,checkversion_Callback);
    ros::Subscriber formatsdcard_sub = nh.subscribe("/cti/fpga_serial/formatsdcard",1,formatsdcard_Callback);
    ros::Subscriber localizerState_sub = nh.subscribe("/cti_localizer_state",1,localizerState_Callback);
    ros::Subscriber dustbin_control_sub;
    ros::Subscriber dustbin_control_sub_new;
    ros::Subscriber dustbin_control_info_sub;
    if(robot_type == 1){
        dustbin_control_sub = nh.subscribe("/cti/chassis_serial/dustbin_control",1,cleanControl_Callback);
        dustbin_control_sub_new = nh.subscribe("/cti/chassis_serial/dustbin_control_new",1,cleanControlNew_Callback);
        dustbin_control_info_sub = nh.subscribe("/cti/chassis_serial/sanitation_vehicle_control",1,cleanControlInfo_Callback);
    }else if(robot_type == 0){
        dustbin_control_sub = nh.subscribe("/cti/chassis_serial/dustbin_control",1,dustbinControl_Callback);
        dustbin_control_info_sub = nh.subscribe("/cti/chassis_serial/sanitation_vehicle_control",1,cleanControlInfo_Callback);
    }else{
        dustbin_control_sub = nh.subscribe("/cti/chassis_serial/dustbin_control",1,dustbinControl_Callback);
        dustbin_control_info_sub = nh.subscribe("/cti/chassis_serial/sanitation_vehicle_control",1,cleanControlInfo_Callback);
    }
    ros::Subscriber dustbin_control_resend_sub;
    if(robot_type == 1){
        dustbin_control_resend_sub = nh.subscribe("/cti/chassis_serial/dustbin_control_include_resend",1,cleanControlResend_Callback);
    }else if(robot_type == 0){
        dustbin_control_resend_sub = nh.subscribe("/cti/chassis_serial/dustbin_control_include_resend",1,dustbinControlResend_Callback);
    }else{
        dustbin_control_resend_sub = nh.subscribe("/cti/chassis_serial/dustbin_control_include_resend",1,dustbinControlResend_Callback);
    }
    if(lift_test_switch_){
    ros::Subscriber lift_test_sub = nh.subscribe("cti/fpga_serial/jacking_test",10,lifttest_Callback);
    }
    ros::Subscriber box_type_sub = nh.subscribe("/cti/rblite/boxtype",1,boxtype_Callback);
    ros::Subscriber sweeper_box_id_sub = nh.subscribe("/cti/chassis_serial/set_sweeper_box_id",1,sweeperBoxID_Callback);
    ros::Subscriber wireless_charge_control_sub = nh.subscribe("/cti/chassis_serial/wireless_charge_control",1,wirelessCharge_Callback);
    ros::Subscriber dust_box_control_sub = nh.subscribe("/cti/chassis_serial/dust_box_control",1,dustBoxControl_Callback);
    ros::Subscriber dust_box_control_sub_new = nh.subscribe("/cti/chassis_serial/dust_box_control_new",1,dustBoxControlNew_Callback);
    ros::Subscriber dust_box_control_info_sub = nh.subscribe("/cti/chassis_serial/sanitation_dustbox_control",1,dustBoxControlInfo_Callback); //环卫车集尘箱控制 cti_msgs::DataArray消息类型
    ros::Subscriber dust_box_autopush_control_sub;
    ros::Subscriber dust_box_fan_speed_sub;
    ros::Subscriber dustbin_damboard_sub;
    if(robot_type == 1){
        dust_box_autopush_control_sub = nh.subscribe("/cti/chassis_serial/dust_box_autopush_control",1,dustBoxAutopushControl_Callback);
        dust_box_fan_speed_sub = nh.subscribe("/cti/chassis_serial/dust_box_fanspeed_control",1,dustBoxFanSpeed_Callback);
        dustbin_damboard_sub = nh.subscribe("/cti/chassis_serial/dustbin_damboard_control",1,dustbinDamboard_Callback);   
    } 
    ros::Subscriber dustbin_sideBrushTrans_sub = nh.subscribe("/cti/chassis_serial/dustbin_sideBrushTrans_control",1,dustbinSideBrushTrans_Callback);//测试用的,单独控制边刷伸展
    ros::Subscriber smart_trash_sub = nh.subscribe("/cti/chassis_serial/smart_trash_control",1,smartTrash_Callback);
    ros::Subscriber dustbox_5g_check_sub = nh.subscribe("/cti/chassis_serial/sanitation_dustbox_5g_check",1,dustbox5GCheck_Callback);
    ros::Subscriber ledshow_sub = nh.subscribe("/cti/ledshow/cmd",1,ledshow_Callback);
    ros::Subscriber ult_cmd_sub_ = nh.subscribe("/cti/ultrasonic/ult_cmd",2,ultCmdCallback);
    ros::Subscriber clean_function_test_sub = nh.subscribe("/cti/chassis_serial/clean_function_test",1,cleanFunctionTest_Callback);//环卫车清扫功能测试
    
//----------------------ros话题发布定义
    //odom_pub = nh.advertise<nav_msgs::Odometry>("/robot_odom", 10);
    odom_pub_4wd = nh.advertise<nav_msgs::Odometry>("/robot_odom_4wd", 10);
    odom_pub_calc = nh.advertise<nav_msgs::Odometry>("/robot_odom_calc", 10);
    sins_pub = nh.advertise<cti_msgs::Sins>("/cti/fpga_serial/sins",10);
    imudata_pub = nh.advertise<sensor_msgs::Imu>("/cti/fpga_serial/imu",10);       
    //steer_pub = nh.advertise<std_msgs::Float64>("/robot_steer_angle", 10);//2020年4月2日停用
    battery_pub = nh.advertise<cti_msgs::BatteryState>("/cti/cti_fpga/battery_state", 10);
    batcell_pub = nh.advertise<cti_msgs::BatteryCellsState>("/cti/cti_fpga/batcell_state", 10);
    //info_pub = nh.advertise<std_msgs::Float64MultiArray>("/cti/fpga_serial/run_info", 10); //2020年4月2日停用
    ctlinfo_pub = nh.advertise<cti_msgs::VehicleCtlRunInfo>("/cti/fpga_serial/ctlrun_info", 10);
    state_pub = nh.advertise<std_msgs::String>("/cti/fpga_serial/error", 10, true);
    stm32_pub = nh.advertise<cti_fpga_serial::updateinfo>("/cti/fpga_serial/stminfo", 10);
    cmd_answer_pub = nh.advertise<cti_fpga_serial::cmdanswer>(cmd_answer_topic, 10);
    boxrfid_pub_all = nh.advertise<cti_msgs::BoxState>("/cti/fpga_serial/rfid_all",10);
    boxrfid_pub_single = nh.advertise<cti_msgs::BoxState>("/cti/fpga_serial/rfid_single",10);
    ranges_pub = nh.advertise<cti_msgs::Range>("/cti/fpga_serial/multiple_ultrasonic", 1);
    for(int i = 0; i < max_type_ult; i++){
        range_pub[i] = nh.advertise<sensor_msgs::Range>("/cti/fpga_serial/"+ult_name[i], 1);
    }
    for(int i = 0; i < sweeper_max_type_ult; i++){
        sweeper_range_pub[i] = nh.advertise<sensor_msgs::Range>("/cti/chassis_serial/"+sweeper_ult_name[i], 1);
    }
    for(int i = 0; i < SWEEPER_ULT_MODULE_NUM; i++){
        for(int j = 0; j < new_sweeper_max_type_ult; j++){
            new_sweeper_range_pub[i][j] = nh.advertise<sensor_msgs::Range>("/cti/chassis_serial/"+new_sweeper_ult_name[i][j],1);
        }
    } 

    //集尘箱超声波发布
    for(int i = 0; i < dustbox_max_type_ult; i++){
        dustbox_range_pub[i] = nh.advertise<sensor_msgs::Range>("/cti/chassis_serial/"+dustbox_ult_name[i], 1);
    }
    raw_lin_range_pub = nh.advertise<cti_fpga_serial::rangeRawData>("/cti/chassis_serial/range_raw_data",25);
    firmvion_pub = nh.advertise<cti_msgs::RobotVersionDisplay>("/cti/fpga_serial/operationControlVersion", 1,true);
    serial_status_pub = nh.advertise<cti_fpga_serial::serialstatus>("/cti/fpga_serial/serial_status", 1,true);
    box_laser_pub = nh.advertise<cti_msgs::Rtcm>("/cti/fpga_serial/box_laser",10);
    chassis_error_pub = nh.advertise<std_msgs::UInt32MultiArray>("/cti/chassis_serial/chassis_error",10);
    navigation_log_pub = nh.advertise<cti_fpga_serial::navigationlog>("/cti/fpga_serial/navigation_log",10); 
    baro_status_pub = nh.advertise<std_msgs::UInt16>("/cti/chassis_serial/baro_status",10);
    formatsdcard_pub = nh.advertise<std_msgs::UInt8>("/cti/fpga_serial/formatsdcard_result",1,true);
    gps_pub = nh.advertise<cti_msgs::GnssRTK>("/cti/chassis_serial/gnss",10);
    compass_pub = nh.advertise<std_msgs::Int32MultiArray>("/cti/chassis_serial/compass",10);
    node_status_pub = nh.advertise<std_msgs::UInt8>("/cti/chassis_serial/node_active",25);
    dustbin_state_pub = nh.advertise<cti_msgs::DustbinState>("/cti/chassis_serial/dustbin_state",25);
    //新的环卫车清扫状态发布,将以前拆开发布的数据放在这里发布
    dustbin_state_pub_new = nh.advertise<cti_msgs::DustbinStateNew>("/cti/chassis_serial/dustbin_state_new",25); 
    firmware_version_status_pub = nh.advertise<std_msgs::Int8>("/cti/chassis_serial/control_board_version_status",1,true);
    firmware_version_check_pub = nh.advertise<cti_fpga_serial::firmwareinfo>("/cti/chassis_serial/FW_check_report", 1,true);
    recv_chassis_info_pub = nh.advertise<cti_fpga_serial::vehiclestate>("/cti/chassis_serial/chassis_info",10);
    set_dustbin_id_state_pub = nh.advertise<cti_msgs::TabState>("/cti/chassis_serial/dustbin_set_id_state",1,true);
    recv_dustbin_id_state_pub = nh.advertise<cti_fpga_serial::dustbinidstate>("/cti/chassis_serial/dustbin_id_state",1,true);
    dust_box_state_pub = nh.advertise<cti_msgs::DustbinState>("/cti/chassis_serial/dust_box_state",25);
    dust_box_state_pub_new = nh.advertise<cti_msgs::DustboxState>("/cti/chassis_serial/dust_box_state_new",25);
    dust_box_state_info_pub = nh.advertise<cti_msgs::DataArray>("/cti/chassis_serial/sanitation_dustbox_state",25);//cti_msgs/DataArray类型//环卫车,吸尘箱状态发布
    dust_box_autopush_pub = nh.advertise<std_msgs::UInt8>("/cti/chassis_serial/dust_box_autopush_state",25);
    dust_box_fanspeed_pub = nh.advertise<std_msgs::UInt8>("/cti/chassis_serial/dust_box_fanspeed_state",25);
    dustbin_damboard_pub = nh.advertise<std_msgs::UInt8>("/cti/chassis_serial/dustbin_damboard_state",25);
    dustbox_wireless_charge_state_pub = nh.advertise<cti_msgs::BatteryState>("/cti/chassis_serial/dustbox_wireless_charge_state",2);
    dustbox_bottom_range_pub = nh.advertise<sensor_msgs::Range>("/cti/chassis_serial/dustbox_bottom",3);
    boxlock_state_pub = nh.advertise<std_msgs::UInt8>("/cti/chassis_serial/boxlock_state",25);
    dust_box_state_pub_json = nh.advertise<std_msgs::String>("/cti/chassis_serial/dust_box_state_json",25);
    rain_sensor_pub = nh.advertise<cti_msgs::BoxState>("/cti/chassis_serial/rain_sensor",25);
    smart_trash_state_pub = nh.advertise<cti_msgs::DataArray>("/cti/chassis_serial/smart_trash_state",25);
    dust_vehicle_state_info_pub = nh.advertise<cti_msgs::DataArray>("/cti/chassis_serial/sanitation_vehicle_state",25);//cti_msgs/DataArray类型//环卫车,车辆清扫状态发布
    dustbox_5g_state_pub = nh.advertise<cti_msgs::DataArray>("/cti/chassis_serial/sanitation_dustbox_5g_state",25);//吸尘箱5g状态发布
    dustbox_batterycell_pub = nh.advertise<cti_msgs::BatteryCellsState>("/cti/chassis_serial/dustbox_batcell_state", 10);//吸尘箱总电量发布
    chat_statue_pub = nh.advertise<cti_msgs::BoxState>("/cti/chassis_serial/communication_state",1);//通信状态发布
    lin_ult_data_pub = nh.advertise<std_msgs::UInt16MultiArray>("/cti/chassis_serial/lin_ult_data",1);//lin通信超声波源数据发布
    dustbox_rear_range_pub = nh.advertise<std_msgs::UInt16MultiArray>("/cti/chassis_serial/sanitation_dustbox_ult",2);//吸尘箱超声波原始数据发送
    clean_function_test_state_pub = nh.advertise<cti_fpga_serial::cleanFuncTestState>("/cti/chassis_serial/clean_function_test_state",10);//清扫功能测试状态发布
    clean_function_test_report_pub = nh.advertise<cti_msgs::DataArray>("/cti/chassis_serial/clean_function_test_report",10);//清扫功能测试结果发布
    lora_id_set_state_pub = nh.advertise<cti_fpga_serial::LoraIdSetState>("/cti/chassis_serial/lora_id_set_state",10);//lora id 设置 状态发布
    wheel_speed_pub = nh.advertise<std_msgs::Float32MultiArray>("/cti/chassis_serial/wheels_speed",10);//四个车轮速度发布
    error_publisher = nh.advertise<cti_msgs::ErrorStatus>("/cti/error_status/set", 1, true);
    vehicle_ult_set_state_pub = nh.advertise<cti_fpga_serial::ultModeSetState>("/cti/chassis_serial/vehicle_ult_set_state",1,true);
    dustbox_ult_set_state_pub = nh.advertise<cti_fpga_serial::ultModeSetState>("/cti/chassis_serial/dustbox_ult_set_state",1,true);
    fw_check_state_pub = nh.advertise<cti_fpga_serial::fwVerCheckState>("/cti/chassis_serial/fw_check_state",1,true);
//----------------------ros定时器定义
    ros::Timer timer = nh.createTimer(ros::Duration(1), &timerCallback);
    ros::Timer timer2 = nh.createTimer(ros::Duration(timer2_duration), &timer2Callback);
    ros::Timer timer3 = nh.createTimer(ros::Duration(cmd_answer_timeout),&timer3Callback);
    ros::Timer timer4 = nh.createTimer(ros::Duration(RECVTIMEOUT_DUR),&timer4Callback);
    //关闭timer5以后，接到底盘发布的错误消息会发布出去，打开timer5,平时会发底盘正常的消息
    //ros::Timer timer5 = nh.createTimer(ros::Duration(0.25),&timer5Callback);
    ros::Timer timer6 = nh.createTimer(ros::Duration(0.04),&timer6Callback);
    ros::Timer timer7 = nh.createTimer(ros::Duration(1),&timer7Callback);//清扫箱箱号设置和获取
    ros::Timer timer_chat_timeout = nh.createTimer(ros::Duration(1),&timer_chat_timeout_Callbak);//清扫箱箱号设置和获取
    ros::Timer timer_set_ult_mode = nh.createTimer(ros::Duration(0.5),&timer_set_ult_mode_Callback);//设置超声波模式
    ros::Timer timer_clean_function_test_timeout = nh.createTimer(ros::Duration(0.5),&timer_clean_function_test_timeout_Callback);//设置超声波模式
    ros::Timer timer_check_fw_version=nh.createTimer(ros::Duration(0.2),&timer_fw_version_check_Callback);//嵌入式版本查询定时器
//----------------------log文件名称
    if(nh.getParam("CTI_RUN_LOG_PATH", filelog))
    {
        filelog += "/control.log";
    }
    else
    {
        pnh.param<std::string>("filenamelog", filelog, "/opt/control.log");
    }
    std::cout << "filenamelog:" << filelog << std::endl;


    //车身lin超声波默认模式赋值 车身前右，后右，左前下，右前下，其他设置为又收又发，车身有效位设置对应值,无效位全部为1
    vehicle_defalt_linult_mode.bits.lin_front_left = 2;
    vehicle_defalt_linult_mode.bits.lin_front_right = 1;
    vehicle_defalt_linult_mode.bits.lin_back_center = 2;
    vehicle_defalt_linult_mode.bits.lin_rear_left = 2;
    vehicle_defalt_linult_mode.bits.lin_rear_right = 1;
    vehicle_defalt_linult_mode.bits.unused1 = 0xffff;
    vehicle_defalt_linult_mode.bits.lin_left_front_up = 2;
    vehicle_defalt_linult_mode.bits.lin_left_front_down = 1;
    vehicle_defalt_linult_mode.bits.lin_left_center = 2;
    vehicle_defalt_linult_mode.bits.lin_right_center = 2;
    vehicle_defalt_linult_mode.bits.lin_right_front_down = 1;
    vehicle_defalt_linult_mode.bits.lin_right_front_up = 2;
    vehicle_defalt_linult_mode.bits.unused2 = 0xffff;
    vehicle_defalt_linult_mode.bits.lin_dustbox_rear_right = 0xff;
    vehicle_defalt_linult_mode.bits.lin_dustbox_rear_left = 0xff;
    vehicle_defalt_linult_mode.bits.lin_dustbox_bottom = 0xff;
    vehicle_defalt_linult_mode.bits.unused3 = 0xffff;
    vehicle_defalt_linult_mode.bits.unused4 = 0xffff;
    //车身lin超声波 全部关闭模式赋值，车身有效位全部为0,无效位全部为1
    vehicle_all_stop_work_mode.data = 0;
    vehicle_all_stop_work_mode.bits.unused1 = 0xffff;
    vehicle_all_stop_work_mode.bits.unused2 = 0xffff;
    vehicle_all_stop_work_mode.bits.unused3 = 0xffff;
    vehicle_all_stop_work_mode.bits.unused4 = 0xffff;
    vehicle_all_stop_work_mode.bits.lin_dustbox_rear_right = 0xff;
    vehicle_all_stop_work_mode.bits.lin_dustbox_rear_left = 0xff;
    vehicle_all_stop_work_mode.bits.lin_dustbox_bottom = 0xff;
    
    

    //吸尘箱lin超声波默认模式赋值 箱子后左 设置为只收不发， 其他设置为又收又发，箱子有效位设置为对应值，其余全为1
    dustbox_defalt_linult_mode.data = 0xffffffffffffffff;
    dustbox_defalt_linult_mode.bits.lin_dustbox_rear_right = 2;
    dustbox_defalt_linult_mode.bits.lin_dustbox_rear_left = 1;
    dustbox_defalt_linult_mode.bits.lin_dustbox_bottom = 2;
    //吸尘箱lin超声波 全部关闭模式赋值，吸尘箱有效位全部为0,无效位全部为1
    dustbox_all_stop_work_mode.data = 0xffffffffffffffff;
    dustbox_all_stop_work_mode.bits.lin_dustbox_rear_right = 0x00;
    dustbox_all_stop_work_mode.bits.lin_dustbox_rear_left = 0x00;
    dustbox_all_stop_work_mode.bits.lin_dustbox_bottom = 0x00;


    printf("vehicle_defalt_linult_mode.data: 0x%016lx\n",vehicle_defalt_linult_mode.data);
    printf("vehicle_all_stop_work_mode.data: 0x%016lx\n",vehicle_all_stop_work_mode.data);
    printf("dustbox_defalt_linult_mode.data: 0x%016lx\n",dustbox_defalt_linult_mode.data);
    printf("dustbox_all_stop_work_mode.data: 0x%016lx\n",dustbox_all_stop_work_mode.data);


   

//----------------------设��串口通信sdk
    strcpy((char *)SERIAL_DEVICE_PORT, serial_port.c_str());
    //strcpy((char *)SERIAL_DEVICE_PORT, "/dev/serialLock0");
    //strcpy((char *)SERIAL_DEVICE_PORT, "/dev/ttyUSB0");
    process_nomal_cmd_cb = process_nomal_cmd_ex;
    process_update_cmd_cb = process_update_cmd_ex;
    module_serial_init();
    ros::Rate loop_rate(100);
    //----------------------------
    initLog(filelog); 
    //--
    databuf.clear();
//---------------------创建接收数据的线程
    int p_ret = 0;
    p_ret = pthread_create(&th_serial_recv, NULL, serial_recv_thread, NULL);
    if(p_ret != 0)
    {
        printf("serial thread creat error!\n");
    }
//----------------------给每一个RFID容器赋初值，防止boxload_callback判断错误
    cti_msgs::TabState tabstate;
    oldtabstate_rfid1.push_back(tabstate);
    oldtabstate_rfid2.push_back(tabstate);
    oldtabstate_rfid3.push_back(tabstate);
    oldtabstate_rfid4.push_back(tabstate);
//-----------------------serial_status赋初值
    serial_status.recv_from_dustbin_state_old_cnt = 0;
    serial_status.callback_dustbin_cnt_old_cnt = 0;
    serial_status.recv_from_dustbin_state_new_cnt = 0;
    serial_status.callback_dustbin_cnt_new_cnt = 0;
    serial_status.dustbin_state_old_size = (uint16_t)sizeof(clean_to_motion_t);
    serial_status.dustbin_state_new_size = (uint16_t)sizeof(clean_to_motion_t_new);
    serial_status.dustbin_state_recv_size = 0;
    serial_status.recv_from_dustbox_state_old_cnt = 0;
    serial_status.callback_dustbox_cnt_old_cnt = 0;
    serial_status.recv_from_dustbox_state_new_cnt = 0;
    serial_status.callback_dustbox_cnt_new_cnt = 0;
    serial_status.dustbox_state_old_size = (uint16_t)sizeof(dust_box_to_motion_t_old);
    serial_status.dustbox_state_new_size = (uint16_t)sizeof(dust_box_to_motion_t_new);
    serial_status.dustbox_state_recv_size = 0;
//从文件中读取初始水箱水量,如果没有,则新建,并创建水量为90%
    std::ifstream in ;
    in.open(config_file_path.c_str(),std::ios::in);
    if(!in.is_open()){
        printf("Could not open the config file: %s\n",config_file_path.c_str());
        Info("could not open file: "<< config_file_path);
        std::string touch_cmd = "touch " + config_file_path;
        std::string modify_file = "echo 90.0 > " + config_file_path;
        FILE *fpr = NULL;
        fpr = popen(touch_cmd.c_str(),"w");
        pclose(fpr);
        fpr = popen(modify_file.c_str(),"w");
        pclose(fpr);
        vehicle_water_status.water_percnt_now = 90.0;
    }else{
        std::string water_per_stored;
        getline(in,water_per_stored);
        vehicle_water_status.water_percnt_now = atof(water_per_stored.c_str());
    }


    while (ros::ok())
    {
	    //module_serial_process();
		module_serial_process_thread();
        sendData();
        ros::spinOnce();
        loop_rate.sleep();      
        //printf("main--loop ----\n");
    }
    Error("cti_fpga_serial_node quit!");
    pthread_cancel(th_serial_recv);
    pthread_join(th_serial_recv, NULL);
    return 0;
}
