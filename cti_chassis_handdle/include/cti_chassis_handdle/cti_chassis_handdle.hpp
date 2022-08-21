#ifndef CTI_CHASSIS_ACCEPT_H
#define CTI_CHASSIS_ACCEPT_H
#include "cti_chassis_handdle/cti_chassis_communicate_interface.hpp"
#include "cti_chassis_handdle/cti_chassis_communicate_realization.hpp"
#include "cti_chassis_handdle/user_cmd.hpp"

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
#include <sstream>
#include <iomanip>
#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include <chrono>
#include "time.h"
#include <sys/time.h>
#include "jsoncpp/json/json.h"
#include <cstring>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include "ctilog/log.hpp"
#include "ctilog/loghelper.cpp.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "cti_msgs/msg/battery_state.hpp"
#include "cti_msgs/msg/target_pose.hpp"
#include "cti_msgs/msg/range.hpp"
#include "cti_msgs/msg/battery_cell.hpp"
#include "cti_msgs/msg/battery_cells_state.hpp"
#include "cti_msgs/msg/vehicle_ctl_run_info.hpp"
#include "cti_msgs/msg/robot_version_display.hpp"
#include "cti_msgs/msg/tab_state.hpp"
#include "cti_msgs/msg/box_state.hpp"
#include "cti_msgs/msg/sins.hpp"
#include "cti_msgs/msg/state.hpp"
#include "cti_msgs/msg/rtcm.hpp"
#include "cti_chassis_msgs/msg/update_info.hpp"
#include "cti_chassis_msgs/msg/cmd_answer.hpp"
#include "cti_chassis_msgs/msg/firm_ware_info.hpp"
#include "sensor_msgs/msg/range.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <cti_msgs/msg/auto_transmission.hpp>
#include <cti_chassis_msgs/msg/navigation_log.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <cti_msgs/msg/gnss_rtk.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cti_msgs/msg/robot_localizer_state.hpp>
#include <cti_msgs/msg/dustbin_control.hpp>
#include <cti_msgs/msg/dustbin_state.hpp>
#include <std_msgs/msg/int8.hpp>
#include <cti_chassis_msgs/msg/vehicle_state.hpp>
#include <std_msgs/msg/int64.hpp>
#include "cti_chassis_msgs/msg/dustbinid_state.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <cti_msgs/msg/dustbin_control_new.hpp>
#include <cti_msgs/msg/dustbin_state_new.hpp>
#include <cti_msgs/msg/dustbox_control.hpp>
#include <cti_msgs/msg/dustbox_state.hpp>
#include <cti_msgs/msg/data.hpp>
#include <cti_msgs/msg/data_array.hpp>
#include <cti_chassis_msgs/msg/range_raw_data.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <cti_chassis_msgs/msg/ult_v30_datas.hpp>

#define DEBUG_PRINT 0
#define cmax(x, y) ((x > y) ? (x) : (y))
#define cmin(x, y) ((x < y) ? (x) : (y))
#define PI 3.14159265358979323846
#define GRAV 9.7803
#define SWEEPER_ULT_FILTER_NUM 2

//通讯端口标志
#define PORT_SERIAL 1
#define PORT_UDP 2

class ChassisHanddle : public rclcpp::Node {
 public:
  explicit ChassisHanddle(std::shared_ptr<ChassisCmncBase> chassisCmncBase_)
      : chassisCmncBase(chassisCmncBase_), Node("cti_chassis_handle_node") {
    this->init();
  }

 private:
  void init();
  void initParam();
  void initSub();
  void initPub();
  void initTimer();
  void initLog(const std::string name);
  void initCtiLog();
  void initChassisCommunication();
  void spinNode();

  void getChassisData();
  void sendChassisData();

  int process_nomal_cmd(unsigned char cmd, unsigned int length,
                        unsigned char *data);

  double calcYawFromQuaternion(const tf2::Quaternion &q);

  std::string hex2string(uint8_t *data, unsigned int Len);

  //----------------------回调函数---------------------------------------------//
  void poweroff_cmd_type_callback(
      const std_msgs::msg::UInt32MultiArray::SharedPtr msg);

  void soft_stop_callback(const std_msgs::msg::Int8::SharedPtr msg);

  void cmd_vel_callback(
      const geometry_msgs::msg::TwistStamped::SharedPtr twistStamped);

  void boxUpDown_Callback(const std_msgs::msg::Int32::SharedPtr msg);

  void stmUpdate_Callback(
      const cti_chassis_msgs::msg::UpdateInfo::SharedPtr msg);

  void lightType_v3_0_Callback(const cti_msgs::msg::DataArray::SharedPtr msg);

  void position_Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void checkversion_Callback(const std_msgs::msg::UInt8::SharedPtr msg);

  void formatsdcard_Callback(const std_msgs::msg::UInt8::SharedPtr msg);

  void cleanControlNew_Callback(
      const cti_msgs::msg::DustbinControlNew::SharedPtr msg);

  void cleanControlInfo_Callback(const cti_msgs::msg::DataArray::SharedPtr msg);

  void dustBoxControlInfo_Callback(
      const cti_msgs::msg::DataArray::SharedPtr msg);

  void sprayControl_Callback(const std_msgs::msg::UInt8::SharedPtr msg);

  void wirelessCharge_Callback(const std_msgs::msg::UInt8::SharedPtr msg);

  void exit_charging_Callback(const std_msgs::msg::UInt8::SharedPtr msg);

  void dustBoxAutopushControl_Callback(
      const std_msgs::msg::UInt8::SharedPtr msg);

  void localizerState_Callback(
      const cti_msgs::msg::RobotLocalizerState::SharedPtr msg);

  //-------------------------定时器-----------------------------------//
  void timer5Callback();

  void timerCallback();

  void timer2Callback();

  void mainTimerCallback();

  void timer_set_process_work_mode_Callback();

  double get_duration(double in_time);

  void timer_chat_timeout_Callbak();

  int process_update_cmd_ex(unsigned char cmd, unsigned char *data,
                            unsigned int length);

  void pub_rain_sensor_new(rain_sensor_t_new *data);

  void pub_dust_box_state_new(dust_box_to_motion_t_new *data);

  void pub_clean_state_new(clean_to_motion_t_new *data);

  void check_rough_res(time_sync_rough_msg_type *data);

  void send_fine_sync_cmd();

  void send_delay_sync_cmd();

  void pub_gps(recv_gps_data_type *data);

  void pub_rtk(msg_rtk_gps_data_1_0 *data);

  void pub_formatsdcard(send_format_sd_card_cmd_type *data);

  void pub_navigationlog(recv_navigation_log_status_type *data);

  void pub_odom(const recv_from_control_status_type *data);

  void pub_alt_3_0_(const msg_upa_pos_data_t *data);

  void pub_battery_3_0(
      const recv_battery_4_to_1_active_report_status_type_3_0_ *data);

  bool less_equa_compare(std::vector<int> vec1, std::vector<int> vec2);

  void getNumInString(std::string str);

  double gps_data_trans(double data);

  void pub_firmwareversion(recv_from_firmware_version_type *data);

  void compar_id_recv_send(recv_from_cmd_answer_type *data);

  void pub_chassiserror(recv_chassis_error_report_type *data);

  //----------------------ros话题订阅定义---------------------------------------//
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_soft_stop;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_vel;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr box_sub;

  rclcpp::Subscription<cti_chassis_msgs::msg::UpdateInfo>::SharedPtr stm32_sub;

  rclcpp::Subscription<cti_msgs::msg::DataArray>::SharedPtr light_v3_0_sub;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr checkversion_sub;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr formatsdcard_sub;

  rclcpp::Subscription<cti_msgs::msg::RobotLocalizerState>::SharedPtr
      localizerState_sub;

  rclcpp::Subscription<cti_msgs::msg::DustbinControlNew>::SharedPtr
      dustbin_control_sub_new;
  rclcpp::Subscription<cti_msgs::msg::DataArray>::SharedPtr
      dustbin_control_info_sub;

  rclcpp::Subscription<cti_msgs::msg::DataArray>::SharedPtr
      dust_box_control_info_sub;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
      wireless_charge_control_sub;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
      dust_box_autopush_control_sub;

  rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr
      power_control_sub;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr exit_charging_sub;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sprayControl_sub;

  ////////////////////////////////////////////////////////////////////////////

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_4wd;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_calc;
  rclcpp::Publisher<cti_chassis_msgs::msg::UpdateInfo>::SharedPtr stm32_pub;
  rclcpp::Publisher<cti_msgs::msg::BatteryCellsState>::SharedPtr batcell_pub;
  rclcpp::Publisher<cti_msgs::msg::VehicleCtlRunInfo>::SharedPtr ctlinfo_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
  rclcpp::Publisher<cti_msgs::msg::RobotVersionDisplay>::SharedPtr firmvion_pub;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr
      alt_3_0_pub[ULT_3_0_TYPE_NUM];
  rclcpp::Publisher<cti_chassis_msgs::msg::RangeRawData>::SharedPtr
      raw_lin_range_pub;  // lin通信超声波原始数据发布
  rclcpp::Publisher<cti_chassis_msgs::msg::CmdAnswer>::SharedPtr cmd_answer_pub;
  rclcpp::Publisher<cti_msgs::msg::Sins>::SharedPtr sins_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imudata_pub;
  rclcpp::Publisher<cti_msgs::msg::Rtcm>::SharedPtr
      box_laser_pub;  //激光对箱信息发布
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr
      chassis_error_pub;  //底盘错误信息发布
  rclcpp::Publisher<cti_chassis_msgs::msg::NavigationLog>::SharedPtr
      navigation_log_pub;  //底盘重要信息上传
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr
      baro_status_pub;  //气压计状态发布
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr
      formatsdcard_pub;  // SD卡格式化结果回复
  rclcpp::Publisher<cti_msgs::msg::GnssRTK>::SharedPtr gps_pub;  // gps信息发布
  rclcpp::Publisher<cti_msgs::msg::GnssRTK>::SharedPtr rtk_pub;  // rtk信息发布
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr
      compass_pub;  // compass信息发布
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr
      firmware_version_status_pub;  //运控版本兼容状态发布
  rclcpp::Publisher<cti_chassis_msgs::msg::FirmWareInfo>::SharedPtr
      firmware_version_check_pub;  //查询地盘固件版本信息的返回详细信息
  rclcpp::Publisher<cti_chassis_msgs::msg::VehicleState>::SharedPtr
      recv_chassis_info_pub;  //收到运控上传所有信息发布
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr
      dust_box_autopush_pub;  //集尘箱自动倒垃圾发布
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr
      boxlock_state_pub;  //顶升磁吸锁状态
  rclcpp::Publisher<cti_msgs::msg::BoxState>::SharedPtr
      rain_sensor_pub;  //雨水传感器数据发布

  rclcpp::Publisher<cti_msgs::msg::DataArray>::SharedPtr
      dust_box_state_info_pub;  // 垃圾箱状态发布 cti_msgs/DataArray类型
  rclcpp::Publisher<cti_msgs::msg::DataArray>::SharedPtr
      dust_vehicle_state_info_pub;  // 环卫车清扫状态发布 cti_msgs/DataArray类型

  rclcpp::Publisher<cti_msgs::msg::BoxState>::SharedPtr
      chat_statue_pub;  //通讯状态发布(箱子和底盘)
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr
      lin_ult_data_pub;  // lin通信超声波原始数据发布
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr
      lin_ult_data_v3_0_pub;  // lin通信超声波原始数据发布_3.0的车　
  rclcpp::Publisher<cti_chassis_msgs::msg::UltV30Datas>::SharedPtr
      new_lin_ult_data_v3_0_pub;  // 新lin通信超声波原始数据发布_3.0的车　

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shallow_sleep_pub;//浅休眠

  //----------------------ros定时器定义-----------------------------------------//
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::TimerBase::SharedPtr timer2;
  rclcpp::TimerBase::SharedPtr timer_chat_timeout;
  rclcpp::TimerBase::SharedPtr timer_process_work_mode;
  rclcpp::TimerBase::SharedPtr main_timer;

  //--灯控
  uint16_t light_type{0};

  //串口通讯端口
  int port_type{0};
  std::string port_type_name;
  std::string serial_port;
  // udp通信端口
  std::string udp_ip;
  int udp_port{0};
  std::string udp_ip_dest;
  int udp_port_dest{0};

  double car_wheel_base;
  bool stm32_update_flag = false;
  unsigned int TIMEOUT{0};
  uint32_t seq_num{0};
  //--里程计
  double odom_x{0}, odom_x_4wd{0};
  double odom_y{0}, odom_y_4wd{0};
  double odom_th{0}, odom_th_4wd{0};
  nav_msgs::msg::Odometry odom, odom_4wd;
  //--IMU
  cti_msgs::msg::Sins sins_info;
  sensor_msgs::msg::Imu imu_info;
  //--路程
  double mileage{0};
  //--电池电量
  cti_msgs::msg::BatteryState batteryState;
  cti_msgs::msg::BatteryCellsState BatCellsState;
  //--版本信息
  cti_msgs::msg::RobotVersionDisplay RobotVersionDisplay;
  //--超声波数据
  cti_msgs::msg::Range rangeDatas;
  sensor_msgs::msg::Range rangeData;
  //--运行信息
  cti_msgs::msg::VehicleCtlRunInfo runinfo;
  //--命令ID全局计数变量
  uint16_t cmd_id_global{0};
  //--已发送命令的储存map，map的大小限制。
  std::map<uint16_t, double> cmd_send_map;
  //--应答成功/失败的命令计数
  uint32_t cmd_answer_success_cnt{0};
  uint32_t cmd_answer_fail_cnt{0};
  //--cmd send count
  uint32_t cmd_withid_send_cnt{0};
  uint32_t cmd_withid_recv_cnt{0};
  //--消息应答情况
  cti_chassis_msgs::msg::CmdAnswer cmd_answer_cnt;
  //--timer3 计时器回调函数变量
  uint8_t sendtime_loop_num{50};
  //--查询到版本标志位
  bool get_version_flag = false;
  //--顶升、锁箱、命令应答全局变量
  int8_t global_up_down_flag{0};
  int8_t global_switch_flag{0};
  double cmd_answer_timeout{0};
  // position global temp
  float global_pose[3] = {0};
  float global_q[4] = {0};

  //-- 障碍物信息
  int16_t global_k{0};       //前障碍物信息
  int16_t global_k_back{0};  //后障碍物信息
  //--激光对箱消息
  cti_msgs::msg::Rtcm box_laser;
  //--底盘错误码消息
  //--底盘导航log消息
  cti_chassis_msgs::msg::NavigationLog navigation_log;
  //--气压计数据储存全局变量
  float baro_raw_old{0};
  //--气压计原始数据值检测消息
  std_msgs::msg::UInt16 baro_status;
  //--底盘错误码全局变量
  uint8_t module_type_global{0};
  uint32_t module_error_code_global{0};
  uint8_t module_error_level_global{0};
  //--portIndex全局变量
  uint8_t send_ctl_portIndex_cnt{0};
  uint8_t send_contraposition_portIndex_cnt{0};
  uint8_t send_poweroff_portIndex_cnt{0};
  uint8_t send_timenow_portIndex_cnt{0};
  //--sd卡格式化结果消息
  std_msgs::msg::UInt8 formatsdcard_result;
  //--gps消息
  cti_msgs::msg::GnssRTK gps_data;
  //--rtk消息
  cti_msgs::msg::GnssRTK rtk_data;
  //--定位状态全局变量
  int32_t robotlocalizerstate_global{0};
  //--版本号全局变量
  int8_t control_version_right = {0};  //运控版本兼容状态 0：运控版本可用;
                                       // 1：运控版本过高; -1：运控版本过低;
  int max_control_board_version_head_{0};
  int min_control_board_version_head_{0};
  int max_control_board_version_mid_{0};
  int min_control_board_version_mid_{0};
  int max_control_board_version_end_{0};
  int min_control_board_version_end_{0};
  //--ptp时间同步全局变量
  bool need_resend_rough = false;
  time_sync_rough_msg_type sync_rough_msg_saved;
  bool check_rough_sync_res_timeoout;
  uint8_t check_rough_sync_res_cnt;
  uint8_t send_fine_sync_cnt{0};
  bool need_send_rough = true;
  bool need_send_fine_sync = false;
  //--清扫箱控制变量
  bool check_clean_mechine_state;
  uint8_t recv_clean_mechine_motor_status;
  //--运控版本兼容状态消息
  std_msgs::msg::Int8 firmwareVersionCheck;  //
  //--车上的箱子类型状态
  int32_t box_type;
  //--清扫箱箱号设置状态
  dusbin_rf_set_state_t dustbin_set_state;
  //--清扫箱在车上设置标志位
  bool set_dustbin_on_car = false;  // false:未进行设置；true:设置中；
  //--读取清扫箱id标志位
  bool read_dustbin_id = false;
  //--清扫箱id设置接受
  cti_chassis_msgs::msg::DustbinidState dustbinidstate;
  int pre_recv_dustbin_id;
  //--机器人版本号
  std::string cti_run_ver;
  //--dataarray类型控制话题的发送结构体
  motion_to_clean_t_new send_to_clean_cmd_new;
  //--存放水量计算的结构体
  vehicle_water_box_status vehicle_water_status;
  std::string config_file_path;
  std::deque<uint8_t> vehicle_water_tank_top;
  std::deque<uint8_t> vehicle_water_tank_bottom;
  int max_vehicle_water_tank_restore = 6;
  //--取消定位和障碍物对底盘速度限制的开关
  bool localization_limit = true;
  //--通讯检测超时时间
  double box_chat_timeout_secs{0};
  double chassis_chat_timeout_secs{0};
  chat_state chassis_chat_state;
  chat_state box_chat_state;
  int dustbox_lora_id = -1;  // 清扫箱设定id号, -1:取消id

  bool device_set_flag{false};
  msg_light_cmd_3_0 light_cmd_v3_0;
  //--发送到集尘箱命令
  motion_to_dust_box_t send_to_dust_box_cmd;
  battery_board_cmd_t send_battery_board;
  process_work_mode process_twork_mode_t;

  std::shared_ptr<ChassisCmncBase> chassisCmncBase;
};

#endif
