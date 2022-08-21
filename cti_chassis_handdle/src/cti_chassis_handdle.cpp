#include "cti_chassis_handdle/cti_chassis_handdle.hpp"

using namespace std::chrono_literals;
// KN:name in log
constexpr char const *kN = "fpga-serial";

using namespace cti::log;

void ChassisHanddle::init() {
  initParam();
  initSub();
  initPub();
  initTimer();
  initCtiLog();
  initChassisCommunication();
  spinNode();
}

void ChassisHanddle::spinNode() {}

void ChassisHanddle::initChassisCommunication() {
  //串口通信
  if (port_type_name == "serial") {
    device_set_flag =
        this->chassisCmncBase->setDevice(port_type_name, serial_port);
  } else if (port_type_name == "udp") {
    device_set_flag = this->chassisCmncBase->setDevice(
        port_type_name, udp_ip, udp_port, udp_ip_dest, udp_port_dest);
  }
}

void ChassisHanddle::getChassisData() {
  unsigned char cmd;
  unsigned int length{0};
  unsigned char data[1024];
  bool flag{false};

  while(chassisCmncBase->getQueueSize() > 0) {
    chassisCmncBase->getDatas(cmd, length, data, flag);
    if (flag == true) {
      process_update_cmd_ex(cmd, data, length);
    } else {
      process_nomal_cmd(cmd, length, data);
    }
  }
}

void ChassisHanddle::sendChassisData() { chassisCmncBase->sendDatas(); }

void ChassisHanddle::initParam() {
  //----------------------获取参数
  //通讯端口类型及参数  "serial":串口通讯 "udp":网口udp通讯
  this->declare_parameter("port_type", std::string("serial"));
  port_type_name = this->get_parameter("port_type").as_string();
  //串口通讯参数
  this->declare_parameter("port_name", std::string("/dev/cti_fpga"));
  serial_port = this->get_parameter("port_name").as_string();
  //网口通讯参数
  this->declare_parameter("udp_ip", std::string("192.168.1.102"));
  udp_ip = this->get_parameter("udp_ip").as_string();
  this->declare_parameter("udp_port", 8888);
  udp_port = this->get_parameter("udp_port").as_int();
  this->declare_parameter("udp_ip_dest", std::string("192.168.1.222"));
  udp_ip_dest = this->get_parameter("udp_ip_dest").as_string();
  this->declare_parameter("udp_port_dest", 8887);
  udp_port_dest = this->get_parameter("udp_port_dest").as_int();
  this->declare_parameter("car_wheel_base", 0.8);
  car_wheel_base = this->get_parameter("car_wheel_base").as_double();

  this->declare_parameter("cmd_answer_timeout", 0.01);
  cmd_answer_timeout = this->get_parameter("cmd_answer_timeout").as_double();

  this->declare_parameter("max_control_board_version_head", 0);
  max_control_board_version_head_ =
      this->get_parameter("max_control_board_version_head").as_int();
  this->declare_parameter("min_control_board_version_head", 0);
  min_control_board_version_head_ =
      this->get_parameter("min_control_board_version_head").as_int();
  this->declare_parameter("max_control_board_version_mid", 0);
  max_control_board_version_mid_ =
      this->get_parameter("max_control_board_version_mid").as_int();
  this->declare_parameter("min_control_board_version_mid", 0);
  min_control_board_version_mid_ =
      this->get_parameter("min_control_board_version_mid").as_int();
  this->declare_parameter("max_control_board_version_end", 0);
  max_control_board_version_end_ =
      this->get_parameter("max_control_board_version_end").as_int();
  this->declare_parameter("min_control_board_version_end", 0);
  min_control_board_version_end_ =
      this->get_parameter("min_control_board_version_end").as_int();
  this->declare_parameter("default_dustbox_fanspeed", 80);
  this->declare_parameter("default_damboard_control", 0);

  this->declare_parameter("CTI_RUN_VER", "");
  cti_run_ver = this->get_parameter("CTI_RUN_VER").as_string();

  this->declare_parameter("config_file_path", "/home/neousys");
  config_file_path = this->get_parameter("config_file_path").as_string();
  config_file_path = config_file_path + "/config.yaml";
  // printf("config_file_path: %s\n",config_file_path.c_str());
  this->declare_parameter("water_out_per_sec", 0.0139);
  vehicle_water_status.out_per_sec =
      this->get_parameter("water_out_per_sec").as_double();
  this->declare_parameter("water_in_per_sec", 0.1);
  vehicle_water_status.in_per_sec =
      this->get_parameter("water_in_per_sec").as_double();
  this->declare_parameter("localization_limit", true);
  localization_limit = this->get_parameter("localization_limit").as_bool();
  this->declare_parameter("chassis_chat_timeout_secs", 10.0);
  chassis_chat_timeout_secs =
      this->get_parameter("chassis_chat_timeout_secs").as_double();
  this->declare_parameter("box_chat_timeout_secs", 10.0);
  box_chat_timeout_secs =
      this->get_parameter("box_chat_timeout_secs").as_double();
}

void ChassisHanddle::initSub() {
  //----------------------ros话题订阅定义

  //订阅话题
  sub_soft_stop = this->create_subscription<std_msgs::msg::Int8>(
      "/cti/rblite/emergency_stop_recovery", rclcpp::QoS{1},
      std::bind(&ChassisHanddle::soft_stop_callback, this,
                std::placeholders::_1));

  sub_cmd_vel = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", rclcpp::QoS{1},
      std::bind(&ChassisHanddle::cmd_vel_callback, this,
                std::placeholders::_1));

  box_sub = this->create_subscription<std_msgs::msg::Int32>(
      "cti/fpga_serial/boxcmd", rclcpp::QoS{1},
      std::bind(&ChassisHanddle::boxUpDown_Callback, this,
                std::placeholders::_1));

  stm32_sub = this->create_subscription<cti_chassis_msgs::msg::UpdateInfo>(
      "cti/fpga_serial/stmupdate", rclcpp::QoS{1},
      std::bind(&ChassisHanddle::stmUpdate_Callback, this,
                std::placeholders::_1));

  light_v3_0_sub = this->create_subscription<cti_msgs::msg::DataArray>(
      "cti/fpga_serial/light_type_v3_0", rclcpp::QoS{1},
      std::bind(&ChassisHanddle::lightType_v3_0_Callback, this,
                std::placeholders::_1));

  pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/lidar_pose", rclcpp::QoS{1},
      std::bind(&ChassisHanddle::position_Callback, this,
                std::placeholders::_1));

  checkversion_sub = this->create_subscription<std_msgs::msg::UInt8>(
      "/cti/fpga_serial/checkversion", rclcpp::QoS{1},
      std::bind(&ChassisHanddle::checkversion_Callback, this,
                std::placeholders::_1));

  formatsdcard_sub = this->create_subscription<std_msgs::msg::UInt8>(
      "/cti/fpga_serial/formatsdcard", rclcpp::QoS{1},
      std::bind(&ChassisHanddle::formatsdcard_Callback, this,
                std::placeholders::_1));

  localizerState_sub =
      this->create_subscription<cti_msgs::msg::RobotLocalizerState>(
          "/cti_localizer_state", rclcpp::QoS{1},
          std::bind(&ChassisHanddle::localizerState_Callback, this,
                    std::placeholders::_1));

  dustbin_control_sub_new =
          this->create_subscription<cti_msgs::msg::DustbinControlNew>(
              "/cti/chassis_serial/dustbin_control_new", rclcpp::QoS{1},
              std::bind(&ChassisHanddle::cleanControlNew_Callback, this,
                        std::placeholders::_1));
                  
  dustbin_control_info_sub =
      this->create_subscription<cti_msgs::msg::DataArray>(
          "/cti/chassis_serial/sanitation_vehicle_control", rclcpp::QoS{1},
          std::bind(&ChassisHanddle::cleanControlInfo_Callback, this,
                    std::placeholders::_1));

  wireless_charge_control_sub = this->create_subscription<std_msgs::msg::UInt8>(
      "/cti/chassis_serial/wireless_charge_control", rclcpp::QoS{1},
      std::bind(&ChassisHanddle::wirelessCharge_Callback, this,
                std::placeholders::_1));

  dust_box_autopush_control_sub =
      this->create_subscription<std_msgs::msg::UInt8>(
          "/cti/chassis_serial/dust_box_autopush_control", rclcpp::QoS{1},
          std::bind(&ChassisHanddle::dustBoxAutopushControl_Callback, this,
                    std::placeholders::_1));

  power_control_sub =
      this->create_subscription<std_msgs::msg::UInt32MultiArray>(
          "/cti/fpga_serial/power_control", rclcpp::QoS{1},
          std::bind(&ChassisHanddle::poweroff_cmd_type_callback, this,
                    std::placeholders::_1));

  exit_charging_sub = this->create_subscription<std_msgs::msg::UInt8>(
      "/cti/chassis_serial/exit_charging_cmd", rclcpp::QoS{1},
      std::bind(&ChassisHanddle::exit_charging_Callback, this,
                std::placeholders::_1));

  sprayControl_sub = this->create_subscription<std_msgs::msg::UInt8>(
      "/cti/chassis_serial/spray_control_cmd", rclcpp::QoS{1},
      std::bind(&ChassisHanddle::sprayControl_Callback, this,
                std::placeholders::_1));

  dust_box_control_info_sub = this->create_subscription<cti_msgs::msg::DataArray>(
      "/cti/chassis_serial/sanitation_dustbox_control", rclcpp::QoS{1},
      std::bind(&ChassisHanddle::dustBoxControlInfo_Callback, this,
                std::placeholders::_1));
}

void ChassisHanddle::initPub() {
  //----------------------ros话题发布定义

  odom_pub_4wd = this->create_publisher<nav_msgs::msg::Odometry>(
      "/robot_odom_4wd", rclcpp::QoS{1});
  odom_pub_calc = this->create_publisher<nav_msgs::msg::Odometry>(
      "/robot_odom_calc", rclcpp::QoS{1});
  sins_pub = this->create_publisher<cti_msgs::msg::Sins>(
      "/cti/fpga_serial/sins", rclcpp::QoS{1});

  imudata_pub = this->create_publisher<sensor_msgs::msg::Imu>(
      "/cti/fpga_serial/imu", rclcpp::QoS{1});
  batcell_pub = this->create_publisher<cti_msgs::msg::BatteryCellsState>(
      "/cti/cti_fpga/batcell_state", rclcpp::QoS{1});
  ctlinfo_pub = this->create_publisher<cti_msgs::msg::VehicleCtlRunInfo>(
      "/cti/fpga_serial/ctlrun_info", rclcpp::QoS{1});

  state_pub = this->create_publisher<std_msgs::msg::String>(
      "/cti/fpga_serial/error", rclcpp::QoS{1});
  stm32_pub = this->create_publisher<cti_chassis_msgs::msg::UpdateInfo>(
      "/cti/fpga_serial/stminfo", rclcpp::QoS{1});

  cmd_answer_pub = this->create_publisher<cti_chassis_msgs::msg::CmdAnswer>(
      "/cti/fpga_serial/cmd_answer_cnt", rclcpp::QoS{1});

  for (int i = 0; i < ULT_3_0_TYPE_NUM; i++) {
    alt_3_0_pub[i] = this->create_publisher<sensor_msgs::msg::Range>(
        "/cti/chassis_serial/" + ult_3_0_name[i], rclcpp::QoS{1});
  }

  firmvion_pub = this->create_publisher<cti_msgs::msg::RobotVersionDisplay>(
      "/cti/fpga_serial/operationControlVersion",
      rclcpp::QoS{1}.transient_local());
  box_laser_pub = this->create_publisher<cti_msgs::msg::Rtcm>(
      "/cti/fpga_serial/box_laser", rclcpp::QoS{1});
  chassis_error_pub = this->create_publisher<std_msgs::msg::UInt32MultiArray>(
      "/cti/chassis_serial/chassis_error", rclcpp::QoS{1});
  navigation_log_pub =
      this->create_publisher<cti_chassis_msgs::msg::NavigationLog>(
          "/cti/fpga_serial/navigation_log", rclcpp::QoS{1});
  baro_status_pub = this->create_publisher<std_msgs::msg::UInt16>(
      "/cti/chassis_serial/baro_status", rclcpp::QoS{1});
  formatsdcard_pub = this->create_publisher<std_msgs::msg::UInt8>(
      "/cti/fpga_serial/formatsdcard_result", rclcpp::QoS{1});
  gps_pub = this->create_publisher<cti_msgs::msg::GnssRTK>(
      "/cti/chassis_serial/gnss", rclcpp::QoS{1});
  compass_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>(
      "/cti/chassis_serial/compass", rclcpp::QoS{1});
  firmware_version_status_pub = this->create_publisher<std_msgs::msg::Int8>(
      "/cti/chassis_serial/control_board_version_status", rclcpp::QoS{1});
  firmware_version_check_pub =
      this->create_publisher<cti_chassis_msgs::msg::FirmWareInfo>(
          "/cti/chassis_serial/FW_check_report", rclcpp::QoS{1});
  recv_chassis_info_pub =
      this->create_publisher<cti_chassis_msgs::msg::VehicleState>(
          "/cti/chassis_serial/chassis_info", rclcpp::QoS{1});
  dust_box_state_info_pub = this->create_publisher<cti_msgs::msg::DataArray>(
      "/cti/chassis_serial/sanitation_dustbox_state",
      rclcpp::QoS{1});  // cti_msgs/DataArray类型//环卫车,吸尘箱状态发布
  dust_box_autopush_pub = this->create_publisher<std_msgs::msg::UInt8>(
      "/cti/chassis_serial/dust_box_autopush_state", rclcpp::QoS{1});
  boxlock_state_pub = this->create_publisher<std_msgs::msg::UInt8>(
      "/cti/chassis_serial/boxlock_state", rclcpp::QoS{1});
  rain_sensor_pub = this->create_publisher<cti_msgs::msg::BoxState>(
      "/cti/chassis_serial/rain_sensor", rclcpp::QoS{1});
  dust_vehicle_state_info_pub =
      this->create_publisher<cti_msgs::msg::DataArray>(
          "/cti/chassis_serial/sanitation_vehicle_state",
          rclcpp::QoS{1});  // cti_msgs/DataArray类型//环卫车,车辆清扫状态发布
  chat_statue_pub = this->create_publisher<cti_msgs::msg::BoxState>(
      "/cti/chassis_serial/communication_state",
      rclcpp::QoS{1});  //通信状态发布

  lin_ult_data_v3_0_pub =
      this->create_publisher<std_msgs::msg::UInt16MultiArray>(
          "/cti/chassis_serial/lin_ult_data_v3_0", rclcpp::QoS{1});

  new_lin_ult_data_v3_0_pub =
      this->create_publisher<cti_chassis_msgs::msg::UltV30Datas>(
          "/cti/chassis_serial/new_lin_ult_data_v3_0",
          rclcpp::QoS{1});  // new环卫车3.0lin通信超声波源数据发布 =

  rtk_pub =
      this->create_publisher<cti_msgs::msg::GnssRTK>(
          "/cti/chassis_serial/rtk", rclcpp::QoS{1});

  shallow_sleep_pub =
      this->create_publisher<std_msgs::msg::Bool>(
          "/cti/chassis/shallow_sleep", rclcpp::QoS{1});
}

void ChassisHanddle::initTimer() {
  //----------------------ros定时器定义
  main_timer = this->create_wall_timer(std::chrono::milliseconds(1),
                                       [this]() { this->mainTimerCallback(); });

  timer = this->create_wall_timer(1s, [this]() { this->timerCallback(); });

  timer2 = this->create_wall_timer(1s, [this]() { this->timer2Callback(); });

  timer_chat_timeout = this->create_wall_timer(
      1s, [this]() { this->timer_chat_timeout_Callbak(); });  //通信超时检测
  timer_process_work_mode = this->create_wall_timer(
      1s, [this]() { this->timer_set_process_work_mode_Callback(); });  //
}

void ChassisHanddle::initLog(const std::string name) {
  // set the logger file name, defaut is "logger.log"
  Logger::setDefaultLogger(name);
  // set log output mode {CoutOrCerr,File,Both}
  Logger::getLogger().setOutputs(Logger::Output::File);
  // set log level mode {Fata,Erro,Warn,Note,Info,Trac,Debu,Deta}
  Logger::getLogger().setLogLevel(LogLevel::Info);
  // enable display thread id , defaut is "false"
  Logger::getLogger().enableTid(false);
  // enable display line id number, defaut is "true"
  Logger::getLogger().enableIdx(true);
  // 1M  默认最小8*1024 默认最大sizeof(long)*32*1024*1024
  Logger::getLogger().setMaxSize(200 * 1024 * 1024);
}

void ChassisHanddle::initCtiLog() {
  std::string filelog;
  //----------------------log文件名称
  this->declare_parameter("CTI_RUN_LOG_PATH", "");
  if (this->has_parameter("CTI_RUN_LOG_PATH")) {
    filelog = this->get_parameter("CTI_RUN_LOG_PATH").as_string();
    filelog += "/control.log";
  } else {
    this->declare_parameter("filenamelog", "/home/neousys/log/control.log");
    filelog = this->get_parameter("filenamelog").as_string();
  }
  filelog = std::string("/home/neousys/log/control.log");
  std::cout << "filenamelog:" << filelog << std::endl;
  initLog(filelog);
}

//************************************** 浅睡眠话题回调函数
void ChassisHanddle::poweroff_cmd_type_callback(
    const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
  // int x=0b1001;
  send_to_poweroff_cmd_type_new send_to_poweroff_cmd_type_new_;
  send_to_poweroff_cmd_type_new_.cmd_id = cmd_id_global++;
  send_to_poweroff_cmd_type_new_.power_off_flag[0] = msg->data[0];
  send_to_poweroff_cmd_type_new_.power_control = msg->data[1];
  send_to_poweroff_cmd_type_new_.bat_control = msg->data[2];

  chassisCmncBase->queueSendDatas(SEND_TO_POWEROFF_CMD,
                                  sizeof(send_to_poweroff_cmd_type_new),
                                  &send_to_poweroff_cmd_type_new_, false);
}

//************************************** 软急停控制回调函数
void ChassisHanddle::soft_stop_callback(
    const std_msgs::msg::Int8::SharedPtr msg) {
  send_battery_board.soft_stop = msg->data;

  chassisCmncBase->queueSendDatas(SEND_BATTERY_CMD, sizeof(battery_board_cmd_t),
                                  &send_battery_board, false);

  send_battery_board.soft_stop = 0;  //复位后为０
}

//************************************** 控制命令话题回调函数
void ChassisHanddle::cmd_vel_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr twistStamped) {
  if (stm32_update_flag) {
    return;
  }

  //测试模式下不受到其他上游控制
  if (process_twork_mode_t.process_work_mode_enum_ ==
          process_work_mode::process_work_mode_test ||
      process_twork_mode_t.SPRAY_LOCK == true) {
    if (twistStamped->header.frame_id != "test") {
      return;
    }
  }

  if (control_version_right != 0) {
    Info("运控版本对应不上，超前或滞后");
    return;
  }
  double msgs_time = rclcpp::Time(twistStamped->header.stamp).seconds();
  double now_time = rclcpp::Clock().now().seconds();
  if ((now_time - msgs_time) > 1) {
    return;
  }
  rclcpp::Time start_time = rclcpp::Clock().now();
  send_to_control_cmd_type control_cmd;
  send_ctl_portIndex_cnt++;
  send_ctl_portIndex_cnt %= 255;
  control_cmd.portIndex = send_ctl_portIndex_cnt;
  control_cmd.linkCnt = 0;
  control_cmd.cmd_id = cmd_id_global++;
  control_cmd.linkFlag = 1;
  control_cmd.cmd_vel_Vx = twistStamped->twist.linear.x;
  // node_status_publisher_ptr_->CHECK_MAX_VALUE("/value/cti_fpga_serial/control_cmd/vx",control_cmd.cmd_vel_Vx,4.5,5,"value
  // control_cmd:vx is too high");
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
  if (0 != robotlocalizerstate_global) {
    control_cmd.robot_status.bits.localizer = 1;
  } else {
    control_cmd.robot_status.bits.localizer = 0;
  }

  chassisCmncBase->queueSendDatas(SEND_TO_CONTROL_CMD,
                                  sizeof(send_to_control_cmd_type),
                                  &control_cmd, false);

  chassisCmncBase->queueSendDatas(SEND_TO_LIGHT_CMD, sizeof(msg_light_cmd_3_0),
                                  &light_cmd_v3_0, false);

  Info("S_CT_PI: " << control_cmd.cmd_id
                   << " S_CT_VX: " << twistStamped->twist.linear.x
                   << " S_CT_VY: " << twistStamped->twist.linear.y
                   << " S_CT_TA: " << twistStamped->twist.angular.z
                   << " S_CT_TM: " << abs(twistStamped->twist.angular.x)
                   << " S_CT_LT: " << light_type << " S_CT_K: " << global_k
                   << " S_CT_KB: " << global_k_back
                   << " S_CT_BK: " << abs(twistStamped->twist.linear.z)
                   << " S_CT_LP: " << (int)global_up_down_flag
                   << " S_CT_LL: " << robotlocalizerstate_global
                   << " S_CT_BL: " << (int)control_cmd.switch_flag);
  static int send_loop_cnt = 0;
  send_loop_cnt++;
  if (send_loop_cnt >= 20) {
    Info(" S_CT_PS: " << global_pose[0] << " " << global_pose[1] << " "
                      << global_pose[2] << " S_CT_Q: " << global_q[0] << " "
                      << global_q[1] << " " << global_q[2] << " "
                      << global_q[3]);
    send_loop_cnt = 0;
  }
}

//**************************************
//集尘箱控制回调函数,cti_msgs::msg::DataArray 消息类型
void ChassisHanddle::dustBoxControlInfo_Callback(
    const cti_msgs::msg::DataArray::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  //数据解析
  for (int i = 0; i < msg->datas.size(); i++) {
    cti_msgs::msg::Data info_msg = msg->datas[i];
    if (info_msg.name == "engin_start")
      //吸尘箱启动 uint8_t
      send_to_dust_box_cmd.engin_start = atoi(info_msg.data.c_str());
    else if (info_msg.name == "fanspeed")
      //风机速度  uint8_t
      send_to_dust_box_cmd.fan_speed = atoi(info_msg.data.c_str());
    else if (info_msg.name == "led")
      // led灯 uint8_t
      send_to_dust_box_cmd.led = atoi(info_msg.data.c_str());
    else if (info_msg.name == "ultrasonic_clean")
      // led灯 uint8_t
      send_to_dust_box_cmd.ultrasonic_clean = atoi(info_msg.data.c_str());
    else
      continue;
  }

  
  send_to_dust_box_cmd.control_mode = 2;  // 2：导航控制
  send_to_dust_box_cmd.port = 0;
  send_to_dust_box_cmd.lift_motor = 0;
  send_to_dust_box_cmd.main_brush = 0;
  send_to_dust_box_cmd.spray_motor = 0;
  send_to_dust_box_cmd.dust_suppresion = 0;
  send_to_dust_box_cmd.side_brush = 0;
  send_to_dust_box_cmd.unused1 = 0;

  chassisCmncBase->queueSendDatas(SEND_TO_DUST_BOX_CMD,
                                  sizeof(send_to_dust_box_cmd), &send_to_dust_box_cmd,
                                  false);

  Info("S_DBO_ES: " << (int)send_to_dust_box_cmd.engin_start
                    << "S_DBO_FS: " << (int)send_to_dust_box_cmd.fan_speed);
}


//************************************** 升降箱命令话题回调函数
void ChassisHanddle::boxUpDown_Callback(
    const std_msgs::msg::Int32::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  global_up_down_flag = msg->data;
  static int old_data = std::numeric_limits<int>::min();
  if (msg->data != old_data) {
    old_data = msg->data;
  }
}

//************************************** 固件升级命令话题回调函数
void ChassisHanddle::stmUpdate_Callback(
    const cti_chassis_msgs::msg::UpdateInfo::SharedPtr msg) {
  TIMEOUT = 0;  //清空定时器

  if (msg->seq_num >= 1) {
    stm32_update_flag = true;
    this->chassisCmncBase->send_serial_update_flow(msg->data.data(),
                                                   msg->data.size());

  } else {
    stm32_update_flag = false;
    seq_num = 0;
  }
}

//**************************************3.0 light control callback
void ChassisHanddle::lightType_v3_0_Callback(
    const cti_msgs::msg::DataArray::SharedPtr msg) {
  for (int i = 0; i < msg->datas.size(); i++) {
    cti_msgs::msg::Data info_msg = msg->datas[i];
    if (info_msg.name == "warning_light")
      light_cmd_v3_0.warning_light = atoi(info_msg.data.c_str());
    if (info_msg.name == "headlight_right")
      light_cmd_v3_0.headlight_right = atoi(info_msg.data.c_str());
    if (info_msg.name == "headlight_left")
      light_cmd_v3_0.headlight_left = atoi(info_msg.data.c_str());
    if (info_msg.name == "backlight_right")
      light_cmd_v3_0.backlight_right = atoi(info_msg.data.c_str());
    if (info_msg.name == "backlight_left")
      light_cmd_v3_0.backlight_left = atoi(info_msg.data.c_str());
    if (info_msg.name == "headlight_circle")
      light_cmd_v3_0.headlight_circle = atoi(info_msg.data.c_str());
    if (info_msg.name == "backlight_circle")
      light_cmd_v3_0.backlight_circle = atoi(info_msg.data.c_str());
    if (info_msg.name == "turn_light_right")
      light_cmd_v3_0.turn_light_right = atoi(info_msg.data.c_str());
    if (info_msg.name == "turn_light_left")
      light_cmd_v3_0.turn_light_left = atoi(info_msg.data.c_str());
    if (info_msg.name == "break_light")
      light_cmd_v3_0.break_light = atoi(info_msg.data.c_str());
    if (info_msg.name == "beep")
      light_cmd_v3_0.beep = atoi(info_msg.data.c_str());
  }
}

//************************************** 位置命令话题回调函数
void ChassisHanddle::position_Callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  global_pose[0] = msg->pose.position.x;
  global_pose[1] = msg->pose.position.y;
  global_pose[2] = msg->pose.position.z;
  global_q[0] = msg->pose.orientation.w;
  global_q[1] = msg->pose.orientation.x;
  global_q[2] = msg->pose.orientation.y;
  global_q[3] = msg->pose.orientation.z;
}

//************************************** 查询固件版本号命令话题回调函数
void ChassisHanddle::checkversion_Callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  send_to_check_version_type check_version_cmd;
  update_info_type update_info_struct;
  update_info_struct.src = MODULE_CHECK_UPD;
  update_info_struct.dest = msg->data;
  check_version_cmd.upd_info = update_info_struct;
  check_version_cmd.check = 01;

  chassisCmncBase->queueSendDatas(SEND_TO_CHECK_PROGRAM_VERSION,
                                  sizeof(check_version_cmd), &check_version_cmd,
                                  false);
}

//************************************** 格式化SD卡命令话题回调函数
void ChassisHanddle::formatsdcard_Callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  send_format_sd_card_cmd_type format_sdcard_cmd;
  format_sdcard_cmd.portIndex = msg->data;

  chassisCmncBase->queueSendDatas(SEND_FORMAT_SD_CARD_CMD,
                                  sizeof(format_sdcard_cmd), &format_sdcard_cmd,
                                  false);
}

void ChassisHanddle::cleanControlNew_Callback(
    const cti_msgs::msg::DustbinControlNew::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  //测试模式　定时过滤１０分钟数据
  if (msg->control_mode == process_work_mode::process_work_mode_test) {
    process_twork_mode_t.cur_cnt =
        process_work_mode::PROCESS_WORK_MODE_INTERVAL;
    process_twork_mode_t.process_work_mode_enum_ =
        process_work_mode::process_work_mode_test;
  }

  motion_to_clean_t_new send_to_clean_cmd_new;
  send_to_clean_cmd_new.port = 0;

  //借用灯的位置测下水泵
  send_to_clean_cmd_new.lift_pump_switch = msg->led;
  send_to_clean_cmd_new.shake_dust_motor_speed = msg->dust_suppresion;
  //借用灯的位置测下水泵 集尘电机

  send_to_clean_cmd_new.engin_start = msg->engin_start;
  send_to_clean_cmd_new.side_brush = msg->side_brush;
  send_to_clean_cmd_new.main_brush = msg->main_brush;
  send_to_clean_cmd_new.spray_motor = msg->spray_motor;
  // send_to_clean_cmd_new.dust_suppresion = msg->dust_suppresion;
  send_to_clean_cmd_new.lift_motor = msg->lift_motor;
  // send_to_clean_cmd_new.led = msg->led;
  send_to_clean_cmd_new.control_mode = msg->control_mode;  // 2：导航控制

  send_to_clean_cmd_new.dam_board = msg->dam_board;
  send_to_clean_cmd_new.side_brush_transform = msg->side_brush_transform;
  send_to_clean_cmd_new.side_brush_speed = msg->side_brush_speed;
  send_to_clean_cmd_new.unused1 = msg->unused1;
  send_to_clean_cmd_new.unused2 = msg->unused2;
  send_to_clean_cmd_new.unused3 = msg->unused3;


  chassisCmncBase->queueSendDatas(SEND_TO_DUSTBIN_CMD,
                                  sizeof(send_to_clean_cmd_new),
                                  &send_to_clean_cmd_new, false);

  Info("S_CC_ES: " << (int)msg->engin_start
                   << " S_CC_LM: " << (int)msg->lift_motor
                   << " S_CC_MB: " << (int)msg->main_brush
                   << " S_CC_SM: " << (int)msg->spray_motor
                   << " S_CC_DS: " << (int)msg->dust_suppresion << " S_CC_SB: "
                   << (int)msg->side_brush << " S_CC_LED: " << (int)msg->led
                   << " S_CC_DOC: " << (int)send_to_clean_cmd_new.dustbin_on_car
                   << " S_CC_DB: " << (int)msg->dam_board
                   << " S_CC_SBT: " << (int)msg->side_brush_transform
                   << " S_CC_SBS: " << (int)msg->side_brush_speed
                   << " S_CC_UN1: " << (int)msg->unused1 << " S_CC_UN2: "
                   << (int)msg->unused2 << " S_CC_UN3: " << (int)msg->unused3);
}

//
void ChassisHanddle::cleanControlInfo_Callback(
    const cti_msgs::msg::DataArray::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }

  //测试模式下不受到其他上游控制
  if (process_twork_mode_t.process_work_mode_enum_ ==
      process_work_mode::process_work_mode_test ||
      process_twork_mode_t.SPRAY_LOCK == true) {
    return;
  }

  //数据解析
  for (int i = 0; i < msg->datas.size(); i++) {
    cti_msgs::msg::Data info_msg = msg->datas[i];
    if (info_msg.name == "engin_start")
      //一键开启 uint8_t  1    0
      send_to_clean_cmd_new.engin_start = atoi(info_msg.data.c_str());
    else if (info_msg.name == "spray_motor")
    //喷水电机 uint8_t 1/0    0
    {
      if (atoi(info_msg.data.c_str()) != 0) {
        send_to_clean_cmd_new.spray_motor = 8;
      } else {
        send_to_clean_cmd_new.spray_motor = 0;
      }
    } else if (info_msg.name == "sidebrush_lift")
      //边刷升降 int8_t 1   0
      send_to_clean_cmd_new.lift_motor = atoi(info_msg.data.c_str());
    else if (info_msg.name == "led")
      // led灯 uint8_t 1   0
      send_to_clean_cmd_new.led = atoi(info_msg.data.c_str());
    else if (info_msg.name == "dam_board")
      //挡板控制 uint8_t 0   0
      send_to_clean_cmd_new.dam_board = atoi(info_msg.data.c_str());
    else if (info_msg.name == "sidebrush_transform")
      //边刷伸展 uint8_t 100   0
      send_to_clean_cmd_new.side_brush_transform = atoi(info_msg.data.c_str());
    else if (info_msg.name == "sidebrush_speed")
      //边刷转速 uint8_t 100  0
      send_to_clean_cmd_new.side_brush_speed = atoi(info_msg.data.c_str());
    else if (info_msg.name == "decorate_light")
      //装饰灯 uint8_t 0:关 1:开
      send_to_clean_cmd_new.decorate_light = atoi(info_msg.data.c_str());
    else if (info_msg.name == "lift_motor")
      //边刷升降 int8_t 0:停止 1:上升 -1：下降
      send_to_clean_cmd_new.lift_motor = atoi(info_msg.data.c_str());
    else if (info_msg.name == "control_mode")
      //
      send_to_clean_cmd_new.control_mode = atoi(info_msg.data.c_str());
    else if (info_msg.name == "lift_pump_switch")
      //水泵开关
      send_to_clean_cmd_new.lift_pump_switch = atoi(info_msg.data.c_str());
    else if (info_msg.name == "shake_dust_motor_speed")
      //震尘速度百分比
      send_to_clean_cmd_new.shake_dust_motor_speed =
          atoi(info_msg.data.c_str());
    else
      continue;
  }

  send_to_clean_cmd_new.port = 0;
  send_to_clean_cmd_new.side_brush = 0;
  send_to_clean_cmd_new.main_brush = 0;
  send_to_clean_cmd_new.dust_suppresion = 0;
  // send_to_clean_cmd_new.control_mode = 2;  // 2：导航控制
  send_to_clean_cmd_new.unused1 = 0;
  send_to_clean_cmd_new.unused2 = 0;
  send_to_clean_cmd_new.unused3 = 0;

  std::string clean_flag = "v6.0";
  std::string::size_type idx = cti_run_ver.find(clean_flag);
  if (idx != std::string::npos) {
  } else {
    chassisCmncBase->queueSendDatas(SEND_TO_DUSTBIN_CMD,
                                    sizeof(send_to_clean_cmd_new),
                                    &send_to_clean_cmd_new, false);
  }

  Info("S_CC_ES: "
       << (int)send_to_clean_cmd_new.engin_start
       << " S_CC_LM: " << (int)send_to_clean_cmd_new.lift_motor
       << " S_CC_MB: " << (int)send_to_clean_cmd_new.main_brush
       << " S_CC_SM: " << (int)send_to_clean_cmd_new.spray_motor
       << " S_CC_DS: " << (int)send_to_clean_cmd_new.dust_suppresion
       << " S_CC_SB: " << (int)send_to_clean_cmd_new.side_brush
       << " S_CC_LED: " << (int)send_to_clean_cmd_new.led
       << " S_CC_DOC: " << (int)send_to_clean_cmd_new.dustbin_on_car
       << " S_CC_DB: " << (int)send_to_clean_cmd_new.dam_board
       << " S_CC_SBT: " << (int)send_to_clean_cmd_new.side_brush_transform
       << " S_CC_SBS: " << (int)send_to_clean_cmd_new.side_brush_speed
       << " S_CC_DE_LG: " << (int)send_to_clean_cmd_new.decorate_light
       << " S_CC_UN1: " << (int)send_to_clean_cmd_new.unused1
       << " S_CC_UN2: " << (int)send_to_clean_cmd_new.unused2
       << " S_CC_UN3: " << (int)send_to_clean_cmd_new.unused3);
}

//************************************** ３。０喷水控制
void ChassisHanddle::sprayControl_Callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {

  if (msg->data != process_work_mode::process_work_mode_stop_spray) {
    process_twork_mode_t.SPRAY_LOCK = true;
  }
  else if(msg->data == process_work_mode::process_work_mode_stop_spray)
  {
    process_twork_mode_t.SPRAY_LOCK = false;
  }

  send_to_clean_cmd_new.engin_start = 0;
  send_to_clean_cmd_new.lift_motor = 0;
  send_to_clean_cmd_new.main_brush = 0;
  send_to_clean_cmd_new.dust_suppresion = 0;
  send_to_clean_cmd_new.side_brush = 0;
  send_to_clean_cmd_new.led = 0;
  send_to_clean_cmd_new.dustbin_on_car = 0;
  send_to_clean_cmd_new.dam_board = 0;
  send_to_clean_cmd_new.side_brush_transform = 0;
  send_to_clean_cmd_new.side_brush_speed = 0;
  send_to_clean_cmd_new.decorate_light = 0;
  send_to_clean_cmd_new.unused1 = 0;
  send_to_clean_cmd_new.unused2 = 0;
  send_to_clean_cmd_new.unused3 = 0;

  // 0x00// 关闭喷水
  // 0x01// 后端球头雷达
  // 0x02// 超声波满溢探头清洗
  // 0x04// 头顶16线雷达zx
  // 0x08// 抑尘喷头
  // 0x10// 前端球头雷达
  send_to_clean_cmd_new.spray_motor = msg->data;
  send_to_clean_cmd_new.control_mode = 2;  // 2：一般控制模式

  chassisCmncBase->queueSendDatas(SEND_TO_DUSTBIN_CMD,
                                  sizeof(send_to_clean_cmd_new),
                                  &send_to_clean_cmd_new, false);
}

void ChassisHanddle::wirelessCharge_Callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  Info("S_WC: " << (int)msg->data);
  send_battery_board.soft_stop = 0;  //　软急停　０：无作为　１：复位　
  send_battery_board.wireless_charge = msg->data;

  chassisCmncBase->queueSendDatas(SEND_BATTERY_CMD, sizeof(send_battery_board),
                                  &send_battery_board, false);
}

void ChassisHanddle::exit_charging_Callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  Info("S_EX_CG: " << (int)msg->data);
  send_battery_board.exit_charging_state = msg->data;
  std::cout << "退出充电状态" << std::endl;

  chassisCmncBase->queueSendDatas(SEND_BATTERY_CMD, sizeof(send_battery_board),
                                  &send_battery_board, false);

  send_battery_board.exit_charging_state = 0;
}

//************************************** ３．０超声波发布话题
void ChassisHanddle::pub_alt_3_0_(const msg_upa_pos_data_t *data) {
  cti_chassis_msgs::msg::UltV30Datas ult_datas;
  ult_datas.ori_datas.data.push_back((double)data->upa_data[2]);
  ult_datas.ori_datas.data.push_back((double)data->upa_data[3]);
  ult_datas.ori_datas.data.push_back((double)data->upa_data[6]);
  ult_datas.ori_datas.data.push_back((double)data->upa_data[7]);

  ult_datas.pos_front_datas.push_back((double)data->pos_dat[0][0]);
  ult_datas.pos_front_datas.push_back((double)data->pos_dat[0][1]);

  ult_datas.pos_back_datas.push_back((double)data->pos_dat[1][0]);
  ult_datas.pos_back_datas.push_back((double)data->pos_dat[1][1]);

  new_lin_ult_data_v3_0_pub->publish(ult_datas);

  Info("T_UL_PF1: " << (double)data->pos_flag[0]
                    << " T_UL_PF2: " << (double)data->pos_flag[1]
                    << " T_UL_SS1: " << (double)data->upa_data[0]
                    << " T_UL_SS2: " << (double)data->upa_data[1]
                    << " T_UL_SS3: " << (double)data->upa_data[2]
                    << " T_UL_SS4: " << (double)data->upa_data[3]
                    << " T_UL_SS5: " << (double)data->upa_data[4]
                    << " T_UL_SS6: " << (double)data->upa_data[5]
                    << " T_UL_SS7: " << (double)data->upa_data[6]
                    << " T_UL_SS8: " << (double)data->upa_data[7]
                    << " T_UL_PSX1: " << (double)data->pos_dat[0][0]
                    << " T_UL_PSY1: " << (double)data->pos_dat[0][1]
                    << " T_UL_PSX2: " << (double)data->pos_dat[1][0]
                    << " T_UL_PSY2: " << (double)data->pos_dat[1][1]);

  ////////////////////////////////////////////////////
}

//************************************** 集尘箱自动倒垃圾控制回调函数
void ChassisHanddle::dustBoxAutopushControl_Callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  send_to_dust_box_cmd.auto_push = msg->data;

  chassisCmncBase->queueSendDatas(SEND_TO_DUST_BOX_CMD,
                                  sizeof(send_to_dust_box_cmd),
                                  &send_to_dust_box_cmd, false);

  Info("S_DBO_AP: " << (int)msg->data);
}

//************************************** 由四元数计算yaw
double ChassisHanddle::calcYawFromQuaternion(const tf2::Quaternion &q) {
  return tf2::impl::getYaw(q);
}

//************************************** 16进制转string函数
std::string ChassisHanddle::hex2string(uint8_t *data, unsigned int Len) {
  int i = 0;
  std::stringstream ss;
  for (i = 0; i < Len; i++) {
    ss << std::setw(2) << std::setfill('0') << std::hex << (int)data[i];
  }
  std::string str(ss.str());
  return str;
}

//************************************** 接收到控制状态后发布话题
void ChassisHanddle::pub_odom(const recv_from_control_status_type *data) {
  if (!data) {
    return;
  }

  //-------------------通讯检测------------------
  chassis_chat_state.module_id = 0;
  chassis_chat_state.recv_state = true;
  chassis_chat_state.time_recv = rclcpp::Clock().now().seconds();
  //---------------------------odom_4wd publish
  tf2::Quaternion q_tf;
  q_tf.setRPY(0.0, 0.0, data->angle_yaw_radian);
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q_tf);
  odom_4wd.header.stamp = rclcpp::Clock().now();
  odom_4wd.header.frame_id = "odom";
  // set the position
  odom_4wd.pose.pose.position.x = data->pos.x;
  odom_4wd.pose.pose.position.y = data->pos.y;
  odom_4wd.pose.pose.position.z = 0.0;
  odom_4wd.pose.pose.orientation = odom_quat;
  // set the velocity
  odom_4wd.child_frame_id = "base_link";
  odom_4wd.twist.twist.linear.x =
      data->vel_liner_x;  // vx == 100时！是底盘有问题！！！！！！！！！
  odom_4wd.twist.twist.linear.y = data->vel_liner_y;
  odom_4wd.twist.twist.angular.z = data->vel_angular;
  // publish the message
  odom_pub_4wd->publish(odom_4wd);
  //---------------------------ctlinfo publish
  runinfo.state_vehicle = data->state_vehicle;    //底盘状态
  runinfo.drivers_enable = data->drivers_enable;  //电机驱动使能
  runinfo.control_mode = data->control_mode;  //控制方式 1:遥控 2：导航
  runinfo.state_brake = data->state_brake;    //刹车
  runinfo.vel_liner_x =
      data->vel_liner_x;  // x轴线速度    vx ==
                          // 100时！是底盘有问题！！！！！！！！！
  runinfo.vel_liner_y = data->vel_liner_y;            // Y轴线速度
  runinfo.vel_angular = data->vel_angular;            //角速度
  runinfo.angle_front_turn = data->angle_front_turn;  //前轮转角
  runinfo.angle_rear_turn = data->angle_rear_turn;    //后轮转角
  //霍尔信号
  uint8_t box_sw = 0;
  box_sw = box_sw | (data->sw_status.bits.box_pos_reach1 & 0x01);
  box_sw = box_sw | ((data->sw_status.bits.box_pos_reach2 << 1) & 0x02);
  box_sw = box_sw | ((data->sw_status.bits.box_pos_reach3 << 2) & 0x04);
  //触边
  box_sw = box_sw | ((!(data->sw_status.bits.lift_pressure) << 3) & 0x08);
  runinfo.box_sw = box_sw;                                //霍尔信号+触边
  runinfo.front_bar = !(data->sw_status.bits.front_bar);  //前防撞杆
  runinfo.rear_bar = !(data->sw_status.bits.rear_bar);    //后防撞杆
  runinfo.lift_position = data->lift_position;            //顶升位置
  runinfo.odometer = data->odometer;                      //里程
  ctlinfo_pub->publish(runinfo);
  //---------------------------Imu publish

  // imu 时间戳
  imu_info.header.stamp = rclcpp::Clock().now();
  imu_info.header.frame_id = "/fpga_serial/imu";
  //四元数位姿
  imu_info.orientation.w = data->q[0];
  imu_info.orientation.x = data->q[1];
  imu_info.orientation.y = data->q[2];
  imu_info.orientation.z = data->q[3];
  //线加速度
  imu_info.linear_acceleration.x = data->acc[0] * GRAV;
  imu_info.linear_acceleration.y = data->acc[1] * GRAV;
  imu_info.linear_acceleration.z = -data->acc[2] * GRAV;
  //角速度
  imu_info.angular_velocity.x = data->gyro[0] * PI / 180.0;
  imu_info.angular_velocity.y = data->gyro[1] * PI / 180.0;
  imu_info.angular_velocity.z = -data->gyro[2] * PI / 180.0;
  //发布
  imudata_pub->publish(imu_info);
  // sins时间戳
  sins_info.header.stamp = imu_info.header.stamp;
  sins_info.header.frame_id = "/fpga_serial/sins";
  // imu
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
  if (abs(data->baro_raw - baro_raw_old) > 100) {
    baro_status.data = 1;
    baro_status_pub->publish(baro_status);
    Info("BARO_RAW_ERROR: "
         << " rawdata_before: " << baro_raw_old
         << " rawdata_now: " << data->baro_raw);
    baro_status.data = 0;
  }
  baro_raw_old = data->baro_raw;
  sins_info.barometer_height = data->baro_height;
  //标志位
  sins_info.state_flag = data->state_flag;
  //发布
  sins_pub->publish(sins_info);

  //---------------------------box_laser publish
  box_laser.header.stamp = rclcpp::Clock().now();
  box_laser.header.frame_id = "/fpga_serial/box_laser";
  uint8_t id_data[6];
  for (int i = 0; i < 6; i++) {
    id_data[i] = data->laser_id[i];
  }
  std::string id_str;
  id_str = hex2string(&(id_data[0]), 6);
  box_laser.rtcm_type = id_str;
  box_laser.data.push_back(data->laser_data[0]);
  box_laser.data.push_back(data->laser_data[1]);
  box_laser_pub->publish(box_laser);
  box_laser.data.clear();
  //------------------------磁吸锁状态发布
  std_msgs::msg::UInt8 boxlockstate;
  boxlockstate.data = data->laser_id[0];
  boxlock_state_pub->publish(boxlockstate);

  //--------------------------磁力计发布
  std_msgs::msg::Int32MultiArray compass_msg;
  compass_msg.data.push_back(data->compass1_str[0]);
  compass_msg.data.push_back(data->compass1_str[1]);
  compass_msg.data.push_back(data->compass1_str[2]);
  compass_msg.data.push_back(data->compass2_str[0]);
  compass_msg.data.push_back(data->compass2_str[1]);
  compass_msg.data.push_back(data->compass2_str[2]);
  compass_pub->publish(compass_msg);

  //-------------------------接受到的所有信息全部发布出去,用于记log
  cti_chassis_msgs::msg::VehicleState chassis_info_msg;
  chassis_info_msg.port_index = data->portIndex;
  chassis_info_msg.state_vehicle = data->state_vehicle;
  chassis_info_msg.drivers_enable = data->drivers_enable;
  chassis_info_msg.control_mode = data->control_mode;
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
  for (int i = 0; i < (sizeof(data->acc) / sizeof(data->acc[0])); i++) {
    chassis_info_msg.acc.push_back(data->acc[i]);
  }
  for (int i = 0; i < (sizeof(data->gyro) / sizeof(data->gyro[0])); i++) {
    chassis_info_msg.gyro.push_back(data->gyro[i]);
  }
  for (int i = 0; i < (sizeof(data->q) / sizeof(data->q[0])); i++) {
    chassis_info_msg.q.push_back(data->q[i]);
  }
  for (int i = 0; i < (sizeof(data->position) / sizeof(data->position[0]));
       i++) {
    chassis_info_msg.position.push_back(data->position[i]);
  }
  for (int i = 0;
       i < (sizeof(data->linear_velocity) / sizeof(data->linear_velocity[0]));
       i++) {
    chassis_info_msg.linear_velocity.push_back(data->linear_velocity[i]);
  }
  chassis_info_msg.baro_raw = data->baro_raw;
  chassis_info_msg.baro_height = data->baro_height;
  for (int i = 0;
       i < (sizeof(data->compass1_str) / sizeof(data->compass1_str[0])); i++) {
    chassis_info_msg.compass1_str.push_back(data->compass1_str[i]);
  }
  for (int i = 0;
       i < (sizeof(data->compass2_str) / sizeof(data->compass2_str[0])); i++) {
    chassis_info_msg.compass2_str.push_back(data->compass2_str[i]);
  }
  chassis_info_msg.state_flag = data->state_flag;
  for (int i = 0; i < (sizeof(data->laser_id) / sizeof(data->laser_id[0]));
       i++) {
    chassis_info_msg.laser_id.push_back(data->laser_id[i]);
  }
  for (int i = 0; i < (sizeof(data->laser_data) / sizeof(data->laser_data[0]));
       i++) {
    chassis_info_msg.laser_data.push_back(data->laser_data[i]);
  }
  // recv_chassis_info_pub->publish(chassis_info_msg);
}

//**************************************
//接收到电池状态后发布话题,3.0车辆的点车状态
void ChassisHanddle::pub_battery_3_0(
    const recv_battery_4_to_1_active_report_status_type_3_0_ *data) {
  //数据处理 to_do
  // std::cout << "recv battery v3_0" << std::endl;
  if (!data) {
    return;
  }
  //电池电量发布
  cti_msgs::msg::BatteryCellsState BatCellsState;
  BatCellsState.header.stamp = rclcpp::Clock().now();
  BatCellsState.soc_all = (double)data->Bat_1_Soc;
  BatCellsState.volt_all = data->Bat_1_Volt;
  BatCellsState.charger_volt = data->battery_charger_voltage;

  BatCellsState.board_temp.clear();
  BatCellsState.board_temp.push_back(data->Board_temp1);
  BatCellsState.board_temp.push_back(data->Board_temp2);
  BatCellsState.board_temp.push_back(data->Board_temp3);
  BatCellsState.board_temp.push_back(data->Board_temp4);
  BatCellsState.board_temp.push_back(data->Board_temp5);
  BatCellsState.board_temp.push_back(data->Board_temp6);
  BatCellsState.board_temp.push_back(data->Board_temp7);
  BatCellsState.board_temp.push_back(data->Board_temp8);
  BatCellsState.board_curr.clear();
  BatCellsState.board_curr.push_back(data->Board_curr1);
  BatCellsState.board_curr.push_back(data->Board_curr2);
  BatCellsState.board_curr.push_back(data->Board_curr3);
  BatCellsState.board_curr.push_back(data->Board_curr4);
  BatCellsState.board_curr.push_back(data->Board_curr5);
  BatCellsState.board_curr.push_back(data->Board_curr6);
  BatCellsState.board_curr.push_back(data->Board_curr7);
  BatCellsState.board_curr.push_back(data->Board_curr8);

  BatCellsState.charge_reverse = data->charge_reverse;
  BatCellsState.v_leakage = data->v_leakage;
  BatCellsState.charge_status = data->ChargeStatus;
  BatCellsState.lock_status = data->brake_status;
  BatCellsState.errorinfo = data->ErrorInfo;

  cti_msgs::msg::BatteryCell BatCell;
  BatCell.bat_soc = data->Bat_1_Soc;
  BatCell.bat_volt = data->Bat_1_Volt;
  BatCell.bat_curr = data->Bat_1_curr;
  BatCell.bat_temp = data->Bat_1_temp;
  BatCell.bat_comm_state = data->Bat_1_comm_state;
  BatCell.bat_cell_num = 1;
  BatCellsState.batcells.push_back(BatCell);

  BatCellsState.charge_type = 2;  // 0:undefined 1:wireless_charge
                                  // 2:wired_charge
  BatCellsState.charge_voltage = data->battery_charger_voltage;
  BatCellsState.charge_current = data->battery_charger_current;
  BatCellsState.charge_reserve = data->charge_reverse;
  BatCellsState.charge_state = data->ChargeStatus;
  BatCellsState.charge_temp = data->Bat_1_temp;
  // BatCellsState.charge_charger = data->battery_charger_state;
  BatCellsState.charge_charger = data->battery_charger_flag;

  batcell_pub->publish(BatCellsState);

  Info(" R_WC_VT: " << (int)data->Bat_1_Volt
                    << " R_WC_CU: " << (int)data->Bat_1_curr
                    << " R_WC_TP: " << (int)data->Bat_1_temp
                    << " R_BA_1_SOC: " << (int)data->Bat_1_Soc
                    << " R_BT_CG_VO: " << (int)data->battery_charger_voltage
                    << " R_BT_CG_CU: " << (int)data->battery_charger_current
                    << " R_BT_CG_CS: " << (int)data->ChargeStatus
                    << " R_BT_CG_FL: " << (int)data->battery_charger_flag
                    << " R_BT_CG_EI: " << (int)data->ErrorInfo);
  

  //浅休眠
  static long int cnt{0};
  static std_msgs::msg::Bool shallow_sleep_msg;

  //
  if(shallow_sleep_msg.data == true)
  {
    cnt++;
  }

  if((int)(data->ErrorInfo&(1<<16))!=0)
  {
    shallow_sleep_msg.data = true;
    shallow_sleep_pub->publish(shallow_sleep_msg);
  }
  else
  {
    shallow_sleep_msg.data = false;
    shallow_sleep_pub->publish(shallow_sleep_msg);
  }

}

//**************************************对比两个输入的vector,返回对比结果
bool ChassisHanddle::less_equa_compare(std::vector<int> vec1,
                                       std::vector<int> vec2) {
  if (vec1[0] < vec2[0]) {
    return true;
  }
  if ((vec1[0] == vec2[0]) && (vec1[1] < vec2[1])) {
    return true;
  }
  if ((vec1[0] == vec2[0]) && (vec1[1] == vec2[1]) && (vec1[2] <= vec2[2])) {
    return true;
  }
  return false;
}
//************************************** 提取版本号中的数字
//并和版本限制进行对比，如果超出版本限制，导航的运动控制命令不会下发
void ChassisHanddle::getNumInString(std::string str) {
  int str_len;
  int start_itr;
  std::vector<int> version;

  std::string num_str;

  str_len = str.length();
  for (int i = 0; i < str_len; i++) {
    if (str[i] == 'V') {
      start_itr = i + 1;
      break;
    }
  }
  for (int i = start_itr; i < str_len; i++) {
    std::string temp_str = "";
    temp_str = str[i];
    if (str[i] >= '0' && str[i] <= '9')
      num_str.push_back(str[i]);
    else if (str[i] == '.') {
      version.push_back(atoi(num_str.c_str()));
      num_str.clear();
      continue;
    } else
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

  bool less_equa_than_max_version = less_equa_compare(version, max_version);
  bool more_equa_than_min_version = less_equa_compare(min_version, version);

  if (less_equa_than_max_version && more_equa_than_min_version) {
    control_version_right = 0;
  }
  if (!less_equa_than_max_version && more_equa_than_min_version) {
    control_version_right = 1;
  }
  if (less_equa_than_max_version && !more_equa_than_min_version) {
    control_version_right = -1;
  }
  firmwareVersionCheck.data = control_version_right;
  firmware_version_status_pub->publish(firmwareVersionCheck);
}
//************************************** 接收到固件版本号后发布话题
void ChassisHanddle::pub_firmwareversion(
    recv_from_firmware_version_type *data) {
  if (!data) {
    return;
  }
  if (data->app_ver != NULL && data->app_ver != " ") {
    get_version_flag = true;
  }
  cti_chassis_msgs::msg::FirmWareInfo fm_info_msg;
  fm_info_msg.src = data->upd_info.src;
  fm_info_msg.dest = data->upd_info.dest;
  fm_info_msg.run_area = data->run_area;
  fm_info_msg.update_status = data->update_status;
  fm_info_msg.boot_ver = data->boot_ver;
  fm_info_msg.app_ver = data->app_ver;
  fm_info_msg.update_lib_ver = data->update_lib_ver;

  firmware_version_check_pub->publish(fm_info_msg);

  switch (data->upd_info.src) {
    case MODULE_DEBUG_BOARD:
      RobotVersionDisplay.debug_interface_version = data->app_ver;
      break;
    case MODULE_MOVE_CONTROL_BOARD: {
      RobotVersionDisplay.operation_control_version = data->app_ver;
      std::string version_info;
      version_info = data->app_ver;
      getNumInString(version_info);
      break;
    }
    case MODULE_POWER_INTEGRATE_BOARD:
      RobotVersionDisplay.battery_tandem_version = data->app_ver;
      break;
    case MODULE_MOTOR_DRIVER_FRONT_LEFT:
      RobotVersionDisplay.left_front_diver_version = data->app_ver;
      break;
    case MODULE_MOTOR_DRIVER_FRONT_RIGHT:
      RobotVersionDisplay.right_front_diver_version = data->app_ver;
      break;
    case MODULE_MOTOR_DRIVER_BACK_LEFT:
      RobotVersionDisplay.left_back_diver_version = data->app_ver;
      break;
    case MODULE_MOTOR_DRIVER_BACK_RIGHT:
      RobotVersionDisplay.right_back_diver_version = data->app_ver;
      break;
    case MODULE_MOTOR_TURN_FRONT:
      RobotVersionDisplay.forwards_way_diver_version = data->app_ver;
      break;
    case MODULE_MOTOR_TURN_BACK:
      RobotVersionDisplay.backwards_way_diver_version = data->app_ver;
      break;
    case MODULE_MOTOR_BRAKE_FRONT:
      RobotVersionDisplay.front_brake_diver_version = data->app_ver;
      break;
    case MODULE_MOTOR_BREAK_BACK:
      RobotVersionDisplay.back_brake_diver_version = data->app_ver;
      break;
    case MODULE_LIGHT_START:
      RobotVersionDisplay.light_control_version = data->app_ver;
      break;
    case MODULE_ULTRASONIC_1:
      RobotVersionDisplay.ultrasonicvers[0] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_2:
      RobotVersionDisplay.ultrasonicvers[1] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_3:
      RobotVersionDisplay.ultrasonicvers[2] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_4:
      RobotVersionDisplay.ultrasonicvers[3] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_5:
      RobotVersionDisplay.ultrasonicvers[4] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_6:
      RobotVersionDisplay.ultrasonicvers[5] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_7:
      RobotVersionDisplay.ultrasonicvers[6] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_8:
      RobotVersionDisplay.ultrasonicvers[7] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_9:
      RobotVersionDisplay.ultrasonicvers[8] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_10:
      RobotVersionDisplay.ultrasonicvers[9] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_11:
      RobotVersionDisplay.ultrasonicvers[10] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_12:
      RobotVersionDisplay.ultrasonicvers[11] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_13:
      RobotVersionDisplay.ultrasonicvers[12] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_14:
      RobotVersionDisplay.ultrasonicvers[13] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_15:
      RobotVersionDisplay.ultrasonicvers[14] = data->app_ver;
      break;
    default:
      break;
  }
  firmvion_pub->publish(RobotVersionDisplay);
}

//************************************** 接收到命令应答之后计数
void ChassisHanddle::compar_id_recv_send(recv_from_cmd_answer_type *data) {
  if (!data) {
    return;
  }
  cmd_send_map.clear();
  cmd_answer_success_cnt++;
}

//************************************** 接收到底盘错误码发布
void ChassisHanddle::pub_chassiserror(recv_chassis_error_report_type *data) {
  if (!data) {
    return;
  }
  std_msgs::msg::UInt32MultiArray chassis_error;
  chassis_error.data.push_back((uint32_t)data->module_type);
  chassis_error.data.push_back(data->module_error_code);
  chassis_error.data.push_back((uint32_t)data->module_error_level);
  chassis_error_pub->publish(chassis_error);

  module_type_global = data->module_type;
  module_error_code_global = data->module_error_code;
  module_error_level_global = data->module_error_level;
}
//*************************************Timer5定时器，发布地盘错误码
void ChassisHanddle::timer5Callback() {
  std_msgs::msg::UInt32MultiArray chassis_error;

  if (0 == module_type_global) {
    chassis_error.data.push_back(0);
  } else {
    chassis_error.data.push_back((uint32_t)module_type_global);
    module_type_global = 0;
  }

  if (0 == module_error_code_global) {
    chassis_error.data.push_back(0);
  } else {
    chassis_error.data.push_back(module_error_code_global);
    module_error_code_global = 0;
  }

  if (0 == module_error_level_global) {
    chassis_error.data.push_back(0);
  } else {
    chassis_error.data.push_back((uint32_t)module_error_level_global);
    module_error_level_global = 0;
  }
  chassis_error_pub->publish(chassis_error);
}
//************************************** 接收到底盘导航重要信息
void ChassisHanddle::pub_navigationlog(recv_navigation_log_status_type *data) {
  if (!data) {
    return;
  }
  navigation_log.header.stamp = rclcpp::Clock().now();
  navigation_log.header.frame_id = "/fpga_serial/navigationlog";
  navigation_log.liner_speed = data->liner_speed;
  navigation_log.turn_angle = data->turn_angle;
  navigation_log.break_torque =
      data->break_torque;  //刹车力度，直接向驱动下发力矩值
  navigation_log.enable_flag = data->enable_flag;
  navigation_log.actual_body_linear_vel_x = data->actual_body_linear_vel_x;
  navigation_log.actual_speed_base_on_left_front_wheel =
      data->actual_speed_base_on_left_front_wheel;  //左前轮折算的车体中心速度
  navigation_log.actual_speed_base_on_right_front_wheel =
      data->actual_speed_base_on_right_front_wheel;
  navigation_log.actual_speed_base_on_left_rear_wheel =
      data->actual_speed_base_on_left_rear_wheel;
  navigation_log.actual_speed_base_on_right_rear_wheel =
      data->actual_speed_base_on_right_rear_wheel;
  navigation_log.actual_turn_front_angle =
      data->actual_turn_front_angle;  // unit:degree 前后转向电机的角度
  navigation_log.actual_turn_rear_angle =
      data->actual_turn_rear_angle;  // unit:degree
  for (int i = 0; i < (sizeof(data->set_torque) / sizeof(int16_t)); i++) {
    navigation_log.set_torque.push_back(data->set_torque[i]);
  }
  for (int i = 0; i < (sizeof(data->now_encoder) / sizeof(uint16_t)); i++) {
    navigation_log.now_encoder.push_back(data->set_torque[i]);
  }
  navigation_log_pub->publish(navigation_log);
  navigation_log.set_torque.clear();
  navigation_log.now_encoder.clear();
}
//************************************** 接收sd卡格式化结果后发布话题
void ChassisHanddle::pub_formatsdcard(send_format_sd_card_cmd_type *data) {
  if (!data) {
    return;
  }
  formatsdcard_result.data = data->portIndex;
  formatsdcard_pub->publish(formatsdcard_result);
}
//************************************** 经纬度数据转化,度.分->度.度
double ChassisHanddle::gps_data_trans(double data) {
  double data_temp = data;
  ;
  if (data < 0) {
    data_temp = -data_temp;
  }
  int int_part = (int)data_temp;
  double dec_part = data_temp - int_part;
  dec_part = dec_part * 100 / 60.0;
  double ret = dec_part + int_part;
  if (data < 0) {
    ret = -ret;
    return ret;
  } else {
    return ret;
  }
}

//************************************** 接收sd卡格式化结果后发布话题
void ChassisHanddle::pub_gps(recv_gps_data_type *data) {
  if (!data) {
    return;
  }
  gps_data.header.stamp = rclcpp::Clock().now();
  gps_data.header.frame_id = "/chassis_gps";
  // gps utc time
  gps_data.utc_seconds =
      data->utc_seconds;  // utc时间,00:00:00至今的秒数,当前未使用

  // position
  switch (data->position_stat)  //位置解状态,NONE=无定位 SINGLE=单点定位
                                // PSFDIFF=伪距差分 NARROW_FLOAT=浮点解
                                // NARROW_INT=固定解
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
  gps_data.lat = (data->lat) / 100;  //负数表示南半球,单位(度)
  gps_data.lat = gps_data_trans(gps_data.lat);
  gps_data.lon = (data->lon) / 100;  //负数表示西半球,单位(度)
  gps_data.lon = gps_data_trans(gps_data.lon);
  gps_data.alt =
      data->alt;  // WGS84 椭球高,若使用海拔高,则需根据undulation计算转换.
  gps_data.lat_err = data->lat_err;            //纬度标准差,单位(m)
  gps_data.lon_err = data->lon_err;            // 经度标准差,单位(m)
  gps_data.alt_err = data->alt_err;            //高程标准差,单位(m)
  gps_data.diff_age = data->diff_age;          // 差分龄,单位(s)
  gps_data.undulation = data->undulation;      // 海拔高 = alt - undulation
  gps_data.sats_tracked = data->sats_tracked;  // 跟踪到的卫星数
  gps_data.sats_used = data->sats_used;        // 解算中使用的卫星数

  // angle 主天线(moving)到副天线(heading)所形成的向量,真北即经线北向
  // position
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
  gps_data.heading =
      data->heading;  // 航向角(度),以为真北为起点,顺时针 0.0 - 359.99
  gps_data.pitch = data->pitch;  // 俯仰角(度),水平为0°,±90°
  gps_data.heading_err = data->heading_err;  //航向角标准差,单位(度)
  gps_data.pitch_err = data->pitch_err;      //俯仰角标准差,单位(度)
  gps_data.baselinelen = data->baselineLen;  // 两天线基线长

  // velocity
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
  gps_data.speed_north =
      (data->speed_north) / 100;  // 东北天系下各速度分量,单位(m/s)
  gps_data.speed_east = (data->speed_east) / 100;
  gps_data.speed_up = (data->speed_up) / 100;
  gps_data.latency = data->latency;  // 速度延迟,单位(s)

  gps_pub->publish(gps_data);
}

void ChassisHanddle::pub_rtk(msg_rtk_gps_data_1_0 *data) {
  if (!data) {
    return;
  }
  rtk_data.header.stamp = rclcpp::Clock().now();
  rtk_data.header.frame_id = "/chassis_rtk";
  // gps utc time
  rtk_data.utc_seconds =
      data->utc_seconds;  // utc时间,00:00:00至今的秒数,当前未使用

  // position
  switch (data->position_stat)  //位置解状态,NONE=无定位 SINGLE=单点定位
                                // PSFDIFF=伪距差分 NARROW_FLOAT=浮点解
                                // NARROW_INT=固定解
  {
    case 0:
    case 48:
      rtk_data.position_stat = "NONE";
      break;
    case 49:
      rtk_data.position_stat = "SINGLE";
      break;
    case 50:
      rtk_data.position_stat = "PSFDIFF";
      break;
    case 51:
      rtk_data.position_stat = "NONE";
      break;
    case 52:
      rtk_data.position_stat = "NONE";
      break;
    case 53:
      rtk_data.position_stat = "NARROW_FLOAT";
      break;
    case 54:
      rtk_data.position_stat = "NARROW_INT";
      break;
    default:
      rtk_data.position_stat = "UNDEFINED";
      break;
  }
  rtk_data.lat = (data->lat) / 100;  //负数表示南半球,单位(度)
  rtk_data.lat = gps_data_trans(rtk_data.lat);
  rtk_data.lon = (data->lon) / 100;  //负数表示西半球,单位(度)
  rtk_data.lon = gps_data_trans(rtk_data.lon);
  rtk_data.alt =
      data->alt;  // WGS84 椭球高,若使用海拔高,则需根据undulation计算转换.
  rtk_data.lat_err = data->lat_err;            //纬度标准差,单位(m)
  rtk_data.lon_err = data->lon_err;            // 经度标准差,单位(m)
  rtk_data.alt_err = data->alt_err;            //高程标准差,单位(m)
  rtk_data.diff_age = data->diff_age;          // 差分龄,单位(s)
  rtk_data.undulation = data->undulation;      // 海拔高 = alt - undulation
  rtk_data.sats_tracked = data->sats_tracked;  // 跟踪到的卫星数
  rtk_data.sats_used = data->sats_used;        // 解算中使用的卫星数

  // angle 主天线(moving)到副天线(heading)所形成的向量,真北即经线北向
  // position
  switch (data->heading_stat)  // 定向解状态,仅 SOL_COMPUTED 时表示定位成功
  {
    case 0:
      rtk_data.heading_stat = "NONE";
      break;
    case 1:
      rtk_data.heading_stat = "SOL_COMPUTED";
      break;
    default:
      rtk_data.heading_stat = "UNDEFINED";
      break;
  }
  rtk_data.heading =
      data->heading;  // 航向角(度),以为真北为起点,顺时针 0.0 - 359.99
  rtk_data.pitch = data->pitch;  // 俯仰角(度),水平为0°,±90°
  rtk_data.heading_err = data->heading_err;  //航向角标准差,单位(度)
  rtk_data.pitch_err = data->pitch_err;      //俯仰角标准差,单位(度)
  rtk_data.baselinelen = data->baselineLen;  // 两天线基线长

  // velocity
  switch (data->velocity_stat)  // 速度解状态,仅 SOL_COMPUTED 时表示定位成功
  {
    case 0:
      rtk_data.velocity_stat = "NONE";
      break;
    case 1:
      rtk_data.velocity_stat = "SOL_COMPUTED";
      break;
    default:
      rtk_data.velocity_stat = "UNDEFINED";
      break;
  }
  rtk_data.speed_north =
      (data->speed_north) / 100;  // 东北天系下各速度分量,单位(m/s)
  rtk_data.speed_east = (data->speed_east) / 100;
  rtk_data.speed_up = (data->speed_up) / 100;
  rtk_data.latency = data->latency;  // 速度延迟,单位(s)

  rtk_pub->publish(rtk_data);
}
//************************************** 定位状态命令话题回调函数
void ChassisHanddle::localizerState_Callback(
    const cti_msgs::msg::RobotLocalizerState::SharedPtr msg) {
  if (localization_limit) {
    robotlocalizerstate_global = msg->id;
  } else {
    robotlocalizerstate_global = 2;
  }
}
//************************************** 发送细时间同步命令
void ChassisHanddle::send_fine_sync_cmd() {
  //获取到微秒级别
  struct timeval tv;
  struct timezone tz;
  gettimeofday(&tv, &tz);

  time_sync_fine_msg_type timenow_cmd;

  timenow_cmd.sec = tv.tv_sec % 60;
  timenow_cmd.millisencond = (tv.tv_usec) / 1000;
  timenow_cmd.microsecondsec = (tv.tv_usec) % 1000;

  chassisCmncBase->queueSendDatas(TIME_SYNC_FINE_CMD, sizeof(timenow_cmd),
                                  &timenow_cmd, true);
}
//************************************** 发送延时时间同步命令
void ChassisHanddle::send_delay_sync_cmd() {
  //获取到微秒级别
  struct timeval tv;
  struct timezone tz;
  gettimeofday(&tv, &tz);

  time_sync_fine_msg_type timenow_cmd;

  timenow_cmd.sec = tv.tv_sec % 60;
  timenow_cmd.millisencond = (tv.tv_usec) / 1000;
  timenow_cmd.microsecondsec = (tv.tv_usec) % 1000;

  chassisCmncBase->queueSendDatas(TIME_SYNC_DELAY_CMD, sizeof(timenow_cmd),
                                  &timenow_cmd, true);
}
//************************************** 粗时间同步回应处理函数
void ChassisHanddle::check_rough_res(time_sync_rough_msg_type *data) {
  if (data->year == sync_rough_msg_saved.year &&
      data->mon == sync_rough_msg_saved.mon &&
      data->day == sync_rough_msg_saved.day &&
      data->hour == sync_rough_msg_saved.hour &&
      data->min == sync_rough_msg_saved.min &&
      data->sec == sync_rough_msg_saved.sec) {
    //发送细时间同步
    need_send_fine_sync = true;
    //取消粗同步回应超时检测
    check_rough_sync_res_timeoout = false;
    check_rough_sync_res_cnt = 0;
    need_send_rough = false;
    // printf("rough_sync_successed@@@@@@@@@@@@@@@\n");
  } else {
    need_resend_rough = true;
  }
}

//************************************** 环卫车清扫状态发布_新的,加边刷伸缩
void ChassisHanddle::pub_clean_state_new(clean_to_motion_t_new *data) {
  //使用cti_msgs/DataArray消息类型发布消息
  cti_msgs::msg::DataArray dustVehicleInfos;
  dustVehicleInfos.header.stamp = rclcpp::Clock().now();
  dustVehicleInfos.header.frame_id = "sanitation_vehicle";
  //水箱高水位
  cti_msgs::msg::Data dustVehicleInfo;
  dustVehicleInfo.name = "water_tank_top";
  dustVehicleInfo.data = std::to_string(data->water_tank_top);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //水箱低水位
  dustVehicleInfo.name = "water_tank_bottom";
  dustVehicleInfo.data = std::to_string(data->water_tank_bottom);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //电机状态
  dustVehicleInfo.name = "motor_status";
  dustVehicleInfo.data = std::to_string(data->motor_status.data);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //挡板状态
  dustVehicleInfo.name = "dam_board_status";
  dustVehicleInfo.data = std::to_string(data->dam_board_status);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //边刷伸展
  dustVehicleInfo.name = "sidebrush_state";
  dustVehicleInfo.data = std::to_string(data->side_brush_transform_state);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //边刷旋转速度
  dustVehicleInfo.name = "sidebrush_speed";
  dustVehicleInfo.data = std::to_string(data->side_brush_speed);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //清扫功能错误码
  dustVehicleInfo.name = "error_code";
  dustVehicleInfo.data = std::to_string(data->error_code);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT32;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //边刷伸展错误码
  dustVehicleInfo.name = "sidebrush_transform_error";
  dustVehicleInfo.data = std::to_string(data->side_brush_transform_error);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //左边刷错误码
  dustVehicleInfo.name = "left_sidebrush_error";
  dustVehicleInfo.data = std::to_string(data->left_side_brush_error);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //右边刷错误码
  dustVehicleInfo.name = "right_sidebrush_error";
  dustVehicleInfo.data = std::to_string(data->right_side_brush_error);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //加水水泵状态
  dustVehicleInfo.name = "increase_water_pump";
  uint8_t increase_water_pump_state =
      data->multi_status.bits.increase_water_pump;
  dustVehicleInfo.data = std::to_string(increase_water_pump_state);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //震尘速度
  dustVehicleInfo.name = "shake_dust_motor_speed_feedback";
  dustVehicleInfo.data = std::to_string(data->shake_dust_motor_speed_feedback);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //滚刷速度
  dustVehicleInfo.name = "roll_brush_speed";
  dustVehicleInfo.data = std::to_string(data->roll_brush_speed);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //滚刷错误码
  dustVehicleInfo.name = "roll_brush_error_code";
  dustVehicleInfo.data = std::to_string(data->roll_brush_error_code);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //边刷上升状态
  dustVehicleInfo.name = "lift_motorr_state";
  dustVehicleInfo.data = std::to_string(data->lift_motorr_state);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //喷水状态
  dustVehicleInfo.name = "spray_state";
  dustVehicleInfo.data = std::to_string(data->spray_state);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //控制模式反馈状态
  dustVehicleInfo.name = "control_mode_state";
  dustVehicleInfo.data = std::to_string(data->control_mode_state);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //
  dustVehicleInfo.name = "push_board_state";
  std::string data_te;
  for (int i = 9; i > 0; i--) {
    data_te = data_te + std::to_string(data->push_board_state[i]) + ",";
  }
  dustVehicleInfo.data = data_te;
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //
  dustVehicleInfo.name = "push_board_error_code";
  std::string data_te2;
  for (int ii = 9; ii > 0; ii--) {
    data_te2 = data_te2 + std::to_string(data->push_board_error_code[ii]) + ",";
  }
  dustVehicleInfo.data = data_te2;
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //存储上水位传感器数据
  vehicle_water_tank_top.push_back(data->water_tank_top);
  vehicle_water_tank_bottom.push_back(data->water_tank_bottom);
  if (vehicle_water_tank_top.size() > max_vehicle_water_tank_restore) {
    vehicle_water_tank_top.pop_front();
  }
  if (vehicle_water_tank_bottom.size() > max_vehicle_water_tank_restore) {
    vehicle_water_tank_bottom.pop_front();
  }
  bool water_full = true;   //默认水满
  bool water_empty = true;  //默认水空
  for (int i = 0; i < max_vehicle_water_tank_restore - 1; i++) {
    if (1 == vehicle_water_tank_top[i]) {
      water_full = false;  //队列中有一个 == 1,就认为水没有满
      break;
    }
  }
  for (int i = 0; i < max_vehicle_water_tank_restore - 1; i++) {
    if (1 == vehicle_water_tank_bottom[i]) {
      water_empty = false;  //队列中有一个 == 1,就认为水没有空
      break;
    }
  }
  //计算水量百分比
  //排水状态
  if (31 == data->motor_status.data) {
    if (false == vehicle_water_status.is_out) {
      vehicle_water_status.is_out = true;
      vehicle_water_status.out_pre_time = rclcpp::Clock().now().seconds();
    }
  } else if (27 == data->motor_status.data || 0 == data->motor_status.data) {
    if (true == vehicle_water_status.is_out) {
      vehicle_water_status.is_out = false;
      vehicle_water_status.out_pre_time = rclcpp::Clock().now().seconds();
    }
  }
  //进水状态
  if (1 == increase_water_pump_state) {
    if (false == vehicle_water_status.is_in) {
      vehicle_water_status.is_in = true;
      vehicle_water_status.in_pre_time = rclcpp::Clock().now().seconds();
    }
  } else if (0 == increase_water_pump_state) {
    if (true == vehicle_water_status.is_in) {
      vehicle_water_status.is_out = false;
      vehicle_water_status.out_pre_time = rclcpp::Clock().now().seconds();
    }
  }

  if (water_full) {
    vehicle_water_status.water_percnt_now = 100.0;
  } else if (water_empty) {
    vehicle_water_status.water_percnt_now = 0.0;
  } else {
    //排水计算
    if (true == vehicle_water_status.is_out) {
      double time_now = rclcpp::Clock().now().seconds();
      double duration = time_now - vehicle_water_status.out_pre_time;
      vehicle_water_status.water_percnt_now =
          vehicle_water_status.water_percnt_now -
          duration * vehicle_water_status.out_per_sec;
      if (vehicle_water_status.water_percnt_now > 90.0) {
        vehicle_water_status.water_percnt_now = 90.0;
      }
      if (vehicle_water_status.water_percnt_now < 10.0) {
        vehicle_water_status.water_percnt_now = 10.0;
      }
      vehicle_water_status.out_pre_time = time_now;
    }
    //加水计算
    if (true == vehicle_water_status.is_in) {
      double time_now = rclcpp::Clock().now().seconds();
      double duration = time_now - vehicle_water_status.in_pre_time;
      vehicle_water_status.water_percnt_now =
          vehicle_water_status.water_percnt_now +
          duration * vehicle_water_status.in_per_sec;
      if (vehicle_water_status.water_percnt_now > 90.0) {
        vehicle_water_status.water_percnt_now = 90.0;
      }
      if (vehicle_water_status.water_percnt_now < 10.0) {
        vehicle_water_status.water_percnt_now = 10.0;
      }
      vehicle_water_status.out_pre_time = time_now;
    }
  }

  //水量百分比
  dustVehicleInfo.name = "water_percent";
  dustVehicleInfo.data = std::to_string(data->water_tank_top);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  std::string modify_file =
      "echo " + std::to_string(vehicle_water_status.water_percnt_now) + " > " +
      config_file_path;
  FILE *fpr = NULL;
  fpr = popen(modify_file.c_str(), "w");
  pclose(fpr);

  //消息发布
  dust_vehicle_state_info_pub->publish(dustVehicleInfos);

  Info("R_CC_ES: " << (int)data->motor_status.data
                   << " R_CC_WT: " << (int)data->water_tank_top
                   << " R_CC_WB: " << (int)data->water_tank_bottom
                   << " R_CC_DS: " << (int)data->dustbin_distance
                   << " R_CC_DOC: " << (int)data->recv_dustbin_on_the_car
                   << " R_CC_DB: " << (int)data->dam_board_status
                   << " R_CC_SBT: " << (int)data->side_brush_transform_state
                   << " R_CC_SBS: " << (int)data->side_brush_speed
                   << " R_CC_ERR: " << (int)data->error_code
                   << " R_CC_SBTE: " << (int)data->side_brush_transform_error
                   << " R_CC_LSBE: " << (int)data->left_side_brush_error
                   << " R_CC_RSBE: " << (int)data->right_side_brush_error
                   << " R_CC_WP: " << (int)vehicle_water_status.water_percnt_now
                   << " R_CC_IN_PU: " << (int)increase_water_pump_state);
  recv_clean_mechine_motor_status = data->motor_status.data;

  //判断清扫车在车上的状态判断
  if (set_dustbin_on_car) {
    if ((data->recv_dustbin_on_the_car == 1 && box_type == 4) ||
        (data->recv_dustbin_on_the_car == 0 && box_type != 4)) {
      set_dustbin_on_car = false;
    }
  }
}
//**************************************

//**************************************
//集尘箱状态发布(新的,加了电池,风机力矩等)
void ChassisHanddle::pub_dust_box_state_new(dust_box_to_motion_t_new *data) {
  //使用cti_msgs/DataArray消息类型发布消息
  cti_msgs::msg::DataArray dustBoxInfos;
  dustBoxInfos.header.stamp = rclcpp::Clock().now();
  dustBoxInfos.header.frame_id = "sanitation_dust_box";
  cti_msgs::msg::Data dustboxinfo;

  //箱子超声波数据
  dustboxinfo.name = "ultrasonic_distance0";
  dustboxinfo.data = std::to_string(data->ultrasonic_distance[0]);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //红外数据
  dustboxinfo.name = "infrared_data";
  dustboxinfo.data = std::to_string(data->infrared_data);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //防夹数据
  dustboxinfo.name = "anti_pinch_data";
  dustboxinfo.data = std::to_string(data->anti_pinch_data);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //拉绳距离
  dustboxinfo.name = "stay_cord_distance";
  dustboxinfo.data = std::to_string(data->stay_cord_distance);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_FLOAT;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //垃圾到顶盖的距离
  dustboxinfo.name = "dust_distance";
  dustboxinfo.data = std::to_string(data->dustbin_distance);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT16;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池1的电压
  dustboxinfo.name = "bat1_vol";
  dustboxinfo.data = std::to_string(data->voltage1);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_FLOAT;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池1的电量
  dustboxinfo.name = "bat1_soc";
  dustboxinfo.data = std::to_string(data->Bat_1_Soc);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池2的电压
  dustboxinfo.name = "bat2_vol";
  dustboxinfo.data = std::to_string(data->voltage2);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_FLOAT;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池2的电量
  dustboxinfo.name = "bat2_soc";
  dustboxinfo.data = std::to_string(data->Bat_2_Soc);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池3的电压
  dustboxinfo.name = "bat3_vol";
  dustboxinfo.data = std::to_string(data->voltage3);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_FLOAT;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池3的电量
  dustboxinfo.name = "bat3_soc";
  dustboxinfo.data = std::to_string(data->Bat_3_Soc);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池4的电压
  dustboxinfo.name = "bat4_vol";
  dustboxinfo.data = std::to_string(data->voltage4);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_FLOAT;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池4的电量
  dustboxinfo.name = "bat4_soc";
  dustboxinfo.data = std::to_string(data->Bat_4_Soc);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //箱子整体电量
  dustboxinfo.name = "dustbox_soc";
  uint8_t dustbox_soc = 0;
  if (data->voltage4 < 0) {
    //箱子最多装3个电池
    dustbox_soc = (data->Bat_1_Soc + data->Bat_2_Soc + data->Bat_3_Soc) / 3;
  } else {
    //箱子最多装4个电池
    dustbox_soc = (data->Bat_1_Soc + data->Bat_2_Soc + data->Bat_3_Soc +
                   data->Bat_4_Soc) /
                  4;
  }
  if (dustbox_soc > 100) dustbox_soc = 100;
  dustboxinfo.data = std::to_string(dustbox_soc);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //吸尘箱启动状态
  dustboxinfo.name = "motor_status";
  dustboxinfo.data = std::to_string(data->motor_status.data);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //自动倾倒垃圾的状态
  dustboxinfo.name = "dust_autopush";
  dustboxinfo.data = std::to_string(data->auto_push);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //风机速度,控制反馈
  dustboxinfo.name = "fan_Speed";
  dustboxinfo.data = std::to_string(data->fan_speed);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //风机转矩
  dustboxinfo.name = "fan_torque";
  dustboxinfo.data = std::to_string(data->fan_torque);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //风机转速,硬件反馈
  dustboxinfo.name = "fan_speed_hd";
  dustboxinfo.data = std::to_string(data->fan_speed_closedloop);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT16;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //风机电流
  dustboxinfo.name = "fan_current";
  dustboxinfo.data = std::to_string(data->fan_current);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //板子错误码
  dustboxinfo.name = "error_code";
  dustboxinfo.data = std::to_string(data->errorInfo);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT64;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //箱子4g状态
  dustboxinfo.name = "4g_state";
  dustboxinfo.data = std::to_string(data->net_state);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //箱子4g信号
  dustboxinfo.name = "4g_signal";
  dustboxinfo.data = std::to_string(data->net_signal);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //垃圾量百分比
  dustboxinfo.name = "trash_percent";
  dustboxinfo.data = std::to_string(data->trash_per);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //自动工作模式 0: 无效,无此功能 1：关 2：开
  dustboxinfo.name = "auto_work";
  dustboxinfo.data = std::to_string(data->auto_work);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //手动工作模式 0: 无效,无此功能 1：关 2：开
  dustboxinfo.name = "mannual_work";
  dustboxinfo.data = std::to_string(data->mannual_work);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //手动工作模式 0: 无效,无此功能 1：关 2：开
  dustboxinfo.name = "ultrasonic_clean";
  dustboxinfo.data = std::to_string(data->ultrasonic_clean);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);


  //消息发布
  dust_box_state_info_pub->publish(dustBoxInfos);

  //自动倒垃圾状态
  std_msgs::msg::UInt8 dust_box_autopush_msg;
  dust_box_autopush_msg.data = data->auto_push;
  dust_box_autopush_pub->publish(dust_box_autopush_msg);

  Info("R_DBO_ES: " << (int)data->motor_status.data
                    << " R_DBO_AW: " << (int)data->auto_work
                    << " R_DBO_MW: " << (int)data->mannual_work
                    << " R_DBO_DS: " << (int)data->dustbin_distance
                    << " R_DBO_DOC: " << (int)data->recv_dustbin_on_the_car
                    << " R_DBO_AP: " << (int)data->auto_push
                    << " R_DBO_FS: " << (int)data->fan_speed
                    << " R_DBO_WC_VT: " << data->wireless_voltage
                    << " R_DBO_WC_CU: " << data->wireless_current
                    << " R_DBO_WC_RE: " << (int)data->wireless_reserve
                    << " R_DBO_WC_ST: " << (int)data->wireless_state
                    << " R_DBO_WC_TP: " << (int)data->wireless_temp
                    << " R_DBO_WC_CH: " << (int)data->ultrasonic_clean
                    << " R_DBO_BAT1: " << data->voltage1 << " R_DBO_SOC1: "
                    << (int)data->Bat_1_Soc << " R_DBO_BAT2: " << data->voltage2
                    << " R_DBO_SOC2: " << (int)data->Bat_2_Soc
                    << " R_DBO_BAT3: " << data->voltage3 << " R_DBO_SOC3: "
                    << (int)data->Bat_3_Soc << " R_DBO_BAT4: " << data->voltage4
                    << " R_DBO_SOC4: " << (int)data->Bat_4_Soc
                    << " R_DBO_FTQ: " << (int)data->fan_torque
                    << " R_DBO_FSCL: " << (int)data->fan_speed_closedloop
                    << " R_DBO_FCU: " << (int)data->fan_current
                    << " R_DBO_ERR: " << (int)data->errorInfo
                    << " R_DBO_TRA_PER: " << (int)data->trash_per
                    << " R_DBO_4G_ST: " << (int)data->net_state
                    << " R_DBO_4G_SI: " << (int)data->net_signal
                    << " R_DBO_PW_SOC: " << (int)dustbox_soc
                    << " R_DBO_ULT_0: " << (int)data->ultrasonic_distance[0]);
  //-------------------通讯检测------------------
  box_chat_state.module_id = 2;
  box_chat_state.recv_state = true;
  box_chat_state.time_recv = rclcpp::Clock().now().seconds();
}

//**************************************
//雨水传感器数据发布--新的结构体<嵌入式有更新的有没有更新的，所以新结构体和旧结构体都要保留>
void ChassisHanddle::pub_rain_sensor_new(rain_sensor_t_new *data) {
  cti_msgs::msg::BoxState rain_sensor_msgs;
  rain_sensor_msgs.header.stamp = rclcpp::Clock().now();
  cti_msgs::msg::TabState rain_sensor_msg;
  rain_sensor_msg.status = data->flag.bits.switch_door;
  rain_sensor_msg.name = "switch_door";
  rain_sensor_msgs.states.push_back(rain_sensor_msg);
  rain_sensor_msg.status = data->flag.bits.tool_door;
  rain_sensor_msg.name = "tool_door";
  rain_sensor_msgs.states.push_back(rain_sensor_msg);
  rain_sensor_msg.status = data->right;
  rain_sensor_msg.name = "right";
  rain_sensor_msgs.states.push_back(rain_sensor_msg);
  rain_sensor_msg.status = data->left;
  rain_sensor_msg.name = "left";
  rain_sensor_msgs.states.push_back(rain_sensor_msg);
  rain_sensor_pub->publish(rain_sensor_msgs);
  Info("R_RS_R: " << (int)data->right << " R_RS_L: " << (int)data->left
                  << " R_RS_F: " << (int)data->flag.data);
}

//************************************** 升级命令接收处理
int ChassisHanddle::process_update_cmd_ex(unsigned char cmd,
                                          unsigned char *data,
                                          unsigned int length) {
  if ((NULL == data) || (length > 510)) {
    return -1;
  }
  cti_chassis_msgs::msg::UpdateInfo upd_data;
  seq_num++;
  upd_data.seq_num = seq_num;
  for (int i = 0; i < length ; i++) {
    upd_data.data.push_back(*(data + i));
  }
  stm32_pub->publish(upd_data);
  return 0;
}

//************************************** 主定时器
void ChassisHanddle::mainTimerCallback() {
  if (this->device_set_flag) {
    getChassisData();
    sendChassisData();
  }
}

//************************************** 定时器 代码更新超时处理
void ChassisHanddle::timerCallback() {
  //--代码更新超时处理--
  if (stm32_update_flag && (++TIMEOUT) >= 20) {
    TIMEOUT = 0;
    stm32_update_flag = false;
  }
}

//************************************** 定时器2 查询版本号和发送时间同步
void ChassisHanddle::timer2Callback() {
  //--查询版本号--
  if (get_version_flag == false) {
    send_to_check_version_type check_version_cmd;
    update_info_type update_info_struct;
    update_info_struct.src = MODULE_CHECK_UPD;
    update_info_struct.dest = MODULE_MOVE_CONTROL_BOARD;
    check_version_cmd.upd_info = update_info_struct;
    check_version_cmd.check = 01;
    chassisCmncBase->queueSendDatas(SEND_TO_CHECK_PROGRAM_VERSION,
                                    sizeof(check_version_cmd),
                                    &check_version_cmd, false);
  }

  if ((sendtime_loop_num >= 60 || need_resend_rough) && need_send_rough) {
    //获取年月日时分秒
    struct tm *local;
    time_t t;
    t = time(NULL);
    local = localtime(&t);
    //获取到微秒级别
    struct timeval tv;
    struct timezone tz;
    gettimeofday(&tv, &tz);

    time_sync_rough_msg_type timenow_cmd;

    timenow_cmd.year = local->tm_year;
    timenow_cmd.mon = local->tm_mon;
    timenow_cmd.day = local->tm_mday;
    timenow_cmd.hour = local->tm_hour;
    timenow_cmd.min = local->tm_min;
    timenow_cmd.sec = local->tm_sec;

    chassisCmncBase->queueSendDatas(TIME_SYNC_ROUGH_CMD, sizeof(timenow_cmd),
                                    &timenow_cmd, true);

    sendtime_loop_num = 0;
    need_resend_rough = false;
    sync_rough_msg_saved = timenow_cmd;
    check_rough_sync_res_timeoout = true;
    check_rough_sync_res_cnt = 0;
  }
  sendtime_loop_num++;

  if (check_rough_sync_res_timeoout) {
    if (check_rough_sync_res_cnt >= 3) {
      ////Info(" recv_rough_sync_res_timeout: "<<1);
      need_resend_rough = true;
      check_rough_sync_res_timeoout = false;
      check_rough_sync_res_cnt = 0;
    }
    check_rough_sync_res_cnt++;
  }

  if (need_send_fine_sync) {
    if (send_fine_sync_cnt >= 60) {
      send_fine_sync_cnt = 0;
      send_fine_sync_cmd();
    }
    send_fine_sync_cnt++;
  }

  cmd_answer_cnt.cmd_answer_success_num = cmd_answer_success_cnt;
  cmd_answer_cnt.cmd_answer_fail_num =
      cmd_withid_send_cnt - cmd_answer_success_cnt;
  cmd_answer_cnt.cmd_withid_send_num = cmd_withid_send_cnt;
  cmd_answer_pub->publish(cmd_answer_cnt);
}

double ChassisHanddle::get_duration(double in_time) {
  return rclcpp::Clock().now().seconds() - in_time;
}

void ChassisHanddle::timer_chat_timeout_Callbak() {
  static uint8_t in_cnt = 0;
  //开机10s后才开始检测
  if (in_cnt < 10) {
    in_cnt++;
    return;
  }

  cti_msgs::msg::BoxState communicate_msg;
  communicate_msg.header.stamp = rclcpp::Clock().now();
  cti_msgs::msg::TabState chassis_state;
  chassis_state.id = 0;
  cti_msgs::msg::TabState dustbox_state;
  dustbox_state.id = 1;
  //含义定义:
  // id: 模块 0:底盘通讯 1:箱子通讯
  // status: 0:空闲 1:通讯正常 -1:通讯超时
  // message: free:空闲; ok:通讯正常; timeout:通讯超时;

  //底盘通讯检测
  if (get_duration(chassis_chat_state.time_recv) > chassis_chat_timeout_secs) {
    chassis_chat_state.recv_state = false;
  }
  if (!chassis_chat_state.recv_state) {
    //底盘通讯异常
    chassis_state.status = -1;
    chassis_state.message = "timeout";
  } else {
    //底盘通讯正常
    chassis_state.status = 1;
    chassis_state.message = "ok";
  }
  //箱子通讯检测
  if (get_duration(box_chat_state.time_recv) > box_chat_timeout_secs) {
    box_chat_state.recv_state = false;
  }
  if (dustbox_lora_id != -1) {
    if (!box_chat_state.recv_state) {
      //箱子通讯异常
      dustbox_state.status = -1;
      dustbox_state.message = "timeout";
    } else {
      //箱子通讯正常
      dustbox_state.status = 1;
      dustbox_state.message = "ok";
    }
  } else {
    // lora_id没有设置
    //发布通讯空闲状态
    box_chat_state.reset();
    dustbox_state.status = 0;
    dustbox_state.message = "free";
  }
  communicate_msg.states.push_back(chassis_state);
  Info("VEH_CHAT: " << (int)chassis_state.status);
  communicate_msg.states.push_back(dustbox_state);
  Info("DBX_ID: " << (int)box_chat_state.module_id);
  Info("DBX_CHAT: " << (int)dustbox_state.status);
  chat_statue_pub->publish(communicate_msg);
}

void ChassisHanddle::timer_set_process_work_mode_Callback() {
  if (process_twork_mode_t.cur_cnt > 0) {
    process_twork_mode_t.cur_cnt--;
    process_twork_mode_t.process_work_mode_enum_ =
        process_work_mode::process_work_mode_test;
  } else {
    process_twork_mode_t.process_work_mode_enum_ =
        process_work_mode::process_work_mode_normal;
  }
}

//************************************** 非升级命令接收处理
int ChassisHanddle::process_nomal_cmd(unsigned char cmd, unsigned int length,
                                      unsigned char *data) {
  if (data == NULL) {
    return 0;
  }

  switch (cmd) {
    case RECV_FROM_RTK:
      msg_rtk_gps_data_1_0 *recv_rtk;
      if (sizeof(msg_rtk_gps_data_1_0) != length) {
        std::cout << "recv gps_data length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(msg_rtk_gps_data_1_0) << std::endl;
        return -1;
      }
      recv_rtk = (msg_rtk_gps_data_1_0 *)data;
      pub_rtk(recv_rtk);
      break;
    case RECV_FROM_DUST_BOX_INFO:
      dust_box_to_motion_t_new *recv_dust_box_state_new;
      if (sizeof(dust_box_to_motion_t_new) == length) {
        recv_dust_box_state_new = (dust_box_to_motion_t_new *)data;
        pub_dust_box_state_new(recv_dust_box_state_new);
      } else {
        std::cout << "recv dust_box_to_motion_t length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(dustbin_rf_set_cmd_t)
                  << " or new: " << sizeof(dust_box_to_motion_t_new)
                  << std::endl;
        return -1;
      }
      break;
    case ULT_POS_SEND_CMD:
      msg_upa_pos_data_t *msg_upa_pos_data;
      if (sizeof(msg_upa_pos_data_t) != length) {
        std::cout << "recv msg_upa_pos_data_t length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(msg_upa_pos_data_t) << std::endl;
        return -1;
      }
      msg_upa_pos_data = (msg_upa_pos_data_t *)data;
      pub_alt_3_0_(msg_upa_pos_data);
      break;

    case RECV_FROM_CONTROL_STATUS:
      recv_from_control_status_type *recv_statusdata;
      if (sizeof(recv_from_control_status_type) != length) {
        std::cout << "recv control length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(recv_from_control_status_type) << std::endl;
        return -1;
      }
      recv_statusdata = (recv_from_control_status_type *)data;
      pub_odom(recv_statusdata);
      break;

    case RECV_FROM_CONTROL_BATTERY_4_TO_1_STATUS:
      recv_battery_4_to_1_active_report_status_type_3_0_
          *recv_battery_4_to_1_active_report_status_type_3_0;

      if (sizeof(recv_battery_4_to_1_active_report_status_type_3_0_) !=
          length) {
        std::cout << "recv battery length: " << length << " sizeof struct_3.0: "
                  << sizeof(recv_battery_4_to_1_active_report_status_type_3_0_)
                  << std::endl;
        return -1;
      } else {
        recv_battery_4_to_1_active_report_status_type_3_0 =
            (recv_battery_4_to_1_active_report_status_type_3_0_ *)data;
        pub_battery_3_0(recv_battery_4_to_1_active_report_status_type_3_0);
      }
      break;

    case RECV_FROM_FIRMWARE_VERSION:
      Info("R_CMD_210: " << 1);
      recv_from_firmware_version_type *recv_version;
      if (sizeof(recv_from_firmware_version_type) != length) {
        std::cout << "recv version length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(recv_from_firmware_version_type) << std::endl;
        return -1;
      } else {
        recv_version = (recv_from_firmware_version_type *)data;
        pub_firmwareversion(recv_version);
      }
      break;

    case RECV_FROM_CMD_ANSWER:
      recv_from_cmd_answer_type *recv_cmdanswer;
      if (sizeof(recv_from_cmd_answer_type) != length) {
        std::cout << "recv command answer length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(recv_from_cmd_answer_type) << std::endl;
        return -1;
      }
      recv_cmdanswer = (recv_from_cmd_answer_type *)data;
      compar_id_recv_send(recv_cmdanswer);
      break;

    case RECV_CHASSIS_ERROE_REPORT:
      recv_chassis_error_report_type *recv_chassiserror;
      if (sizeof(recv_chassis_error_report_type) != length) {
        std::cout << "recv chassis error length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(recv_chassis_error_report_type) << std::endl;
        return -1;
      }
      recv_chassiserror = (recv_chassis_error_report_type *)data;
      pub_chassiserror(recv_chassiserror);
      break;

    case RECV_NAVIGATION_LOG_STATUS:
      recv_navigation_log_status_type *recv_navigationlog;
      if (sizeof(recv_navigation_log_status_type) != length) {
        std::cout << "recv navigation log length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(recv_navigation_log_status_type) << std::endl;
        return -1;
      }
      recv_navigationlog = (recv_navigation_log_status_type *)data;
      pub_navigationlog(recv_navigationlog);
      break;

    // SDK格式化是否成功的回复。0：成功
    case SEND_FORMAT_SD_CARD_CMD:
      send_format_sd_card_cmd_type *recv_formatsdcard;
      if (sizeof(send_format_sd_card_cmd_type) != length) {
        std::cout << "recv format sd card length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(send_format_sd_card_cmd_type) << std::endl;
        return -1;
      }
      recv_formatsdcard = (send_format_sd_card_cmd_type *)data;
      pub_formatsdcard(recv_formatsdcard);
      break;

    case RECV_FROM_GPS:
      recv_gps_data_type *recv_gps;
      if (sizeof(recv_gps_data_type) != length) {
        std::cout << "recv gps_data length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(recv_gps_data_type) << std::endl;
        return -1;
      }
      recv_gps = (recv_gps_data_type *)data;
      pub_gps(recv_gps);
      break;

    case UPD_CMD_206:
      get_version_flag = false;
      need_send_rough = true;  //运运控重启之后，重新进行时间粗同步
      Info("R_CMD_206: " << 1);
      break;

    case TIME_SYNC_ROUGH_CMD:
      time_sync_rough_msg_type *recv_rough_res;
      if (sizeof(time_sync_rough_msg_type) != length) {
        std::cout << "recv rough_time_response length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(time_sync_rough_msg_type) << std::endl;
        return -1;
      }
      recv_rough_res = (time_sync_rough_msg_type *)data;
      check_rough_res(recv_rough_res);
      break;

    case TIME_SYNC_FINE_CMD:
      if (sizeof(time_sync_fine_msg_type) != length) {
        std::cout << "recv fine_time_response length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(time_sync_fine_msg_type) << std::endl;
        return -1;
      }
      break;

    case TIME_SYNC_DELAY_CMD:
      if (sizeof(time_sync_fine_msg_type) != length) {
        std::cout << "recv delay_time_response length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(time_sync_fine_msg_type) << std::endl;
        return -1;
      }
      send_delay_sync_cmd();
      break;

    case RECV_FROM_DUSTBIN_INFO:
      clean_to_motion_t_new *recv_clean_state_new;
      if (sizeof(clean_to_motion_t_new) == length) {
        recv_clean_state_new = (clean_to_motion_t_new *)data;
        pub_clean_state_new(recv_clean_state_new);
      } else {
        std::cout << "recv clean_to_motion_t length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(clean_to_motion_t_new) << std::endl;
        return -1;
      }
      break;
    case RAIN_SENSOR:
      rain_sensor_t_new *recv_rain_sensor_new;
      if (sizeof(rain_sensor_t_new) != length) {
        std::cout << "recv rain_sensor_t length: " << length
                  << "size of struct length new: " << sizeof(rain_sensor_t_new)
                  << std::endl;
        return -1;
      }
      if (sizeof(rain_sensor_t_new) == length) {
        recv_rain_sensor_new = (rain_sensor_t_new *)data;
        pub_rain_sensor_new(recv_rain_sensor_new);
      }
      break;
    default:
      break;
  }
  return 0;
}