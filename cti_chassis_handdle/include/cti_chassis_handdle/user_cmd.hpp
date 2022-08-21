#ifndef USER_CMD_H
#define USER_CMD_H
#include <inttypes.h>  //or stdint.h uint16_t uint32_t
#include <string>

#define SWEEPER_ULT_MODULE_NUM 3
#define ULT_3_0_TYPE_NUM 6
//*************************正常工作模式0　测试模式3
//*************************收到测试模式时，开始定时１０分钟过滤上游信息
struct process_work_mode{
  static const int PROCESS_WORK_MODE_INTERVAL{600};  //设置测试模式过滤时间
  bool SPRAY_LOCK{0};  //设置喷水锁
  int cur_cnt{0};
  enum process_work_mode_enum {
    process_work_mode_normal = 1,
    process_work_mode_test = 2,
    process_work_mode_stop_spray = 0,
  } process_work_mode_enum_;
};

//*************************3_0_超声波名称枚举
typedef enum ult_3_0_type_enum {
  ult_3_0_front,
  ult_3_0_right_front_down,
  ult_3_0_right_down,
  ult_3_0_back,
  ult_3_0_left_down,
  ult_3_0_left_front_down
} ult_3_0_type_enum_t;


//*************************3_0_超声波类型话题名称枚举
const std::string ult_3_0_name[ULT_3_0_TYPE_NUM] = {
    "ult_3_0_front", "ult_3_0_right_front_down", "ult_3_0_right_down",
    "ult_3_0_back",  "ult_3_0_left_down",        "ult_3_0_left_front_down"};
    
//************************命令id的枚举
enum {
  USER_CMD_START = 1,
  /********USER CODE BEGIN********/
  /*add your command here*/
  SEND_TO_TIMENOW_CMD = 4,             //发送时间，为时间同步
  SEND_TO_CONTROL_CMD = 5,             //发送控制信息
  SEND_TO_CONTROL_CONTRAPOSITION = 6,  //对箱信息
  SEND_TO_CONTROL_BOX = 7,             //箱子升降
  SEND_TO_CONTROL_BOX_LOCK = 8,        //锁箱信息
  SEND_TO_POWEROFF_CMD = 9,            //掉电命令

  RECV_FROM_DUSTBIN_INFO = 12,   //接收清扫车命令
  SEND_TO_DUSTBIN_CMD = 13,      //清扫车命令
  SEND_BATTERY_CMD = 16,         //无线充电控制命令
  SEND_TO_DUST_BOX_CMD = 17,     //集尘箱控制命令
  RECV_FROM_DUST_BOX_INFO = 18,  //集尘箱信息接收
  SEND_TO_LIGHT_CMD = 21,        // ligt cmd for sannitation 3.0

  SMART_TRASH_SEND = 22,  //智能垃圾箱升降发送
  SMART_TRASH_RECV = 23,  //智能垃圾箱状态接收
  RAIN_SENSOR = 27,       //雨水传感器

  TIME_SYNC_ROUGH_CMD = 32,  //粗时间同步，发送和接收命令相同
  TIME_SYNC_FINE_CMD = 33,   //细时间同步，发送和接收命令相同
  TIME_SYNC_DELAY_CMD = 34,  //时间延时，发送和接收命令相同

  DUSBIN_RF_SET_READ_CMD = 40,  //设定清扫箱箱号,接收清扫箱箱号

  DUST_BOX_5G_CHECK_CMD = 80,  //查询吸尘箱5g状态
  DUST_BOX_5G_RECV_CMD = 81,   //接收吸尘箱5g状态

  RECV_FROM_RTK = 98,                             //接收底盘RTK信息
  RECV_FROM_GPS = 99,                             //接收底盘GPS信息
  RECV_FROM_CONTROL_STATUS = 105,                 //接收底盘控制信息
  RECV_FROM_CONTROL_BATTERY_4_TO_1_STATUS = 106,  //接收带电池信息
  RECV_FROM_DRIVER_STATUS = 107,     //接收电机驱动状态信息
  RECV_FROM_CMD_ANSWER = 108,        //接收命令应答信息
  RECV_FROM_RFID_INFO = 109,         //接收RFID的信息
  RECV_CHASSIS_ERROE_REPORT = 117,   //接收底盘错误码
  RECV_NAVIGATION_LOG_STATUS = 111,  //接收底盘重要的运行信息

  RECV_FROM_ULTRASONIC_DATA = 127,  //接收超声波信息
  SEND_FORMAT_SD_CARD_CMD = 128,    // sd卡格式化命令

  ULT_POS_SEND_CMD = 132,  // 3.0超声波

  SEND_TO_CHECK_PROGRAM_VERSION =
      209,  //定义：工控机查询STM32各程序区版本号命令.
  RECV_FROM_FIRMWARE_VERSION =
      198,  //定义：STM32返回各程序区版本号字符串命令，以及当前升级状态标记等。

  /********USER CODE END*********/
  USER_CMD_END = 199,
  UPD_CMD_206 = 206,
  UPD_CMD_210 = 210,  //查询版本的命令2

};
//**************************各模块地址
enum {
  MODULE_NONE = 0,
  MODULE_DEBUG_BOARD = 1,  // 10-10 belongs to normal moduls
  MODULE_CONTROL_PC = 2,
  MODULE_MOVE_CONTROL_BOARD = 3,
  MODULE_POWER_INTEGRATE_BOARD = 4,
  MODULE_SWEEPING_BOX_BOARD = 5,
  MODULE_DUST_BOX_BOARD = 6,
  MODULE_GPS = 7,
  MODULE_POWER_BOARD = 8,

  MODULE_MOTOR_DRIVER_START = 10,  // 10-29 belongs to motor drivers
  MODULE_MOTOR_DRIVER_FRONT_LEFT = MODULE_MOTOR_DRIVER_START,
  MODULE_MOTOR_DRIVER_FRONT_RIGHT = 11,
  MODULE_MOTOR_DRIVER_BACK_LEFT = 12,
  MODULE_MOTOR_DRIVER_BACK_RIGHT = 13,
  MODULE_MOTOR_TURN_FRONT = 14,
  MODULE_MOTOR_TURN_BACK = 15,
  MODULE_MOTOR_BRAKE_FRONT = 16,
  MODULE_MOTOR_BREAK_BACK = 17,
  MODULE_MOTOR_DRIVER_END = MODULE_MOTOR_BREAK_BACK,

  MODULE_LIGHT_START = 30,  // 30-49 belongs to lights
  MODULE_LIGHT_FRONT_LEFT = MODULE_LIGHT_START,
  MODULE_LIGHT_FRONT_RIGHT = 31,
  MODULE_LIGHT_BACK_LEFT = 32,
  MODULE_LIGHT_BACK_RIGHT = 33,
  MODULE_LIGHT_BACK = 34,
  MODULE_LIGHT_HEAD = 35,
  MODULE_LIGHT_END = MODULE_LIGHT_HEAD,
  MODULE_LIGHT_TAPE = 39,

  MODULE_PUSH_ROD_START = 40,
  MODULE_PUSH_ROD_0 = MODULE_PUSH_ROD_START,
  MODULE_PUSH_ROD_1 = 41,
  MODULE_PUSH_ROD_2 = 42,
  MODULE_PUSH_ROD_3 = 43,
  MODULE_PUSH_ROD_4 = 44,
  MODULE_PUSH_ROD_5 = 45,
  MODULE_PUSH_ROD_6 = 46,
  MODULE_PUSH_ROD_7 = 47,
  MODULE_PUSH_ROD_8 = 48,
  MODULE_PUSH_ROD_9 = 49,
  MODULE_PUSH_ROD_10 = 50,
  MODULE_PUSH_ROD_11 = 51,
  MODULE_PUSH_ROD_12 = 52,
  MODULE_PUSH_ROD_13 = 53,
  MODULE_PUSH_ROD_14 = 54,
  MODULE_PUSH_ROD_15 = 55,
  MODULE_PUSH_ROD_END = MODULE_PUSH_ROD_15,

  MODULE_ULTRASONIC_START = 60,
  MODULE_ULTRASONIC_MASTER = MODULE_ULTRASONIC_START,
  MODULE_ULTRASONIC_1 = 61,
  MODULE_ULTRASONIC_2 = 62,
  MODULE_ULTRASONIC_3 = 63,
  MODULE_ULTRASONIC_4 = 64,
  MODULE_ULTRASONIC_5 = 65,
  MODULE_ULTRASONIC_6 = 66,
  MODULE_ULTRASONIC_7 = 67,
  MODULE_ULTRASONIC_8 = 68,
  MODULE_ULTRASONIC_9 = 69,
  MODULE_ULTRASONIC_10 = 70,
  MODULE_ULTRASONIC_11 = 71,
  MODULE_ULTRASONIC_12 = 72,
  MODULE_ULTRASONIC_13 = 73,
  MODULE_ULTRASONIC_14 = 74,
  MODULE_ULTRASONIC_15 = 75,
  MODULE_ULTRASONIC_END,

  MODULE_BELOW_START = 80,  // ohter modules start
  MODULE_CHECK_UPD = 81,
  MAX_MODULE,
};
//**************************控制命令发送结构体中的union
typedef union {
  uint8_t data;
  struct {
    uint8_t localizer : 1;  // 1:已定位 0： 未定位
    uint8_t unused1 : 1;
    uint8_t unused2 : 1;
    uint8_t unused3 : 1;
    uint8_t unused4 : 1;
    uint8_t unused5 : 1;
    uint8_t unused6 : 1;
    uint8_t unused7 : 1;
  } bits;
} robot_status_t;
//**************************控制命令发送结构体
typedef struct __attribute__((packed)) {
  uint8_t portIndex;
  uint16_t cmd_id;  //命令id
  uint16_t linkCnt;
  uint8_t linkFlag;
  float cmd_vel_Vx;       // x轴线速度
  float cmd_vel_Vy;       // y轴线速度
  float cmd_vel_W;        //角度
  int8_t cmd_turn_mode;   // 0:front_turn	1:front_rear_turn
  int8_t cmd_break_flag;  //刹车
  int8_t up_down_flag;    // 0:stop,-1:down,1:up 箱子升降
  int8_t switch_flag;
  union {
    uint16_t light_type;  //灯控指令
    struct {
      uint8_t position : 1;    // LED3 6 9 12 13 on
      uint8_t brake : 1;       // LED7 10 on
      uint8_t turn_left : 1;   // LED2 8 blink
      uint8_t turn_right : 1;  // LED5 11 blink
      uint8_t headlight : 1;   // led1 4 on
      uint8_t warning : 1;     // led 14 on
      uint8_t fan : 1;         // led fan1 fan2 on
      uint8_t backward : 1;    // move backward reserved
      uint8_t dangerous : 1;   // dangerous warning  reservd
      uint8_t free : 7;
    } LightTypeBits;
  };
  float pose[3];                    //上位机imu位置
  float q[4];                       //上位机imu四元数
  uint32_t imu_flag;                // imu状态位
  int16_t front_obstacle_distance;  //前方障碍物cm
  int16_t rear_obstacle_distance;   //后方障碍物cm
  robot_status_t robot_status;
} send_to_control_cmd_type;  //

//**************************浅休眠发送结构体，和掉电命令用同一个命令号
typedef struct __attribute__((packed)) {
  // uint8_t portIndex;
  uint16_t cmd_id;
  int16_t power_off_flag[4];  //[0]:休眠时间:分钟 [1]-[3]保留
  uint32_t power_control;     //用于控制电源板设备
  uint32_t bat_control;       //用于控制汇接板设备
  uint32_t unused;            //预留
} send_to_poweroff_cmd_type_new;
//电源板设备
typedef union {
  uint32_t power_control;
  struct {
    uint32_t Debug : 1;     //调试接口板
    uint32_t IPC : 1;       //工控机
    uint32_t Lidar : 1;     //雷达
    uint32_t Audio : 1;     //功放
    uint32_t Fan : 1;       //风扇
    uint32_t IMU : 1;       //惯性传感器
    uint32_t WAN : 1;       //路由器
    uint32_t HUB : 1;       //交换机
    uint32_t Camera : 1;    //行车记录仪    已删 3.0可控 1.2不可控
    uint32_t RTK : 1;       //定位
    uint32_t ULT : 1;       //超声波        已删 3.0可控 1.2不可控
    uint32_t USB : 1;       // 5V USB        已删 3.0可控 1.2不可控
    uint32_t Nano : 1;      //英伟达
    uint32_t LCD : 1;       //显示屏
    uint32_t Vol_12V : 1;   //预留口12V
    uint32_t Vol_12V1 : 1;  //预留口12V     已删 3.0可控 1.2不可控
    uint32_t Vol_12V2 : 1;  //预留口12V     已删 3.0可控 1.2不可控
    uint32_t Vol_12V3 : 1;  //预留口12V     已删 3.0可控 1.2不可控
    uint32_t Vol_48V : 1;   //预留口48V
    uint32_t Light;         //灯板
    uint32_t unused1 : 1;
    uint32_t unused2 : 1;
    uint32_t unused3 : 1;
    uint32_t unused4 : 1;
    uint32_t unused5 : 1;
    uint32_t unused6 : 1;
    uint32_t unused7 : 1;
    uint32_t unused8 : 1;
    uint32_t unused9 : 1;
    uint32_t unused10 : 1;
    uint32_t unused11 : 1;
    uint32_t unused12 : 1;
  } bits;
} power_control_t;
//汇接板设备
typedef union {
  uint32_t bat_control;
  struct {
    uint32_t inverter : 1;                    //逆变器
    uint32_t mouse_expeller : 1;              //驱鼠器
    uint32_t Left_wheel_drive : 1;            //左轮驱动
    uint32_t right_wheel_drive : 1;           //右轮驱动
    uint32_t clean_the_board : 1;             //清扫板
    uint32_t brake_driver : 1;                //刹车驱动
    uint32_t steer_driver : 1;                //转向驱动
    uint32_t move_control_board : 1;          //运控
    uint32_t power_interface_board : 1;       //电源板
    uint32_t internal_contracting_brake : 1;  //抱闸
    uint32_t unused1 : 1;
    uint32_t unused2 : 1;
    uint32_t unused3 : 1;
    uint32_t unused4 : 1;
    uint32_t unused5 : 1;
    uint32_t unused6 : 1;
    uint32_t unused7 : 1;
    uint32_t unused8 : 1;
    uint32_t unused9 : 1;
    uint32_t unused10 : 1;
    uint32_t unused11 : 1;
    uint32_t unused12 : 1;
    uint32_t unused13 : 1;
    uint32_t unused14 : 1;
    uint32_t unused15 : 1;
    uint32_t unused16 : 1;
    uint32_t unused17 : 1;
    uint32_t unused18 : 1;
    uint32_t unused19 : 1;
    uint32_t unused20 : 1;
    uint32_t unused21 : 1;
    uint32_t unused22 : 1;
  } bits;
} union_bat_control;

//**************************时间同步命令发送结构体
typedef struct __attribute__((packed)) {
  int year; /**< Years since 1900 */
  int mon;  /**< Months since January - [0,11] */
  int day;  /**< Day of the month - [1,31] */
  int hour; /**< Hours since midnight - [0,23] */
  int min;  /**< Minutes after the hour - [0,59] */
  int sec;  /**< Seconds after the minute - [0,59] */
  int hsec; /**< Hundredth part of second - [0,99] */
} send_to_timenow_cmd_type;

//**************************查询版本命令发送结构体
typedef struct __attribute__((packed)) {
  unsigned char src;   //发起模块地址
  unsigned char dest;  //接收模块地址
} update_info_type;
typedef struct __attribute__((packed)) {
  update_info_type upd_info;
  unsigned char check;  // check: 01  要查询
} send_to_check_version_type;

//*************************带有id的发送命令包结构体
//class serial_frame_include_id_type
typedef struct __attribute__((packed))
{
  uint16_t id;
  unsigned char cmd;
  int need_id;  // 1:need 0:no need
  serial_frame_type frame;
}serial_frame_include_id_type;

//**************************版本信息接收结构体
#define MAX_VERSION_STR 10
typedef struct __attribute__((packed)) {
  update_info_type upd_info;
  unsigned char run_area;       // 0:boot, 1:app, 2:app2
  unsigned char update_status;  // update status in stage
  char boot_ver[MAX_VERSION_STR];
  // unsigned char boot_ver[MAX_VERSION_STR];
  char app_ver[MAX_VERSION_STR];
  // unsigned char app_ver[MAX_VERSION_STR];
  char update_lib_ver[MAX_VERSION_STR];
  // unsigned char update_lib_ver[MAX_VERSION_STR];
} recv_from_firmware_version_type;

//*************************命令应答接收结构体
typedef struct __attribute__((packed)) {
  uint8_t portIndex;
  uint16_t cmd_id;
} recv_from_cmd_answer_type;
//*************************控制信息接收里的union
typedef union {
  uint16_t data;
  struct {
    uint8_t turn_front_left : 1;    //前转向的左边的限位霍尔
    uint8_t turn_front_center : 1;  //前转向中间的限位霍尔
    uint8_t turn_front_right : 1;   //前转向右边的限位霍尔
    uint8_t front_bar : 1;          //前防撞杆
    uint8_t rear_bar : 1;           //后防撞杆
    uint8_t turn_rear_left : 1;     //后转向的左边的限位霍尔
    uint8_t turn_rear_center : 1;   //后转向的中间的限位霍尔
    uint8_t turn_rear_right : 1;    //后转向的右边的限位霍尔
    uint8_t box_pos_reach1 : 1;     //箱子霍尔1
    uint8_t box_pos_reach2 : 1;     //箱子霍尔2
    uint8_t box_pos_reach3 : 1;     //箱子霍尔3
    uint8_t lift_pressure : 1;      //顶升防触边0：没触到 1：触到
    uint8_t reserve : 1;
    uint8_t unused1 : 1;
    uint8_t unused2 : 1;
    uint8_t unused3 : 1;
  } bits;
} sw_status_t;
//*************************控制状态接收结构体
typedef struct __attribute__((packed)) {
  uint8_t portIndex;
  int8_t state_vehicle;     //底盘状态
  uint16_t drivers_enable;  //电机驱动器使能
  int8_t control_mode;      //控制模式
  int8_t state_brake;       //刹车
  float vel_liner_x;  // x线速度  vx == 100时！是底盘有问题！！！！！！！！！
  float vel_liner_y;       // y线速度
  float vel_angular;       //角速度 目前无
  float angle_front_turn;  //前轮转向
  float angle_rear_turn;   //后轮转向
  // uint8_t box_sw; //霍尔状态
  // uint8_t front_bar; //前防撞杆
  // uint8_t rear_bar;//后防撞杆
  sw_status_t sw_status;
  double odometer;  //里程计
  struct {
    double x;
    double y;
  } pos;
  float angle_yaw_radian;
  uint8_t lift_position;     //顶升位置
  float acc[3];              //三轴加速度
  float gyro[3];             //三轴角速度
  float q[4];                //姿态四元数
  float position[3];         //位置
  float linear_velocity[3];  //线速度
  float baro_raw;            //气压原始数据
  float baro_height;         //气压计 高度
  int compass1_str[3];
  int compass2_str[3];
  uint32_t state_flag;    //状态位
  uint8_t laser_id[6];    //激光对箱id 6bytes
  uint8_t laser_data[2];  //激光对箱数据 2bytes
} recv_from_control_status_type;

//*************************电池状态接收结构体 不带无线充电
typedef struct __attribute__((packed)) {
  uint8_t portIndex;

  float Bat_1_Volt;          // voltage
  uint8_t Bat_1_Soc;         // percentage
  float Bat_1_temp;          // temperature
  float Bat_1_curr;          // current
  uint8_t Bat_1_comm_state;  // communication state, 0:communication
                             // error,1:communication ok
  uint8_t Bat_1_cell_num;    // battery cell number

  float Bat_2_Volt;
  uint8_t Bat_2_Soc;
  float Bat_2_temp;
  float Bat_2_curr;
  uint8_t Bat_2_comm_state;
  uint8_t Bat_2_cell_num;

  float Bat_3_Volt;
  uint8_t Bat_3_Soc;
  float Bat_3_temp;
  float Bat_3_curr;
  uint8_t Bat_3_comm_state;
  uint8_t Bat_3_cell_num;

  float Bat_backup_Volt;  // backup battery voltage

  float Bus_Volt;      // main power bus voltage
  float Charger_Volt;  // charger voltage

  // battery board temperature
  float Board_temp1;  // Bat1 MOS
  float Board_temp2;  // MCU
  float Board_temp3;  // BAT2 BAT3 MOS
  float Board_temp4;  // front left motor driver MOS
  float Board_temp5;  // charger In MOS
  float Board_temp6;  // Move Control board port
  float Board_temp7;  // back left motor driver MOS
  float Board_temp8;  // AI Computer port

  float Board_curr1;  // wheel1
  float Board_curr2;  // wheel2
  float Board_curr3;  // wheel3
  float Board_curr4;  // wheel4
  float Board_curr5;  // AI Computer
  float Board_curr6;  // move control board and backup battery
  float Board_curr7;  // brake motor
  float Board_curr8;  // steel driver

  uint8_t charge_reverse;  // charger voltage reverse
  uint8_t v_leakage;       // shell with high voltage

  uint8_t ChargeStatus;
  uint8_t Lock_status;
  union {
    uint32_t errorInfo;
    struct {
      uint32_t bat1_volt_exception : 1;
      uint32_t bat2_volt_exception : 1;
      uint32_t bat3_volt_exception : 1;
      uint32_t reserve_bat_volt_exception : 1;
      uint32_t bat1_temp_exception : 1;
      uint32_t bat2_temp_exception : 1;
      uint32_t bat3_temp_exception : 1;
      uint32_t reserve_temp_exception : 1;
      uint32_t charge_volt_exception : 1;
      uint32_t board_pos1_temp_exception : 1;
      uint32_t board_pos2_temp_exception : 1;
      uint32_t board_pos3_temp_exception : 1;
      uint32_t board_pos4_temp_exception : 1;
      uint32_t box_lock_suction_exception : 1;
      uint32_t shell_leakage : 1;
      uint32_t remote_shutdown : 1;
      uint32_t emergency_btn_press : 1;
      uint32_t wheel_driver1_over_current_protect : 1;
      uint32_t wheel_driver2_over_current_protect : 1;
      uint32_t wheel_driver3_over_current_protect : 1;
      uint32_t wheel_driver4_over_current_protect : 1;
      uint32_t industrial_pc_over_current_protect : 1;
      uint32_t motion_control_board_over_current_protect : 1;
      uint32_t brake_motor_over_current_protect : 1;
      uint32_t turn_motor_over_current_protect : 1;
      uint32_t unused1 : 1;
      uint32_t unused2 : 1;
      uint32_t unused3 : 1;
      uint32_t unused4 : 1;
      uint32_t unused5 : 1;
      uint32_t unused6 : 1;
      uint32_t unused7 : 1;
    } errorInfoBits;
  };
} recv_battery_4_to_1_active_report_status_type;  //

//*************************无线充电 软急停 控制结构体
typedef struct __attribute__((packed)) _battery_board_cmd_s {
  uint8_t wireless_charge{0};  //充电 0:关闭充电 1:开始充电
  uint8_t soft_stop{0};        //软急停 0:松开急停　１：按下急停
  uint8_t exit_charging_state{0};  //　３。０退出充电状态  0默认  1退出
  uint8_t unused3{0};
  uint8_t unused4{0};
  uint8_t unused5{0};
} battery_board_cmd_t;

//************************底盘错误信息接收结构体
//错误类型
typedef enum module_error_enum {
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
} module_error_t;
//错误级别
typedef enum {
  warning_level = 1,
  serious_level = 2,
  deadly_level = 3
} chassis_error_level_enum;

//串口通讯结构体
typedef struct __attribute__((packed)) {
  uint8_t module_type;
  uint8_t module_error_level;
  uint32_t module_error_code;
} recv_chassis_error_report_type;

//************************底盘运行重要信息接收结构体
typedef struct __attribute__((packed)) {
  uint8_t portIndex;
  float liner_speed;
  float turn_angle;
  int8_t break_torque;  //刹车力度，直接向驱动下发力矩值
  uint8_t enable_flag;
  float actual_body_linear_vel_x;
  float actual_speed_base_on_left_front_wheel;  //左前轮折算的车体中心速度
  float actual_speed_base_on_right_front_wheel;
  float actual_speed_base_on_left_rear_wheel;
  float actual_speed_base_on_right_rear_wheel;
  float actual_turn_front_angle;  // unit:degree 前后转向电机的角度
  float actual_turn_rear_angle;   // unit:degree
  int16_t set_torque[8];
  uint16_t now_encoder[8];
} recv_navigation_log_status_type;
//************************发送运控格式化SD卡
typedef struct __attribute__((packed)) {
  uint8_t portIndex;  // =0:格式化
} send_format_sd_card_cmd_type;
//*************************底盘gps信息接收
typedef struct __attribute__((packed)) {
  double utc_seconds;

  char position_stat;
  double lat;
  double lon;
  double alt;
  float lat_err;
  float lon_err;
  float alt_err;
  float diff_age;
  float undulation;
  uint32_t sats_tracked;
  uint32_t sats_used;

  char heading_stat;
  float heading;
  float pitch;
  float heading_err;
  float pitch_err;
  float baselineLen;

  char velocity_stat;
  float speed_north;
  float speed_east;
  float speed_up;
  float latency;
} recv_gps_data_type;
//*************************粗时间同步
typedef struct __attribute__((packed)) {
  uint16_t year;  // years since 1900
  uint8_t mon;    // months since january [1,12]
  uint8_t day;    // day of month [1,31]
  uint8_t hour;   // hours since midnight [0,23]
  uint8_t min;    // minutes after hour [0,59]
  uint8_t sec;    // seconds after the minutes [0,59]
} time_sync_rough_msg_type;

//*************************细时间同步
typedef struct __attribute__((packed)) {
  uint8_t sec;              // seconds after the minutes [0,59]
  uint16_t millisencond;    // millisecond[0,999]
  uint16_t microsecondsec;  // microsecondsec [0,999]
} time_sync_fine_msg_type;

//***********************清扫箱电机状态union
typedef union {
  uint8_t data;
  struct {
    uint8_t motro1 : 1;
    uint8_t motor2 : 1;
    uint8_t motor3 : 1;
    uint8_t motor4 : 1;
    uint8_t motor5 : 1;
    uint8_t unused5 : 1;
    uint8_t unused6 : 1;
    uint8_t unused7 : 1;
  } bits;
} motor_status_t;
//************************环卫车发送清扫的命令__新的增加边刷伸展功能等 3.0
typedef struct __attribute__((packed)) {
  uint8_t port{0};
  uint8_t engin_start{0};
  uint8_t side_brush{0};
  uint8_t main_brush{0};
  uint8_t spray_motor{0};
  uint8_t dust_suppresion{0};
  int8_t lift_motor{0};  //-1 下降 1： 上升 0：停止
  uint8_t led{0};
  uint8_t control_mode{2};
  uint8_t dustbin_on_car{0};
  uint8_t dam_board{1};             //挡板升降 1:降 0:升
  uint8_t side_brush_transform{0};  //边刷收缩/展开,百分比
  uint8_t side_brush_speed{0};      //边刷速度 单位待定
  uint8_t decorate_light;           //装饰灯 0:关 1:开
  uint8_t unused1;
  uint8_t unused2;
  uint8_t unused3;
  uint8_t lift_pump_switch;
  uint8_t shake_dust_motor_speed;  // 0 - 100
} motion_to_clean_t_new;

//************************环卫车接受到清扫的状态_新的增加边刷伸展功能等

typedef union {
  uint8_t data;
  struct {
    uint8_t decorate_light : 1;       //装饰灯 0:关 1:开
    uint8_t increase_water_pump : 1;  //加水水泵 0:关 1:开
    uint8_t unused1 : 1;
    uint8_t unused2 : 1;
    uint8_t unused3 : 1;
    uint8_t unused4 : 1;
    uint8_t unused5 : 1;
    uint8_t unused6 : 1;
  } bits;
} multi_status_t;

typedef struct __attribute__((packed)) {
  uint8_t portIndex;
  uint8_t water_tank_top;
  uint8_t water_tank_bottom;
  uint8_t spray_state;
  uint16_t dustbin_distance;
  uint16_t ultrasonic_distance[6];
  float voltage1;
  float voltage2;
  float voltage3;
  motor_status_t motor_status;
  uint8_t recv_dustbin_on_the_car;
  uint8_t
      dam_board_status;  //挡板状态: 0:升 1:降           //2:OPEN 1:CLOAE 3:AUTO
  uint8_t side_brush_transform_state;  ///边刷收缩/展开,百分比
  uint8_t side_brush_speed;            //边刷速度 单位待定
  uint32_t error_code;                 //错误码
  uint8_t side_brush_transform_error;  //边刷收缩,展开推杆错误码
  uint8_t left_side_brush_error;       //左边刷异常码
  uint8_t right_side_brush_error;      //右边刷异常码
  multi_status_t multi_status;         //多种状态
  int8_t lift_motorr_state;
  uint8_t control_mode_state;  // 1:up -1:bottom 2:acting
  uint8_t push_board_state[9];
  uint16_t push_board_error_code[9];
  uint8_t shake_dust_motor_speed_feedback;  // 0 - 100
  uint16_t roll_brush_speed;
  uint32_t roll_brush_error_code;
} clean_to_motion_t_new;

//************************发送到集尘箱的命令
typedef struct __attribute__((packed)) {
  uint8_t port{0};
  uint8_t engin_start{0};
  uint8_t side_brush{0};
  uint8_t main_brush{0};
  uint8_t spray_motor{0};
  uint8_t dust_suppresion{0};
  int8_t lift_motor{0};
  uint8_t led{0};
  uint8_t control_mode{2};
  uint8_t dustbin_on_car{0};
  uint8_t auto_push{0};   //自动倒垃圾 1:开 0:关
  uint8_t fan_speed{80};  //风机速度
  uint8_t ultrasonic_clean;
  uint8_t unused1;
} motion_to_dust_box_t;

//************************接受到集尘箱的状态(新的,增加电池状态,风机转矩等)
typedef struct __attribute__((packed)) {
  uint8_t portIndex;
  uint8_t auto_work;     // 0: 无效,无此功能 1：关 2：开
  uint8_t mannual_work;  // 0：无效，无此功能 1：关 2：开
  uint8_t trash_per;
  uint16_t dustbin_distance;
  uint16_t ultrasonic_distance[6];
  float voltage1;
  uint8_t Bat_1_Soc;
  float voltage2;
  uint8_t Bat_2_Soc;
  float voltage3;
  uint8_t Bat_3_Soc;
  float voltage4;
  uint8_t Bat_4_Soc;
  motor_status_t motor_status;
  uint8_t recv_dustbin_on_the_car;
  uint8_t auto_push;   //自动倒垃圾 1:开 0:关
  uint8_t fan_speed;   //风机速度
  uint8_t net_state;   //箱子
  uint8_t net_signal;  //
  //无线充电协议
  float wireless_voltage;    //充电电压
  float wireless_current;    //充电电流
  uint8_t wireless_reserve;  //保留
  union {
    uint8_t wireless_state;
    struct {
      uint8_t unused1 : 1;
      uint8_t unused2 : 1;
      uint8_t unused3 : 1;
      uint8_t charge_error : 1;  // charge voltage or current error
      uint8_t unused4 : 1;
      uint8_t unused5 : 1;
      uint8_t wireless_connected : 1;
      uint8_t temperature_error : 1;
    } wirelessStateBits;
  };
  uint8_t wireless_temp;     //充电板温度
  uint8_t ultrasonic_clean;  //是否在充电

  uint8_t fan_torque;             //风机转矩
  uint16_t fan_speed_closedloop;  //风机转速,闭环控制
  uint8_t fan_current;            //风机电流
  uint64_t errorInfo;             //板子错误码

  uint8_t infrared_data;     //红外数据
  uint8_t anti_pinch_data;   //防夹数据
  float stay_cord_distance;  //拉绳距离

} dust_box_to_motion_t_new;
///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
//设定清扫箱箱号,接收清扫箱箱号
typedef struct __attribute__((packed)) {
  uint8_t rw;        // 0[read]  1[write]
  uint8_t reset;     // rw   default 0
  uint8_t mode;      // r
  uint8_t link;      // r
  int16_t baud{-1};  // rw   default -1
  int16_t id;        // rw
} dustbin_rf_set_cmd_t;

//清扫箱箱号发送和接受状态结构体
struct dusbin_rf_set_state_t {
  bool if_send;
  int send_id;
  bool if_recv;
  int recv_id;
  int send_id_times;
  int send_id_loop_cnt;
  dusbin_rf_set_state_t() {
    if_send = false;
    send_id = -1;
    if_recv = false;
    recv_id = -2;
    send_id_times = 0;
    send_id_loop_cnt = 0;
  }
  void reset() {
    if_send = false;
    send_id = 1;
    if_recv = false;
    recv_id = -2;
    send_id_times = 0;
    send_id_loop_cnt = 0;
  }
};

typedef union {
  uint16_t data;
  struct {
    uint16_t switch_door : 1;
    uint16_t tool_door : 1;
    uint16_t reserve1 : 14;
  } bits;
} rain_sensor_flag_t;

//接收雨水传感器状态的结构体————新的
typedef struct __attribute__((packed)) {
  rain_sensor_flag_t
      flag;  // 3.0车辆左右两侧门开关传感器状态放在这里。详见rain_sensor_flag_t
  uint8_t right;
  uint8_t left;
} rain_sensor_t_new;

//智能垃圾箱的控制和状态上传
typedef struct __attribute__((packed)) {
  uint8_t lift_mode;          // 1:上升 2：下降：3:急停
  uint16_t reaction_control;  // 1:关闭感应开盖 0:允许感应开盖
  uint16_t unuse2;
  uint16_t unuse3;
  uint16_t unuse4;
} smart_trash_cmd;
typedef struct __attribute__((packed)) {
  uint8_t current_position;  // 1:底部：2:顶部
  uint8_t kinestate;         // 0：静止 1:向上，2向下
  uint16_t location;         //位置
  uint16_t errorinfo;        //升降报错
  uint16_t reaction_state;   // 1:关闭感应开盖 0:允许感应开盖
  uint16_t unuse2;
  uint16_t unuse3;
} smart_trash_report;

typedef struct {
  uint8_t uint8_data;
  int8_t int8_data;
  uint16_t uint16_data;
  int16_t int16_data;
  uint32_t uint32_data;
  int32_t int32_data;
  uint64_t uint64_data;
  int64_t int64_data;
  float float_data;
  double double_data;
  std::string string_data;
} parsed_ChassisInfo_t;
typedef struct vehicle_water_box_status {
  double water_percnt_now;
  bool is_out;
  bool is_in;
  double out_pre_time;
  double in_pre_time;
  double out_per_sec;
  double in_per_sec;
  vehicle_water_box_status() {
    water_percnt_now = 90.0;
    is_out = false;
    is_in = false;
    out_pre_time = 0.0;
    in_pre_time = 0.0;
    out_per_sec = 0.0;
    in_per_sec = 0.0;
  }
} vehicle_water_box_status;

//吸尘箱5g状态的查询和状态上传
typedef struct __attribute__((packed)) dustbox_5g_check_cmd_t {
  uint8_t port;
  uint8_t msg1;
  uint8_t unused1;
  uint8_t unused2;
  dustbox_5g_check_cmd_t() {
    port = 0;
    msg1 = 0;
    unused1 = 0;
    unused2 = 0;
  }
} dustbox_5g_check_cmd_t;
typedef struct __attribute__((packed)) {
  uint8_t portIndex;
  uint8_t msg1[32];
  uint8_t unused1[32];
  uint8_t unused2[32];
} dustbox_5g_recv_cmd_t;
//---------------------------
typedef struct chat_state {
  int module_id;  //模块id 0:底盘;1:清扫箱(旧);2:吸尘箱;3:智能垃圾箱
  bool recv_state;   //接收状态 true:正常 false:异常
  double time_recv;  //最近的接收时间
  chat_state() {
    module_id = -1;
    recv_state = false;
    time_recv = 0.0f;
  }
  void reset() {
    module_id = -1;
    recv_state = false;
    time_recv = 0.0f;
  }
} chat_state;

typedef struct __attribute__((packed)) {
  /// saturated uint8 warning_light
  uint8_t warning_light;

  /// saturated uint8 headlight_right
  uint8_t headlight_right;

  /// saturated uint8 headlight_left
  uint8_t headlight_left;

  /// saturated uint8 backlight_right
  uint8_t backlight_right;

  /// saturated uint8 backlight_left
  uint8_t backlight_left;

  /// saturated uint8 headlight_circle
  uint8_t headlight_circle;

  /// saturated uint8 backlight_circle
  uint8_t backlight_circle;

  /// saturated uint8 turn_light_right
  uint8_t turn_light_right;

  /// saturated uint8 turn_light_left
  uint8_t turn_light_left;

  /// saturated uint8 break_light
  uint8_t break_light;

  /// saturated uint8 beep
  uint8_t beep;
} msg_light_cmd_3_0;

// humble３．０超声波协议
typedef struct __attribute__((packed)) {
  uint16_t dat_seq;       // sequence
  uint16_t com_seq;       // sequence
  uint8_t pos_flag[2];    // 0:normal 1:pos 2:pos lost
  uint16_t upa_data[8];   // Data[0] ~ sensor1  (unit:mm)
  int16_t pos_dat[2][2];  // pos_dat[x][y] x:pos num; y:pos info
  uint16_t unused;
} msg_upa_pos_data_t;

//汇接板
typedef struct __attribute__((packed)) {
  uint8_t portIndex;

  float Bat_1_Volt;          // voltage
  uint8_t Bat_1_Soc;         // percentage
  float Bat_1_temp;          // temperature
  float Bat_1_curr;          // current
  uint8_t Bat_1_comm_state;  // communication state, 0:communication
                             // error,1:communication ok
  uint8_t Bat_1_cell_num;    // battery cell number
  float full_capacity;       //满容量
  float surplus_capacity;    //剩余容量
  uint8_t Bat_1_Soh;         // SOH

  uint16_t cell_state;       //电芯状态1-16
  uint16_t ntc_state;        // ntc状态1-7
  uint16_t volt_curr_state;  //电池电压电流状态
  uint16_t user1_state;      //用户自定义状态1
  uint16_t user2_state;      //用户自定义状态2
  uint16_t user3_state;      //用户自定义状态3
  uint16_t user4_state;      //用户自定义状态4

  float Bus_Volt;      // main power bus voltage
  float Charger_Volt;  // charger voltage

  // battery board temperature
  float Board_temp1;  // Bat1 MOS
  float Board_temp2;  // MCU
  float Board_temp3;  //
  float Board_temp4;  // front left motor driver MOS
  float Board_temp5;  // charger In MOS
  float Board_temp6;  // clean board port
  float Board_temp7;  // back left motor driver MOS
  float Board_temp8;  // inverter port

  float Board_curr1;  // wheel1
  float Board_curr2;  // wheel2
  float Board_curr3;  // inverter
  float Board_curr4;  // clean
  float Board_curr5;  // power
  float Board_curr6;  // move control board
  float Board_curr7;  // brake motor
  float Board_curr8;  // steel driver

  uint8_t charge_reverse;  // charger voltage reverse
  uint8_t v_leakage;       // shell with high voltage

  uint8_t ChargeStatus;  //板子充电状态
  uint8_t brake_status;  //抱闸状态
  uint32_t ErrorInfo;    //错误码

  float battery_charger_voltage;  //充电机实际输出电压
  float battery_charger_current;  //充电机实际输出电流
  uint8_t battery_charger_flag;   //充电标志
  uint8_t battery_charger_state;  //充电机状态

  uint32_t uused1;  //预留
  uint32_t uused2;
  uint32_t uused3;
} recv_battery_4_to_1_active_report_status_type_3_0_;

typedef union {
  uint32_t Machine_error;
  struct {
    uint32_t BAT_VOLT_ERR : 1;          //电池电压异常
    uint32_t BAT_NOT_ERR : 1;           //电池不在位
    uint32_t BAT_NTC_ERR : 1;           //电池电芯温度异常
    uint32_t BAT_TEMP_ERR : 1;          //电池通道温度异常
    uint32_t BAT_COM_ERR : 1;           //电池通信异常
    uint32_t BAT_REP_ERR : 1;           //电池主动报错
    uint32_t CHARGE_REVERSE_ERR : 1;    //充电口反接
    uint32_t CHARGE_ERR : 1;            //充电口电压异常
    uint32_t CHARGE_COM_ERR : 1;        //充电机通信异常
    uint32_t CHARGE_TEMP_ERR : 1;       //充电机输入MOS温度异常
    uint32_t MCU_TEMP_ERR : 1;          // MCU温度异常
    uint32_t LEFT_STEER_TEMP_ERR : 1;   //左后轮供电MOS温度异常
    uint32_t RIGHT_STEER_TEMP_ERR : 1;  //右后轮供电MOS温度异常
    uint32_t CLEAN_TEMP_ERR : 1;        //清扫主板供电MOS温度异常
    uint32_t INVERTER_TEMP_ERR : 1;     //逆变器供电MOS温度异常
    uint32_t SHELL_LEAKAGE : 1;         //外壳带电
    uint32_t SLEEP_FLAG : 1;            //休眠
    uint32_t HARD_EMERGENCY_OFF : 1;    //硬急停
    uint32_t SOFT_EMERGENCY_OFF : 1;    //软急停
    uint32_t WHEEL1_ERR : 1;            //左后轮过流
    uint32_t WHEEL2_ERR : 1;            //右后轮过流
    uint32_t INVERTER_ERR : 1;          //逆变器过流
    uint32_t CLEAN_ERR : 1;             //清扫过流
    uint32_t POWER_ERR : 1;             //电源板过流
    uint32_t MOVE_ERR : 1;              //运控板过流
    uint32_t BREAK_ERR : 1;             //刹车过流
    uint32_t STEER_ERR : 1;             //转向过流
    uint32_t BAND_TYPE_ERR : 1;         //抱闸异常（预留）
    uint32_t POWER_BORAD_COM : 1;       //电源板通信失败
    uint32_t unused2 : 1;
    uint32_t unused3 : 1;
  } bits;
} UNION_MACHINE_ERROR;


typedef struct __attribute__((packed))
{
    double utc_seconds;
    int8_t position_stat;
    double lat;
    double lon;
    double alt;
    float lat_err;
    float lon_err;
    float alt_err;
    float diff_age;
    float undulation;
    uint32_t sats_tracked;
    uint32_t sats_used;
    int8_t heading_stat;
    float heading;
    float pitch;
    float heading_err;
    float pitch_err;
    float baselineLen;
    int8_t velocity_stat;
    float speed_north;
    float speed_east;
    float speed_up;
    float latency;
} msg_rtk_gps_data_1_0;

#endif
