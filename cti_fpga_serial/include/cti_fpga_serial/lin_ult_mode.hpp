#ifndef LIN_ULT_MODE_H
#define LIN_ULT_MODE_H
#include <inttypes.h>    //or stdint.h uint16_t uint32_t
#include "lin_ult_mode.hpp"
#include "user_cmd.hpp"

//--需要构建控制指令的顺序和lin通讯的超声波序号之间的对应表 在cpp文件中构建（-1,代表不存在）
//int8_t linult_order_to_control[] = {2, -1, 1, 11, -1, 7, 6, -1, 10, 9, 4, 3, 8, 5};

//--ult type enum  此处说明，旧的电应普的超声波的控制顺序是使用的下面的顺序，为了和其进行兼容。LIN同新超声波的控制在原来的电应普控制的基础上进行增加。
//关闭前方所有超声波  bin:  0001 1111 1111 1111 1000  dec: 131064  hex: 0x0001fff8
//关闭左方所有超声波  bin:  0001 1110 1100 1111 1111  dec: 126207  hex: 0x0001ecff
//关闭右方所有超声波  bin:  0001 1101 0011 1111 1111  dec: 119807  hex: 0x0001d3ff
//关闭后方所有超声波  bin:  0001 1111 1111 0000 1111  dec: 7951    hex: 0x00001f0f
//关闭吸尘箱超声波    bin:  0000 0011 1111 1111 1111  dec: 16383   hex: 0x00003fff

//关闭所有超声波：    bin:  0000 0000 0000 0000 0000  dec: 0       hex: 0x0000
//所有超声波打开：    bin:  0001 1111 1111 1111 1111  dec: 131071  hex: 0x0001ffff
typedef enum lin_ult_type_ctrl_enum{
    lin_front_right_ult,      //0 
    lin_front_center_ult,     //1  
    lin_front_left_ult,       //2  
    lin_back_center_ult,      //3
    lin_left_rear_ult,        //4
    lin_rear_left_ult,        //5
    lin_rear_right_ult,       //6
    lin_right_rear_ult,       //7
    lin_left_front_up_ult,    //8
    lin_left_front_down_ult,  //9
    lin_right_front_up_ult,   //10
    lin_right_front_down_ult, //11
    lin_left_center_ult,      //12
    lin_right_center_ult,     //13  
    lin_dustbox_rear_right,   //14  
    lin_dustbox_rear_left,    //15    
    lin_dustbox_bottom,       //16   
    lin_max_type_ult          //17  
}lin_ult_type_ctrl_t;

//模式设置以及回复，运控最多支持接4块超声波板子 每个板子最多接8个探头 每个探头有3种工作模式 （用两个位来表示）， 因此使用 uint64_t 类型数据来表示
//其含义为 0-15bits：第一块板子  16-31bits：第二块板子  32-47bits:第三块板子  48-63bits:第四块板子
typedef struct{
    union{
        uint64_t data;
        struct
        {
            //board_1
            uint8_t lin_front_left:2;    //前左   0: 休眠 1：只收不发 2：又收又发 
            uint8_t lin_front_right:2;   //前右 
            uint8_t lin_back_center:2;   //背部中间 
            uint8_t lin_rear_left:2;     //后左 
            uint8_t lin_rear_right:2;    //后右 
            uint16_t unused1:6;
            //board_2
            uint8_t lin_left_front_up:2;     //左前上  0: 休眠 1：只收不发 2：又收又发 
            uint8_t lin_left_front_down:2;   //左前下
            uint8_t lin_left_center:2;       //左中 
            uint8_t lin_right_center:2;      //右中
            uint8_t lin_right_front_down:2;  //右前下
            uint8_t lin_right_front_up:2;    //右前下
            uint16_t unused2:4;
            //board_3
            uint8_t lin_dustbox_rear_right:2;     //吸尘箱后右
            uint8_t lin_dustbox_rear_left:2;      //吸尘箱后左
            uint8_t lin_dustbox_bottom:2;         //吸尘箱底部
            uint16_t unused3:10;
            //board_4
            uint16_t unused4:16;
        }bits;
    };
}lin_ult_mode_union;

 //--查询lin通讯超声波工作模式的命令结构体
typedef struct __attribute__((packed)) 
{
    uint16_t src;   //起始地址  2：工控机  50: 车身超声波板 6：吸尘箱超声波板
    uint16_t dest;  //目标地址  2：工控机  50: 车身超声波板 6：吸尘箱超声波板
    uint64_t check_ult_Serial; //要查询的超声波的序号，目前不启用，发任意数据均回复所有探头的工作模式
    uint8_t unused;
}check_lin_ult_mode_type;

 //--设置lin通讯超声波工作模式的命令结构体
typedef struct __attribute__((packed)) 
{
    uint16_t src;  //2：工控机  50: 车身超声波板 6：吸尘箱超声波板
    uint16_t dest; //2：工控机  50: 车身超声波板 6：吸尘箱超声波板
    lin_ult_mode_union mode; //设定工作模式
    uint8_t unused;
}set_lin_ult_mode_type;

typedef struct __attribute__((packed)) 
{
    uint16_t src;   //2：工控机  50: 车身超声波板 6：吸尘箱超声波板
    uint16_t dest;  //2：工控机  50: 车身超声波板 6：吸尘箱超声波板
    lin_ult_mode_union mode; //回复工作模式
    uint8_t unused;
}report_lin_ult_mode_type;

//---------------------------  超声波模式控制状态
typedef struct ultmode_set_state
{
    int8_t state; //设置状态 127:初始化 0：空闲 1：正在查询 2：等待查询结果 3：正在设置 4：等待设置结果  -1：查询超时 -2：设置超时 -3:设置失败
    uint64_t set_mode; //需要设置的mode
    double   set_start_time; //设置开始的时间
    uint64_t recv_mode; //接收到的mode
    double   check_start_time;//查询开始的时间
    ultmode_set_state()
    {
        state = 127;
        set_mode = 0;
        set_start_time = 0.0f;
        recv_mode = 0;
        check_start_time = 0.0f;
    }
    void reset(){
        state = 0;
        set_mode = 0;
        set_start_time = 0.0f;
        recv_mode = 0;
        check_start_time = 0.0f;
    }
}ultmode_set_state;


#endif
