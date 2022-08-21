#include <cti_fpga_data/cti_fpga_data.h>
#include "cti_msgs/BoxOperationService.h"
CtiFpgaData::CtiFpgaData():pnh("~")
{
    boxloadstatus_old = 9;
    RFIDTIMEOUT_DUR = 6;
    boxloadid_old = "";
    rfid4th_old = "";
    rfid4thid_old = "";
    rfid1_empty_cnt = 0;
    rfid2_empty_cnt = 0;
    rfid3_empty_cnt = 0;
    rfid4_empty_cnt = 0;
    RFID_EMPTY_LIMIT = 5; //rfid为空的滤波
    RFIDTIMEOUT_DUR = 6;  //rfid接收超时时间
    rfid2_read_lock = 0; 
    cti_msgs::TabState tabstate;
    tabstate_rfid1.push_back(tabstate);
    tabstate_rfid2.push_back(tabstate);
    tabstate_rfid3.push_back(tabstate);
    tabstate_rfid4.push_back(tabstate);
    boxload_sub = nh.subscribe("/box_assemble/singleload_state",5,&CtiFpgaData::boxload_Callback, this);
    //rfid_sub = nh.subscribe("/cti/fpga_serial/rfid_single",5,&CtiFpgaData::rfidsingle_Callback,this);//放弃使用rfid来判断是否有箱子,改用平台信息.
    chassis_error_sub = nh.subscribe("/cti/chassis_serial/chassis_error",5,&CtiFpgaData::chassiserror_Callback,this);//嵌入式错误信息接收
    navigation_log_sub = nh.subscribe("/cti/fpga_serial/navigation_log",5,&CtiFpgaData::navigationlog_Callback,this);//嵌入式导航信息log
    chassis_info_sub = nh.subscribe("/cti/chassis_serial/chassis_info",5,&CtiFpgaData::chassisInfo_Callback,this);//嵌入式运动信息log
    box_info_sub = nh.subscribe("/cloud_scheduling_node/response/ask/boxinfo",1,&CtiFpgaData::boxinfo_Callback,this);//箱子信息接收

    rfid_pub = nh.advertise<cti_msgs::BoxState>("/cti/fpga_serial/rfid_data_all",10); //所有rfid信息发布
    box_exist_pub = nh.advertise<std_msgs::UInt8>("/cti/fpga_serial/box_status",1);//箱子状态发布

    std::string filelog;
    if(nh.getParam("CTI_RUN_LOG_PATH", filelog))
    {
        filelog += "/chassis_data.log";
    }
    else
    {
        pnh.param<std::string>("filenamelog", filelog, "/opt/fpga_data.log");
    }
    std::cout << "filenamelog:" << filelog << std::endl;
    initLog(filelog);

    node_status_publisher_ptr_ = std::make_shared<cti_monitor::NodeStatusPublisher>(nh,pnh);//节点状态监控
    node_status_publisher_ptr_->ENABLE();
    node_status_publisher_ptr_->NODE_ACTIVATE();
    
    //超声波数据处理
    lin_ult_data_sub = nh.subscribe("/cti/chassis_serial/lin_ult_data",1,&CtiFpgaData::lin_ult_data_Callback,this);
    //发布和旧超声波相同的话题出去,用来检测超声波的数据
    pub_all =  nh.advertise<std_msgs::Float32MultiArray>("/cti/ultrasonic/data_all",2);
    pub_all_filtered =  nh.advertise<std_msgs::Float32MultiArray>("/cti/ultrasonic/data_all_filtered",2);
    pub_0 =  nh.advertise<std_msgs::Float32MultiArray>("/cti/ultrasonic/data_0",2);
    pub_1 =  nh.advertise<std_msgs::Float32MultiArray>("/cti/ultrasonic/data_1",2);
    pub_2 =  nh.advertise<std_msgs::Float32MultiArray>("/cti/ultrasonic/data_2",2);
    pub_3 =  nh.advertise<std_msgs::Float32MultiArray>("/cti/ultrasonic/data_3",2);
    //数据发布
    for(int i = 0; i < max_type_ult; i++){
        lin_ult_pub[i] = nh.advertise<sensor_msgs::Range>("/cti/ultrasonic/" + ult_name[i],1);
    }
    lin_ult_front_center_pub = nh.advertise<sensor_msgs::Range>("/cti/ultrasonic/front_center_ult",1);
    lin_ult_rear_center_pub = nh.advertise<sensor_msgs::Range>("/cti/ultrasonic/rear_center_ult",1);
    ult_topic_names_pub =  nh.advertise<std_msgs::String>("/cti/ultrasonic/topic_names",1,true);
    pnh.param<double>("ult_min_range",min_range, 0.25);
    pnh.param<double>("ult_max_range",max_range, 1.5);
    pnh.param<double>("rear_ult_detect_offset",rear_ult_detect_offset, 0.05);//单位 m
    pnh.param<double>("rear_ult_detect_max",rear_ult_detect_max, 2.0);//单位 m
    pnh.param<double>("rear_ult_detect_min",rear_ult_detect_min, 2.0);//单位 m
    pnh.param<double>("front_ult_detect_offset",front_ult_detect_offset, 0.05);//单位 m
    pnh.param<double>("front_ult_detect_max",front_ult_detect_max, 2.0);//单位 m
    pnh.param<double>("front_ult_detect_min",front_ult_detect_min, 2.0);//单位 m
    pnh.param<double>("vehicle_width",VEHICLE_WIDTH, 900);//单位 mm
    pnh.param<double>("rear_ult_dist",REAR_ULT_DIST, 650);//单位 mm
    pnh.param<double>("front_ult_dist",FRONT_ULT_DIST, 600);//单位 mm
    pnh.param<double>("msg_max_range",msg_max_range, 1.5);//单位 m
    pnh.param<int>("vehicle_LIN_ult_pub", vehicle_LIN_ult_pub, 0);
    pnh.param<int>("vehicle_rear_LIN_ult_pub", vehicle_rear_LIN_ult_pub, 0);
    pnh.param<int>("front_center_buffer_max", front_center_buffer_max, 3);
    pnh.param<int>("rear_center_buffer_max", rear_center_buffer_max, 3);
    pnh.param<double>("rear_ground_filter_dist",rear_ground_filter_dist, 0.3);//单位 m
    timer_ = nh.createTimer(1.0, &CtiFpgaData::timerCallback,this);

    //吸尘箱超声波数据处理
    pnh.param<int>("dustbox_LIN_ult_pub", dustbox_LIN_ult_pub, 0);
    dustbox_lin_ult_data_sub = nh.subscribe("/cti/chassis_serial/sanitation_dustbox_ult",1,&CtiFpgaData::dustbox_lin_ult_data_Callback,this);
    dustbox_ult_rear_center_pub =  nh.advertise<sensor_msgs::Range>("/cti/chassis_serial/dustbox_rear_center",1);
    dustbox_ult_bottom_pub =  nh.advertise<sensor_msgs::Range>("/cti/chassis_serial/dustbox_bottom",1);
    pub_dustbox_all_before_kalman = nh.advertise<std_msgs::Float32MultiArray>("/cti/ultrasonic/data_all_before_kalman_dustbox",2);
    pub_dustbox_all_filtered = nh.advertise<std_msgs::Float32MultiArray>("/cti/ultrasonic/data_all_after_kalman_dustbox",2);

    //kalman 滤波器初始化
    pnh.param<bool>("kalman_enable",kalman_enable_, false);//单位 m
    ROS_INFO("kalman_enable: %s",kalman_enable_?"true":"false");
    pnh.param<double>("kalman_R",kalman_R_, 0.01);//单位 m
    ROS_INFO("kalman_R: %f",kalman_R_);
    pnh.param<double>("kalman_Q",kalman_Q_, 0.01);//单位 m
    ROS_INFO("kalman_Q: %f",kalman_Q_);
    pub_all_before_kalman = nh.advertise<std_msgs::Float32MultiArray>("/cti/ultrasonic/data_all_before_kalman",2);
    kalman_init();
}
/**********************************
*kalman初始化
***********************************/
void CtiFpgaData::kalman_init(){
    Kalman KF;
    for(int i = 0; i < max_type_ult; i++){
        KF.Q = kalman_Q_;         //过程噪声可以认为是0
        KF.R = kalman_R_; //使用参数进行调节,测量噪声
        KF.Kg = 0;
        KF.lastP = 1;     //上一次的协方差，初始值可以为10,不可以为0
        KF.x_hat = 0;
        ult_Kalman_list_.push_back(KF);
    }
    for(int i = 0; i < dustbox_max_type_ult; i++){
        KF.Q = kalman_Q_;         //过程噪声可以认为是0
        KF.R = kalman_R_; //使用参数进行调节,测量噪声
        KF.Kg = 0;
        KF.lastP = 1;     //上一次的协方差，初始值可以为10,不可以为0
        KF.x_hat = 0;
        dustbox_ult_Kalman_list_.push_back(KF);
    }
    
}
/**********************************
*kalman滤波
***********************************/
void CtiFpgaData::kalman_filter(uint8_t ult_type,uint16_t ult_data[] , int len){
    float data_filtered = 0, x_t;
    float data_orig;
    Kalman* KF_temp;
    // for(int i = 0; i < len; i++){
    //     printf("ult_data[%d]: %d \n",i,ult_data[i]);
    // }
    if(ult_type == ult_vehicle){
        for(int i = 0; i < len; i++){
            KF_temp = &ult_Kalman_list_[i];
            data_orig = (float)ult_data[i];
            x_t = KF_temp->x_hat;
            // if(5 == i){
            //     printf("--------------------\n");
            // }
            // if(5 == i){
            //     printf("x_t: %f\n",x_t);
            // }
            KF_temp->nowP = KF_temp->lastP + KF_temp->Q;
            // if(5 == i){
            //     printf("nowP: %f,lastP: %f,Q:%f\n",KF_temp->nowP,KF_temp->lastP,KF_temp->Q);
            // }
            KF_temp->Kg = KF_temp->nowP / (KF_temp->nowP + KF_temp->R);
            // if(5 == i){
            //     printf("Kg: %f,data_orig: %f,x_t:%f\n",KF_temp->Kg,data_orig,x_t);
            // }
            data_filtered = x_t + KF_temp->Kg*(data_orig - x_t);
            KF_temp->x_hat = data_filtered;
            KF_temp->lastP = (1 - KF_temp->Kg) * KF_temp->nowP;
            // if(5 == i){
            //     printf("ult_data[%d]: %d,data_orig:%f,data_filtered:%f\n",i,ult_data[i],data_orig,data_filtered);
            // }
            ult_data[i] = (uint16_t)data_filtered;
        }
    }
    if(ult_type == ult_dustbox){
        for(int i = 0; i < len; i++){
            KF_temp = &dustbox_ult_Kalman_list_[i];
            data_orig = (float)ult_data[i];
            x_t = KF_temp->x_hat;
            // if(5 == i){
            //     printf("--------------------\n");
            // }
            // if(5 == i){
            //     printf("x_t: %f\n",x_t);
            // }
            KF_temp->nowP = KF_temp->lastP + KF_temp->Q;
            // if(5 == i){
            //     printf("nowP: %f,lastP: %f,Q:%f\n",KF_temp->nowP,KF_temp->lastP,KF_temp->Q);
            // }
            KF_temp->Kg = KF_temp->nowP / (KF_temp->nowP + KF_temp->R);
            // if(5 == i){
            //     printf("Kg: %f,data_orig: %f,x_t:%f\n",KF_temp->Kg,data_orig,x_t);
            // }
            data_filtered = x_t + KF_temp->Kg*(data_orig - x_t);
            KF_temp->x_hat = data_filtered;
            KF_temp->lastP = (1 - KF_temp->Kg) * KF_temp->nowP;
            // if(5 == i){
            //     printf("ult_data[%d]: %d,data_orig:%f,data_filtered:%f\n",i,ult_data[i],data_orig,data_filtered);
            // }
            ult_data[i] = (uint16_t)data_filtered;
        }
    }

}


void CtiFpgaData::initLog(const std::string name)
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

/**********************************
*箱子信息回调函数。 消息类型:cti_rblite_msgs/BoxAskResponse
***********************************/
void CtiFpgaData::boxinfo_Callback(const cti_rblite_msgs::BoxAskResponse::ConstPtr &msg)
{
    if(msg->infos.empty()){
        box_exist.data = 0;
        box_exist_pub.publish(box_exist);
    }else{
        for(int i = 0; i< msg->infos.size();i++){
            if(msg->infos[i].hive_type == cti_rblite_msgs::BoxInfo::HIVE_TYPE_DEFAULT)
            {
                box_exist.data = 1;
                box_exist_pub.publish(box_exist);
                return;
            }else{
                //Info("box_status: -1 " << "hive_type: " <<  msg->infos[i].hive_type << "hive_name" << msg->infos[i].name);
            }
        }
        box_exist.data = 0;
        box_exist_pub.publish(box_exist);

    }   
    Info("CB_BE: "<<(int)box_exist.data)
}
/**********************************
*底盘错误回调函数。 
***********************************/

void CtiFpgaData::chassiserror_Callback(const std_msgs::UInt32MultiArray::ConstPtr &msg)
{
    switch (msg->data[0])
    {
        case motion_control_board:
        Info(" ERR_MC: " << (int)msg->data[1]);
        break;
        case battery_board:
        Info(" ERR_BAT: " << (int)msg->data[1]);
        break;
        case left_front_driver:
        Info(" ERR_LFD: " << (int)msg->data[1]);
        break;
        case right_front_driver:
        Info(" ERR_RFD: " << (int)msg->data[1]);
        break;
        case left_rear_driver:
        Info(" ERR_LRD: " << (int)msg->data[1]);
        break;
        case right_rear_driver:
        Info(" ERR_RRD: " << (int)msg->data[1]);
        break;
        case front_turn_driver:
        Info(" ERR_FTD:: " << (int)msg->data[1]);
        break;
        case rear_turn_driver:
        Info(" ERR_RTD: " << (int)msg->data[1]);
        break;
        case front_brake_driver:
        Info(" ERR_RBD: " << (int)msg->data[1]);
        break;
        case rear_brake_driver:
        Info(" ERR_RBD: " << (int)msg->data[1]);
        break;
        default:
        break;
    }
}
/**********************************
*底盘重要信息log回调函数。 消息类型:cti_fpga_serial/navigationlog
***********************************/

void CtiFpgaData::navigationlog_Callback(const cti_fpga_serial::navigationlog::ConstPtr &msg)
{
    node_status_publisher_ptr_->CHECK_MAX_VALUE("/value/cti_fpga_data/recv_navigationlog/liner_speed",msg->liner_speed,4.5,5,"value recv_navigationlog:liner_speed is too high");
    Info("NS_LS: " << msg->liner_speed << " NS_TA: " << msg->turn_angle
    << " NS_BT: " << (int)msg->break_torque << " NS_EF: " <<(int) msg->enable_flag
    << " NS_VX: " << msg->actual_body_linear_vel_x << " NS_LFV: " <<msg->actual_speed_base_on_left_front_wheel
    << " NS_RFV: " << msg->actual_speed_base_on_right_front_wheel 
    << " NS_LRV: " << msg->actual_speed_base_on_left_rear_wheel
    << " NS_RRV: " << msg->actual_speed_base_on_right_rear_wheel << " NS_FTA: " << msg->actual_turn_front_angle
    << " NS_RTA: "<<msg->actual_turn_rear_angle
    << " NS_ST: "<<msg->set_torque[0]<<" "<<msg->set_torque[1]<<" "<<msg->set_torque[2]<<" "<<msg->set_torque[3]<<" "
    <<msg->set_torque[4]<<" "<<msg->set_torque[5]<<" "<<msg->set_torque[6]<<" "<<msg->set_torque[7]<<" "
    << " NS_NE: "<<msg->now_encoder[0]<<" "<<msg->now_encoder[1]<<" "<<msg->now_encoder[2]<<" "<<msg->now_encoder[3]<<" "
    <<msg->now_encoder[4]<<" "<<msg->now_encoder[5]<<" "<<msg->now_encoder[6]<<" "<<msg->now_encoder[7]);  
}

/**********************************
*底盘信息log回调函数。 消息类型:cti_fpga_serial/vehicelstate
***********************************/
void CtiFpgaData::chassisInfo_Callback(const cti_fpga_serial::vehiclestate::ConstPtr &msg)
{
    node_status_publisher_ptr_->CHECK_MAX_VALUE("/value/cti_fpga_data/recv_chassisInfo/vx",msg->vel_liner_x,4.5,5,"value chassisInfo:vx is too high");
    Info("CHASSIS_INFO_LOG: "
    <<" R_PI: "<<(int)msg->portIndex
    <<" R_SV: "<<(int)msg->state_vehicle
    <<" R_DE: "<<(int)msg->drivers_enable
    <<" R_CM: "<<(int)msg->control_mode
    <<" R_BK: "<<(int)msg->state_brake
    <<" R_VX: "<<msg->vel_liner_x
    <<" R_VY: "<<msg->vel_liner_y
    <<" R_VA: "<<msg->vel_angular
    <<" R_AFT: "<<msg->angle_front_turn
    <<" R_ART: "<<msg->angle_rear_turn
    <<" R_SW: "<<(int)msg->sw_status_data
    <<" R_OD: "<<msg->odometer
    <<" R_PX: "<<msg->pos_x
    <<" R_PY: "<<msg->pos_y
    <<" R_YAW: "<<msg->angle_yaw_radian
    <<" R_LP: "<<(int)msg->lift_position
    <<" R_ACC: "<<msg->acc[0]<<" "<<msg->acc[1]<<" "<<msg->acc[2]
    <<" R_GY: "<<msg->gyro[0]<<" "<<msg->gyro[1]<<" "<<msg->gyro[2]
    <<" R_Q: "<<msg->q[0]<<" "<<msg->q[1]<<" "<<msg->q[2]<<" "<<msg->q[3]
    <<" recv_baro_raw: "<<msg->baro_raw
    <<" recv_baro_height: "<<msg->baro_height
    <<" R_CP1: "<<msg->compass1_str[0]<<" "<<msg->compass1_str[1]<<" "<<msg->compass1_str[2]
    <<" R_CP2: "<<msg->compass2_str[0]<<" "<<msg->compass2_str[1]<<" "<<msg->compass2_str[2]
    <<" R_BL: "<<(int)msg->laser_id[0]
    );
}
/*msg->code 为状态： 0 空闲，1 装箱中，2 装箱完成，-1 装箱失败。
*msg->name 命令的id,一次装箱过程中的所有命令ID 一致。
***********************************/

void CtiFpgaData::boxload_Callback(const cti_msgs::State::ConstPtr &msg)
{
    if(msg->id == cti_msgs::TargetPose::UNLOAD)
    {
        rfid2_read_lock = 0;
        //Info("UNLOAD: 3 "<<"id:"<<msg->name);
    }
    if (this->boxloadstatus_old == msg->code && this->boxloadid_old == msg->name)
    {
        return;
    }
    else
    {
        boxloadstatus_old = msg->code;
        boxloadid_old = msg->name;
    }
    switch (msg->code)
    {
        case cti_msgs::BoxOperationServiceResponse::UNPREPARE:
            break;
        case cti_msgs::BoxOperationServiceResponse::ERROR:
            rfid4th_old = "";
            rfid4thid_old = "";
            break;
        case cti_msgs::BoxOperationServiceResponse::PREPARE:
            if(tabstate_rfid1[0].message == "" && tabstate_rfid2[0].message ==  "" &&
            tabstate_rfid3[0].message == "" && tabstate_rfid4[0].message != "")
            {
                rfid4th_old = tabstate_rfid4[0].message;
                rfid4thid_old = msg->name;
            }
            break;
        case  cti_msgs::BoxOperationServiceResponse::COMPLETED:
            if(rfid4thid_old == msg->name && tabstate_rfid4[0].message != rfid4th_old && 
            tabstate_rfid2[0].message == "" && tabstate_rfid3[0].message == "" && tabstate_rfid1[0].message == "")
            {
                tabstate_rfid2[0].message = rfid4th_old;
                tabstate_rfid3[0].message = rfid4th_old;
                tabstate_rfid2[0].status = 200;//200:第二个箱子没有读到，强制赋值
                tabstate_rfid3[0].status = 200;
                rfid4th_old = "";
                rfid4thid_old = "";
		        rfid2_read_lock = 1;
            }
            else if(rfid4thid_old != msg->name)
            {
                rfid4th_old = "";
                rfid4thid_old = "";
		        rfid2_read_lock = 0;
            }
            break;
    }          
}
void CtiFpgaData::rfidsingle_Callback(const cti_msgs::BoxState::ConstPtr &msg)
{
    cti_msgs::TabState tabstate;
    tabstate = msg->states[0];
    switch (msg->states[0].id)
    {
    case 1:
        if (tabstate.message == "")
        {
            rfid1_empty_cnt++;
            if(rfid1_empty_cnt >= RFID_EMPTY_LIMIT)
            {
                tabstate_rfid1.clear();
                tabstate_rfid1.push_back(tabstate);
                rfid1_empty_cnt = 0;
            }      
        }
        else
        {
        tabstate_rfid1.clear();
        tabstate_rfid1.push_back(tabstate);
        rfid1_empty_cnt = 0;
        rfid2_read_lock = 0;
        }
        break;
    case 2:
        if(rfid2_read_lock == 0)
        {
        tabstate_rfid2.clear();
        tabstate_rfid2.push_back(tabstate);
        }
        break;
    case 3:
        if(rfid2_read_lock == 0)
        {
        tabstate_rfid3.clear();
        tabstate.id = 2;
        tabstate_rfid3.push_back(tabstate);
        //rfid3_empty_cnt = 0;
        }
        break;
    case 4:
        if (tabstate.message == "")
        {
            rfid4_empty_cnt++;
            if(rfid4_empty_cnt >= RFID_EMPTY_LIMIT)
            {
                tabstate_rfid4.clear();
                tabstate.id = 3;
                tabstate_rfid4.push_back(tabstate);
                rfid4_empty_cnt = 0;
            }      
        }
        else
        {
        tabstate_rfid4.clear();
        tabstate.id = 3;
        tabstate_rfid4.push_back(tabstate);
        rfid4_empty_cnt = 0;
        }
        break;
    default:
        break;    
    }
     switch(msg->states[0].id)
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
		tabstate_rfid1.clear();
        cti_msgs::TabState tabstate;
		tabstate.id = 1;
        tabstate.status = 140;//140 means recv timeout!
		tabstate.message = "";
        tabstate_rfid1.push_back(tabstate);
		oldtime_rfid1 = ros::Time::now().toSec();
	}
	else
	{
		recv_rfid1_timeout = false;
	}
    if (RFIDTIMEOUT_DUR < (nowtime_rfid - oldtime_rfid2))
    {
		recv_rfid2_timeout = true;                
		tabstate_rfid2.clear();
        cti_msgs::TabState tabstate;
		tabstate.id = 2;
        tabstate.status = 140;//140 means recv timeout!
		tabstate.message = "";
        tabstate_rfid2.push_back(tabstate);
        oldtime_rfid2 = ros::Time::now().toSec();
	} 
	else
	{
		recv_rfid2_timeout = false;
	}   
    if (RFIDTIMEOUT_DUR < (nowtime_rfid - oldtime_rfid3))
    {
        recv_rfid3_timeout = true; 
		tabstate_rfid3.clear();
        cti_msgs::TabState tabstate;
		//tabstate.id = 3;
        tabstate.id = 2;
        tabstate.status = 140;//140 means recv timeout!
		tabstate.message = "";
        tabstate_rfid3.push_back(tabstate);
        oldtime_rfid3 = ros::Time::now().toSec();
	}  
	else
	{
		recv_rfid3_timeout = false;
	}
    if (RFIDTIMEOUT_DUR < (nowtime_rfid - oldtime_rfid4))
    {
        recv_rfid4_timeout = true; 
		tabstate_rfid4.clear();
        cti_msgs::TabState tabstate;
		//tabstate.id = 4;
        tabstate.id = 3;
        tabstate.status = 140;//140 means recv timeout!
		tabstate.message = "";
        tabstate_rfid4.push_back(tabstate);
        oldtime_rfid4 = ros::Time::now().toSec();
        //Info("RECV RFID: module_id_4 recv timeout!");
	}  
	else
	{
		recv_rfid4_timeout = false;
	}      
    if(4 == msg->states[0].id || recv_rfid4_timeout == true)
    {
        rfid_all.states.clear();
        rfid_all.header.stamp = ros::Time::now();
        rfid_all.header.frame_id = "/fpga_data/rfid";
        rfid_all.states.insert(rfid_all.states.end(),tabstate_rfid1.begin(),tabstate_rfid1.end());
        rfid_all.states.insert(rfid_all.states.end(),tabstate_rfid2.begin(),tabstate_rfid2.end());
        rfid_all.states.insert(rfid_all.states.end(),tabstate_rfid3.begin(),tabstate_rfid3.end()); 
        rfid_all.states.insert(rfid_all.states.end(),tabstate_rfid4.begin(),tabstate_rfid4.end()); 
        rfid_pub.publish(rfid_all);
        if((tabstate_rfid1[0].message != "") || (tabstate_rfid2[0].message != "") || (tabstate_rfid3[0].message != "") || (tabstate_rfid4[0].message != ""))
        {
            box_exist.data = 1;
            box_exist_pub.publish(box_exist);
        }
        else
        {
            box_exist.data = 0;
            box_exist_pub.publish(box_exist);
        }
        
    }      
}

//----------lin 通信 超声波数据处理
void CtiFpgaData::lin_ult_data_Callback(const std_msgs::UInt16MultiArray::ConstPtr &msg){
    uint16_t ult_data[max_type_ult];
    for(int i = 0;i < max_type_ult;i++){
        ult_data[i] = (float)msg->data[i];
    }
   
    //发布源数据用于观测,和数据的监测(为了和旧超声波保持一致,需要发data0,data_1,data_2.这里三个数据发的一样的)
    std_msgs::Float32MultiArray dataArray;
    for(int i = 0; i < max_type_ult; i++){
        dataArray.data.push_back(ult_data[i] / 1000.0f);
    }
    pub_all.publish(dataArray);

    //发布数据用于webtools网页显示
    //pub_0:  ult_msg_0.data[0]:前右; 
    //        ult_msg_0.data[1]:前中;
    //        ult_msg_0.data[2]:前左; 
    std_msgs::Float32MultiArray ult_msg_0;
    ult_msg_0.data.push_back(ult_data[1] / 1000.0f);
    ult_msg_0.data.push_back(-1.0);   //-1:代表此探头未安装
    ult_msg_0.data.push_back(ult_data[0] / 1000.0f);
    pub_0.publish(ult_msg_0);
    //pub_1:  ult_msg_1.data[0]:后左左; 
    //        ult_msg_1.data[1]:后左;  
    //        ult_msg_1.data[2]:后右;
    //        ult_msg_1.data[3]:后右右;
    // ---------------------车尾---------------------------//
    //   超声波：   O         O            O          O     //
    //          后左左      后左          后右       后右右   //
    // ----------------------------------------------------//
    std_msgs::Float32MultiArray ult_msg_1;
    ult_msg_1.data.push_back(-1.0);
    ult_msg_1.data.push_back(ult_data[6] / 1000.0f);   
    ult_msg_1.data.push_back(ult_data[5] / 1000.0f);
    ult_msg_1.data.push_back(-1.0);
    pub_1.publish(ult_msg_1);
    //pub_2:  ult_msg_2.data[0]:前左上; 
    //        ult_msg_2.data[1]:前左下;  
    //        ult_msg_2.data[2]:前右上; 
    //        ult_msg_2.data[3]:前右下;
    std_msgs::Float32MultiArray ult_msg_2;
    ult_msg_2.data.push_back(ult_data[9] / 1000.0f);
    ult_msg_2.data.push_back(ult_data[8] / 1000.0f);   
    ult_msg_2.data.push_back(ult_data[3] / 1000.0f);
    ult_msg_2.data.push_back(ult_data[2] / 1000.0f);
    pub_2.publish(ult_msg_2);
    //pub_3:  ult_msg_3.data[0]:左中; 
    //        ult_msg_3.data[1]:右中;  
    std_msgs::Float32MultiArray ult_msg_3;
    ult_msg_3.data.push_back(ult_data[7] / 1000.0f);   
    ult_msg_3.data.push_back(ult_data[4] / 1000.0f);
    pub_3.publish(ult_msg_3);

    //数据处理--
    //将大于最大值和最小值的数据进行收敛
    for(int i = 0;i < max_type_ult;i++){
        if(ult_data[i] < min_range * 1000.f){
            ult_data[i] = min_range * 1000.f;
        }
        if(ult_data[i] > max_range * 1000.f){
            ult_data[i] = max_range * 1000.f;
        }
    }
    
    //发现后右有一个固定噪点348或359,进行过滤，疑似超声波装的太紧，在只接收的工作模式下会出现。或者是打到车身某个地方
    if(ult_data[rear_right_ult] ==  348 || ult_data[rear_right_ult] ==  359){
       ult_data[rear_right_ult] = max_range * 1000.f;
    }

    //障碍过滤
    //1.将数据放入各自的状态队列
    for(int i = 0;i < max_type_ult;i++){
        ult_state[i].data_buffer.push_back(ult_data[i]);
        while(ult_state[i].data_buffer.size() > MAX_FILTER_NUM){
            ult_state[i].data_buffer.pop_front();
        }
    }
    //2.对数据进行滤波处理，分为两种，如果是特殊情况，1. {max, max, 障碍} 2.{max, 障碍， max}, 使用幅值滤波，直接输出max.如果不是这两种情况使用动态加权平均滤波
    for(size_t ult_num = 0;ult_num < max_type_ult;ult_num++){//外围循环是探头循环
        if(
            (ult_state[ult_num].data_buffer[MAX_FILTER_NUM-3] == max_range * 1000.f && ult_state[ult_num].data_buffer[MAX_FILTER_NUM-2] == max_range * 1000.f &&
             ult_state[ult_num].data_buffer[MAX_FILTER_NUM-1] < max_range * 1000.f) ||
            (ult_state[ult_num].data_buffer[MAX_FILTER_NUM-3] == max_range * 1000.f && ult_state[ult_num].data_buffer[MAX_FILTER_NUM-2] < max_range * 1000.f &&
             ult_state[ult_num].data_buffer[MAX_FILTER_NUM-1] == max_range * 1000.f)
          ){
            //幅值滤波
            ult_state[ult_num].data_output = max_range * 1000.f;
        }else{
            //动态加权递推平均滤波法
            float diff[MAX_FILTER_NUM];//存放计算的差值
            //计算差值
            for(size_t data_num = 1; data_num < MAX_FILTER_NUM; data_num++){
                //这里为了保持插值和对应的测量数据一致， diff从第二个开始赋值，第一个是无效的，下面的权重 wight同理
                diff[data_num] = (float)abs(ult_state[ult_num].data_buffer[data_num] - ult_state[ult_num].data_buffer[data_num-1]);
                if(diff[data_num] == 0){
                    //对于 0 的赋值为1,因为下面计算这个值会作为分母
                    diff[data_num] = 1;
                }
            }
            //计算权重
            for(int diff_num = 1; diff_num < MAX_FILTER_NUM; diff_num++){
                //差值归一化
                diff[diff_num] = 1.f / diff[diff_num]; 
            }
            float diff_sum = 0.f;
            for(int diff_num = 1; diff_num < MAX_FILTER_NUM; diff_num++){
                //计算差值总和
                diff_sum += diff[diff_num];
            }
            for(int diff_num = 1; diff_num < MAX_FILTER_NUM; diff_num++){
                //权重
                ult_state[ult_num].wight[diff_num] = diff[diff_num] / diff_sum;
            }
            //计算输出值
            float data_calc = 0;
            for(int data_num = 1; data_num < MAX_FILTER_NUM; data_num++){
                data_calc += (float)ult_state[ult_num].data_buffer[data_num] * ult_state[ult_num].wight[data_num];
            }
            ult_state[ult_num].data_output = (uint16_t)data_calc;
        }
        ult_data[ult_num] = ult_state[ult_num].data_output;
    }

    
    std_msgs::Float32MultiArray dataArray_before_kalman;
    for(int i = 0 ; i < max_type_ult;i++){
        dataArray_before_kalman.data.push_back(ult_data[i] / 1000.0f);
    }
    pub_all_before_kalman.publish(dataArray_before_kalman);
    //在此处增加卡尔曼滤波
    if(kalman_enable_){
        kalman_filter(ult_vehicle,ult_data, max_type_ult);
    }
        
    std_msgs::Float32MultiArray dataArray_filtered;
    for(int i = 0 ; i < max_type_ult;i++){
        dataArray_filtered.data.push_back(ult_data[i] / 1000.0f);
    }
    pub_all_filtered.publish(dataArray_filtered);

    //数据发布
    sensor_msgs::Range rangeData;
    rangeData.header.stamp = ros::Time::now();
    rangeData.radiation_type = 0;
    rangeData.min_range = min_range;
    rangeData.max_range = msg_max_range;
    rangeData.field_of_view = 24*M_PI/180.0f;
    std::string frame_id_prefix = "clean_";
    if(vehicle_LIN_ult_pub){
        for(int i = 0 ; i < max_type_ult;i++){
            rangeData.range =  ult_data[i] / 1000.f;
            rangeData.header.frame_id = frame_id_prefix + ult_name[i];
            //前面和后面的超声波数据不单独发布,按整合之后发布
            if( i != front_left_ult && i != front_right_ult && i != rear_right_ult && i != rear_left_ult &&
                i != back_center_ult)
                lin_ult_pub[i].publish(rangeData);
        }
    }
    //车背后的超声波数据是直接发出来的，不受参数控制
    rangeData.range =  ult_data[back_center_ult] / 1000.f;
    rangeData.header.frame_id = frame_id_prefix + ult_name[back_center_ult];
    lin_ult_pub[back_center_ult].publish(rangeData);

    //-----------------------------------------------------------------------------------------
    //前方超声波发布,策略：
    /*
    探头2（前右）只收不发，探头 2（前左）又发又收
    当两个探头数据满足 0 < 数据 < front_ult_detect_max 时采用三角形解算 
    此处在判断数据是否为有效数据时，使用的最大距离限制是front_ult_detect_max，是为了对数据在 max_range 的限制下再次进行限制，去除滤波时数据经过计算出现漂移产生的影响。
    其余看情况解析
    */
   if(ult_data[front_right_ult] == min_range*1000.f && ult_data[front_left_ult] == min_range * 1000.f){
       //两个探头都为最小值时输出最小值
       rangeData.range = min_range;
   }
   else if( min_range*1000.f < ult_data[front_right_ult] < front_ult_detect_max*1000.f  && ult_data[front_left_ult] >= front_ult_detect_max * 1000.f){
       //只有一个探头的数据有效时，对数据进行限制,小于front_ult_detect_min 才视为障碍，否则赋最大值
       if(ult_data[front_right_ult] < front_ult_detect_min * 1000.f ){
           rangeData.range = ult_data[front_right_ult] / 1000.f;
       }
       else{
           rangeData.range = front_ult_detect_max;
       }
   }
   else if( ult_data[front_right_ult] >= front_ult_detect_max * 1000.f &&  min_range*1000.f < ult_data[front_left_ult] < front_ult_detect_max*1000.f){
        //只有一个探头的数据有效时，对数据进行限制,小于front_ult_detect_min 才视为障碍
        if(ult_data[front_left_ult] < front_ult_detect_min * 1000.f ){
           rangeData.range = ult_data[front_left_ult] / 1000.f;
       }
       else{
           rangeData.range = front_ult_detect_max;
       }
   }
   else if(ult_data[front_right_ult] >= front_ult_detect_max * 1000.f && ult_data[front_left_ult] >= front_ult_detect_max * 1000.f){
       //两个都大于限制时输出限制值
       rangeData.range = front_ult_detect_max;
   }
   else if(min_range*1000.f < ult_data[front_right_ult] < front_ult_detect_max*1000.f && min_range*1000.f < ult_data[front_left_ult] < front_ult_detect_max*1000.f){
       //三角形解算
       //首先判断是否具备组成三角形的条件
       if( (ult_data[front_right_ult] + ult_data[front_left_ult]) > FRONT_ULT_DIST &&
           (ult_data[front_right_ult] + FRONT_ULT_DIST) > ult_data[front_left_ult] &&
           (ult_data[front_left_ult] + FRONT_ULT_DIST) > ult_data[front_right_ult]
        ){
            //前左探头数据和车尾形成的夹角 余弦函数再反余弦求出前左超声波探头所在点那个角的角度
            double angle_frontleft_to_front = acos((float)(FRONT_ULT_DIST * FRONT_ULT_DIST + ult_data[front_left_ult] * ult_data[front_left_ult] - ult_data[front_right_ult] * ult_data[front_right_ult]) / (float)(2 * FRONT_ULT_DIST * ult_data[front_left_ult]));
            //障碍物到左前探头的水平距离
            double dist_H_obs_to_frontleft = fabs(ult_data[front_left_ult] * cos(angle_frontleft_to_front));
            //障碍物到左前探头的垂直
            double dist_V_obs_to_frontleft = fabs(ult_data[front_left_ult] * sin(angle_frontleft_to_front));
            if(angle_frontleft_to_front < PI / 2){
                //如果是锐角，判断障碍点有没有超过右边车身
                if(dist_H_obs_to_frontleft < ( (VEHICLE_WIDTH - FRONT_ULT_DIST) / 2 + FRONT_ULT_DIST + front_ult_detect_offset * 1000.f) ){
                    //如果没有超过车身，输出垂直距离
                    rangeData.range = dist_V_obs_to_frontleft / 1000.f;
                }else{
                    //如果超过车身，输出最大距离
                    rangeData.range = front_ult_detect_max;
                }
            }
            else if(angle_frontleft_to_front ==  PI / 2) {
                //如果是直角，输出垂直距离
                rangeData.range = dist_V_obs_to_frontleft / 1000.f;
            }
            else if(angle_frontleft_to_front > PI /2 ){
                //如果是钝角，判断障碍点有没有超过左边车身
                if(dist_H_obs_to_frontleft < ( (VEHICLE_WIDTH - FRONT_ULT_DIST) / 2 + front_ult_detect_offset * 1000.f) ){
                    //如果没有超过车身，输出垂直距离
                    rangeData.range = dist_V_obs_to_frontleft / 1000.f;
                }else{
                    //如果超过车身，输出最大距离
                    rangeData.range = front_ult_detect_max;
                }
            }
        }
        else{
            //不满足组成三角形的条件，输出两个探头较小的值
             rangeData.range = cmin(ult_data[front_right_ult],ult_data[front_left_ult]) / 1000.f;
        }
   }
   else{
       //其他（应该没有了，以防万一），输出两个探头较小值
       rangeData.range = cmin(ult_data[front_right_ult],ult_data[front_left_ult]) / 1000.f;
   }
   if(vehicle_LIN_ult_pub){
        //在输出之前再次进行过滤 对无障碍值 重复3次才会发布，否则保持上一次的值
        front_center_buffer.push_back(rangeData.range);
        while(front_center_buffer.size() > front_center_buffer_max){
            front_center_buffer.pop_front();
        }
        bool is_all_max = true;
        for(int i = 0; i < front_center_buffer.size();i++){
            is_all_max = is_all_max && (front_center_buffer[i] == front_ult_detect_max);
        }
        if( rangeData.range == front_ult_detect_max){
            if(!is_all_max){
                 rangeData.range =  front_center_pre;
            }
        }
        front_center_pre = rangeData.range;

        rangeData.header.frame_id = frame_id_prefix + "front_center_ult";
        lin_ult_front_center_pub.publish(rangeData);
   }

    //----------------------------------------------------------------------------------------
    //后方超声波发布,策略：
    /*
    探头7（后左）又发又收，探头 6（后右只收不发）
    当两个探头数据满足 0 < 数据 < rear_ult_detect_max 时采用三角形解算 
    此处在判断数据是否为有效数据时，使用的最大距离限制是rear_ult_detect_max，是为了对数据在 max_range 的限制下再次进行限制，去除滤波时数据经过计算出现漂移产生的影响。
    其余看情况解析
    */
 
   if(ult_data[rear_right_ult] == min_range*1000.f && ult_data[rear_left_ult] == min_range * 1000.f){
       //两个探头都为最小值时输出最小值
       rangeData.range = min_range;
   }
   else if( min_range*1000.f < ult_data[rear_right_ult] < rear_ult_detect_max*1000.f  && ult_data[rear_left_ult] >= rear_ult_detect_max * 1000.f){
       //只有一个探头的数据有效时，对数据进行限制,小于rear_ult_detect_min 才视为障碍
       if(ult_data[rear_right_ult] < rear_ult_detect_min * 1000.f ){
           rangeData.range = ult_data[rear_right_ult] / 1000.f;
       }
       else{
           rangeData.range = rear_ult_detect_max;
       }
   }
   else if( ult_data[rear_right_ult] >= rear_ult_detect_max * 1000.f &&  min_range*1000.f < ult_data[rear_left_ult] < rear_ult_detect_max*1000.f){
        //只有一个探头的数据有效时，对数据进行限制,小于rear_ult_detect_min 才视为障碍
        if(ult_data[rear_left_ult] < rear_ult_detect_min * 1000.f ){
           rangeData.range = ult_data[rear_left_ult] / 1000.f;
       }
       else{
           rangeData.range = rear_ult_detect_max;
       }
   }
   else if(ult_data[rear_right_ult] >= rear_ult_detect_max * 1000.f && ult_data[rear_left_ult] >= rear_ult_detect_max * 1000.f){
       //两个都大于限制时输出限制值
       rangeData.range = rear_ult_detect_max;
   }
   else if(min_range*1000.f < ult_data[rear_right_ult] < rear_ult_detect_max*1000.f && min_range*1000.f < ult_data[rear_left_ult] < rear_ult_detect_max*1000.f){
       //三角形解算
       //首先判断是否具备组成三角形的条件
       if( (ult_data[rear_right_ult] + ult_data[rear_left_ult]) > REAR_ULT_DIST &&
           (ult_data[rear_right_ult] + REAR_ULT_DIST) > ult_data[rear_left_ult] &&
           (ult_data[rear_left_ult] + REAR_ULT_DIST) > ult_data[rear_right_ult]
        ){
            //后左探头数据和车尾形成的夹角 余弦函数再反余弦求出后左超声波探头所在点那个角的角度
            double angle_rearleft_to_rear = acos((float)(REAR_ULT_DIST * REAR_ULT_DIST + ult_data[rear_left_ult] * ult_data[rear_left_ult] - ult_data[rear_right_ult] * ult_data[rear_right_ult]) / (float)(2 * REAR_ULT_DIST * ult_data[rear_left_ult]));
            //障碍物到左后探头的水平距离
            double dist_H_obs_to_rearleft = fabs(ult_data[rear_left_ult] * cos(angle_rearleft_to_rear));
            //障碍物到左后探头的垂直
            double dist_V_obs_to_rearleft = fabs(ult_data[rear_left_ult] * sin(angle_rearleft_to_rear));
            if(angle_rearleft_to_rear < PI / 2){
                //如果是锐角，判断障碍点有没有超过右边车身
                if(dist_H_obs_to_rearleft < ( (VEHICLE_WIDTH - REAR_ULT_DIST) / 2 + REAR_ULT_DIST + rear_ult_detect_offset * 1000.f) ){
                    //如果没有超过车身，输出垂直距离
                    rangeData.range = dist_V_obs_to_rearleft / 1000.f;
                }else{
                    //如果超过车身，输出最大距离
                    rangeData.range = rear_ult_detect_max;
                }
            }
            else if(angle_rearleft_to_rear ==  PI / 2) {
                //如果是直角，输出垂直距离
                rangeData.range = dist_V_obs_to_rearleft / 1000.f;
            }
            else if(angle_rearleft_to_rear > PI /2 ){
                //如果是钝角，判断障碍点有没有超过左边车身
                if(dist_H_obs_to_rearleft < ( (VEHICLE_WIDTH - REAR_ULT_DIST) / 2 + rear_ult_detect_offset * 1000.f) ){
                    //如果没有超过车身，输出垂直距离
                    rangeData.range = dist_V_obs_to_rearleft / 1000.f;
                }else{
                    //如果超过车身，输出最大距离
                    rangeData.range = rear_ult_detect_max;
                }
            }
        }
        else{
            //不满足组成三角形的条件，输出两个探头较小的值
             rangeData.range = cmin(ult_data[rear_right_ult],ult_data[rear_left_ult]) / 1000.f;
        }
   }
   else{
       //其他（应该没有了，以防万一），输出两个探头较小值
       rangeData.range = cmin(ult_data[rear_right_ult],ult_data[rear_left_ult]) / 1000.f;
   }

    if(vehicle_LIN_ult_pub && vehicle_rear_LIN_ult_pub){

        //在输出之前再次进行过滤 对无障碍值 重复3次才会发布，否则保持上一次的值
        rear_center_buffer.push_back(rangeData.range);
        while(rear_center_buffer.size() > rear_center_buffer_max){
            rear_center_buffer.pop_front();
        }
        bool is_all_max = true;
        for(int i = 0; i < rear_center_buffer.size();i++){
            is_all_max = is_all_max && (rear_center_buffer[i] == rear_ult_detect_max);
        }
        if( rangeData.range == rear_ult_detect_max){
            if(!is_all_max){
                 rangeData.range =  rear_center_pre;
            }
        }

        //在输出之前再次进行过滤 对小障碍重复3次才会发布，否则保持上一次的值（主要为了过滤地面）
        bool is_all_ground = true;
        for(int i = 0; i < rear_center_buffer.size();i++){
            is_all_ground = is_all_ground && (rear_center_buffer[i] < rear_ground_filter_dist);
        }
        if( rangeData.range  < rear_ground_filter_dist){
            if(!is_all_ground){
                 rangeData.range =  rear_center_pre;
            }
        }

        rear_center_pre = rangeData.range;

        rangeData.header.frame_id = frame_id_prefix + "rear_center_ult";
        lin_ult_rear_center_pub.publish(rangeData);
   }
}

//定时发布超声波话题消息
void CtiFpgaData::timerCallback(const ros::TimerEvent& event){
    //车身超声波话题名称
    std_msgs::String ult_topic_names;
    for(int i = 0; i < max_type_ult; i++){
       std::string ult_topic_name = "/cti/ultrasonic/" + ult_name[i] + ";";
       ult_topic_names.data += ult_topic_name;
    }
    ult_topic_names.data += "/cti/ultrasonic/front_center_ult;";
    ult_topic_names.data += "/cti/ultrasonic/rear_center_ult;";
    ult_topic_names.data += "/cti/ultrasonic/left_rear_ult;";
    ult_topic_names.data += "/cti/ultrasonic/right_rear_ult;";

     //旧的清扫箱（第一代 12个电应普超声波探头）
    ult_topic_names.data += "/cti/chassis_serial/sweeper_left_front;";
    ult_topic_names.data += "/cti/chassis_serial/sweeper_left_up;";
    ult_topic_names.data += "/cti/chassis_serial/sweeper_center_down;";
    ult_topic_names.data += "/cti/chassis_serial/sweeper_right_front;";
    ult_topic_names.data += "/cti/chassis_serial/sweeper_left_center;";
    ult_topic_names.data += "/cti/chassis_serial/sweeper_center_up;";
    ult_topic_names.data += "/cti/chassis_serial/sweeper_right_down;";
    ult_topic_names.data += "/cti/chassis_serial/sweeper_right_center;";
    ult_topic_names.data += "/cti/chassis_serial/sweeper_left_rear;";
    ult_topic_names.data += "/cti/chassis_serial/sweeper_left_down;";
    ult_topic_names.data += "/cti/chassis_serial/sweeper_right_up;";
    ult_topic_names.data += "/cti/chassis_serial/sweeper_right_rear;";
    
    //吸尘箱
    ult_topic_names.data += "/cti/chassis_serial/dustbox_rear_left;";
    ult_topic_names.data += "/cti/chassis_serial/dustbox_rear_center;";
    ult_topic_names.data += "/cti/chassis_serial/dustbox_rear_right";

    ult_topic_names_pub.publish(ult_topic_names);
}

//吸尘箱超声波数据处理
void CtiFpgaData::dustbox_lin_ult_data_Callback(const std_msgs::UInt16MultiArray::ConstPtr &msg){
    if(!dustbox_LIN_ult_pub)
    {
        return;
    }
    if(msg->data[0] !=  0xfefe){  //0xfefe 为箱子lin超声波的标志
        return;
    }
    
    uint16_t dustbox_ult_data[dustbox_max_type_ult];
    for(int i = 0;i < dustbox_max_type_ult;i++){
        dustbox_ult_data[i] = (float)msg->data[i+2];
    }
    //将大于最大值和最小值的数据进行收敛
    for(int i = 0;i < dustbox_max_type_ult;i++){
        if(dustbox_ult_data[i] < min_range * 1000.f){
            dustbox_ult_data[i] = min_range * 1000.f;
        }
        if(dustbox_ult_data[i] > max_range * 1000.f){
            dustbox_ult_data[i] = max_range * 1000.f;
        }
    }
    //障碍过滤
    //1.将数据放入各自的状态队列
    for(int i = 0;i < dustbox_max_type_ult;i++){
        dustbox_ult_state[i].data_buffer.push_back(dustbox_ult_data[i]);
        while(dustbox_ult_state[i].data_buffer.size() > MAX_FILTER_NUM){
            dustbox_ult_state[i].data_buffer.pop_front();
        }
    }
    //2.对数据进行滤波处理，分为两种，如果是特殊情况，1. {max, max, 障碍} 2.{max, 障碍， max}, 使用幅值滤波，直接输出max.如果不是这两种情况使用动态加权平均滤波
    for(size_t ult_num = 0;ult_num < dustbox_max_type_ult;ult_num++){//外围循环是探头循环
        if(
            (dustbox_ult_state[ult_num].data_buffer[MAX_FILTER_NUM-3] == max_range * 1000.f && dustbox_ult_state[ult_num].data_buffer[MAX_FILTER_NUM-2] == max_range * 1000.f &&
             dustbox_ult_state[ult_num].data_buffer[MAX_FILTER_NUM-1] < max_range * 1000.f) ||
            (dustbox_ult_state[ult_num].data_buffer[MAX_FILTER_NUM-3] == max_range * 1000.f && dustbox_ult_state[ult_num].data_buffer[MAX_FILTER_NUM-2] < max_range * 1000.f &&
             dustbox_ult_state[ult_num].data_buffer[MAX_FILTER_NUM-1] == max_range * 1000.f)
          ){
            //幅值滤波
            dustbox_ult_state[ult_num].data_output = max_range * 1000.f;
        }else{
            //动态加权递推平均滤波法
            float diff[MAX_FILTER_NUM];//存放计算的差值
            //计算差值
            for(size_t data_num = 1; data_num < MAX_FILTER_NUM; data_num++){
                //这里为了保持插值和对应的测量数据一致， diff从第二个开始赋值，第一个是无效的，下面的权重 wight同理
                diff[data_num] = (float)abs(dustbox_ult_state[ult_num].data_buffer[data_num] - dustbox_ult_state[ult_num].data_buffer[data_num-1]);
                if(diff[data_num] == 0){
                    //对于 0 的赋值为1,因为下面计算这个值会作为分母
                    diff[data_num] = 1;
                }
            }
            //计算权重
            for(int diff_num = 1; diff_num < MAX_FILTER_NUM; diff_num++){
                //差值归一化
                diff[diff_num] = 1.f / diff[diff_num]; 
            }
            float diff_sum = 0.f;
            for(int diff_num = 1; diff_num < MAX_FILTER_NUM; diff_num++){
                //计算差值总和
                diff_sum += diff[diff_num];
            }
            for(int diff_num = 1; diff_num < MAX_FILTER_NUM; diff_num++){
                //权重
                dustbox_ult_state[ult_num].wight[diff_num] = diff[diff_num] / diff_sum;
            }
            //计算输出值
            float data_calc = 0;
            for(int data_num = 1; data_num < MAX_FILTER_NUM; data_num++){
                data_calc += (float)dustbox_ult_state[ult_num].data_buffer[data_num] * dustbox_ult_state[ult_num].wight[data_num];
            }
            dustbox_ult_state[ult_num].data_output = (uint16_t)data_calc;
        }
        dustbox_ult_data[ult_num] = dustbox_ult_state[ult_num].data_output;
    }
    

    std_msgs::Float32MultiArray dataArray_before_kalman;
    for(int i = 0 ; i < dustbox_max_type_ult;i++){
        dataArray_before_kalman.data.push_back(dustbox_ult_data[i] / 1000.0f);
    }
    pub_dustbox_all_before_kalman.publish(dataArray_before_kalman);
    //在此处增加卡尔曼滤波
    if(kalman_enable_){
        kalman_filter(ult_dustbox,dustbox_ult_data,dustbox_max_type_ult);
    }
        
    std_msgs::Float32MultiArray dataArray_filtered;
    for(int i = 0 ; i < dustbox_max_type_ult;i++){
        dataArray_filtered.data.push_back(dustbox_ult_data[i] / 1000.0f);
    }
    pub_dustbox_all_filtered.publish(dataArray_filtered);

    //数据发布
    //箱子底部超声波发布
    sensor_msgs::Range rangeData;
    rangeData.header.stamp = ros::Time::now();
    rangeData.radiation_type = 0;
    rangeData.min_range = min_range;
    rangeData.max_range = msg_max_range;
    rangeData.field_of_view = 24*M_PI/180.0f;
    rangeData.range = dustbox_ult_data[dustbox_bottom_ult]  / 1000.f;
    rangeData.header.frame_id = "dustbox_bottom";
    dustbox_ult_bottom_pub.publish(rangeData);
    //----------------------------------------------------------------------------------------
    //箱子后方超声波发布,策略：
    /*
    探头1（后右）只收不发，探头 2（后左又发又收）
    当两个探头数据满足 0 < 数据 < rear_ult_detect_max 时采用三角形解算 
    此处在判断数据是否为有效数据时，使用的最大距离限制是rear_ult_detect_max，是为了对数据在 max_range 的限制下再次进行限制，去除滤波时数据经过计算出现漂移产生的影响。
    其余看情况解析
    */
   if(dustbox_ult_data[dustbox_rear_right_ult] == min_range*1000.f && dustbox_ult_data[dustbox_rear_left_ult] == min_range * 1000.f){
       //两个探头都为最小值时输出最小值
       rangeData.range = min_range;
   }
   else if( min_range*1000.f < dustbox_ult_data[dustbox_rear_right_ult] < rear_ult_detect_max*1000.f  && dustbox_ult_data[dustbox_rear_left_ult] >= rear_ult_detect_max * 1000.f){
       //只有一个探头的数据有效时，对数据进行限制,小于rear_ult_detect_min 才视为障碍
       if(dustbox_ult_data[dustbox_rear_right_ult] < rear_ult_detect_min * 1000.f ){
           rangeData.range = dustbox_ult_data[dustbox_rear_right_ult] / 1000.f;
       }
       else{
           rangeData.range = rear_ult_detect_max;
       }
   }
   else if( dustbox_ult_data[dustbox_rear_right_ult] >= rear_ult_detect_max * 1000.f &&  min_range*1000.f < dustbox_ult_data[dustbox_rear_left_ult] < rear_ult_detect_max*1000.f){
        //只有一个探头的数据有效时，对数据进行限制,小于rear_ult_detect_min 才视为障碍
        if(dustbox_ult_data[dustbox_rear_left_ult] < rear_ult_detect_min * 1000.f ){
           rangeData.range = dustbox_ult_data[dustbox_rear_left_ult] / 1000.f;
       }
       else{
           rangeData.range = rear_ult_detect_max;
       }
   }
   else if(dustbox_ult_data[dustbox_rear_right_ult] >= rear_ult_detect_max * 1000.f && dustbox_ult_data[dustbox_rear_left_ult] >= rear_ult_detect_max * 1000.f){
       //两个都大于限制时输出限制值
       rangeData.range = rear_ult_detect_max;
   }
   else if(min_range*1000.f < dustbox_ult_data[dustbox_rear_right_ult] < rear_ult_detect_max*1000.f && min_range*1000.f < dustbox_ult_data[dustbox_rear_left_ult] < rear_ult_detect_max*1000.f){
       //三角形解算
       //首先判断是否具备组成三角形的条件
       if( (dustbox_ult_data[dustbox_rear_right_ult] + dustbox_ult_data[dustbox_rear_left_ult]) > REAR_ULT_DIST &&
           (dustbox_ult_data[dustbox_rear_right_ult] + REAR_ULT_DIST) > dustbox_ult_data[dustbox_rear_left_ult] &&
           (dustbox_ult_data[dustbox_rear_left_ult] + REAR_ULT_DIST) > dustbox_ult_data[dustbox_rear_right_ult]
        ){
            //后左探头数据和车尾形成的夹角 余弦函数再反余弦求出后左超声波探头所在点那个角的角度
            double angle_rearleft_to_rear = acos((float)(REAR_ULT_DIST * REAR_ULT_DIST + dustbox_ult_data[dustbox_rear_left_ult] * dustbox_ult_data[dustbox_rear_left_ult] \
             - dustbox_ult_data[dustbox_rear_right_ult] * dustbox_ult_data[dustbox_rear_right_ult]) / (float)(2 * REAR_ULT_DIST * dustbox_ult_data[dustbox_rear_left_ult]));
            //障碍物到左后探头的水平距离
            double dist_H_obs_to_rearleft = fabs(dustbox_ult_data[dustbox_rear_left_ult] * cos(angle_rearleft_to_rear));
            //障碍物到左后探头的垂直
            double dist_V_obs_to_rearleft = fabs(dustbox_ult_data[dustbox_rear_left_ult] * sin(angle_rearleft_to_rear));
            if(angle_rearleft_to_rear < PI / 2){
                //如果是锐角，判断障碍点有没有超过右边车身
                if(dist_H_obs_to_rearleft < ( (VEHICLE_WIDTH - REAR_ULT_DIST) / 2 + REAR_ULT_DIST + rear_ult_detect_offset * 1000.f) ){
                    //如果没有超过车身，输出垂直距离
                    rangeData.range = dist_V_obs_to_rearleft / 1000.f;
                }else{
                    //如果超过车身，输出最大距离
                    rangeData.range = rear_ult_detect_max;
                }
            }
            else if(angle_rearleft_to_rear ==  PI / 2) {
                //如果是直角，输出垂直距离
                rangeData.range = dist_V_obs_to_rearleft / 1000.f;
            }
            else if(angle_rearleft_to_rear > PI /2 ){
                //如果是钝角，判断障碍点有没有超过左边车身
                if(dist_H_obs_to_rearleft < ( (VEHICLE_WIDTH - REAR_ULT_DIST) / 2 + rear_ult_detect_offset * 1000.f) ){
                    //如果没有超过车身，输出垂直距离
                    rangeData.range = dist_V_obs_to_rearleft / 1000.f;
                }else{
                    //如果超过车身，输出最大距离
                    rangeData.range = rear_ult_detect_max;
                }
            }
        }
        else{
            //不满足组成三角形的条件，输出两个探头较小的值
             rangeData.range = cmin(dustbox_ult_data[dustbox_rear_right_ult],dustbox_ult_data[dustbox_rear_left_ult]) / 1000.f;
        }
   }
   else{
       //其他（应该没有了，以防万一），输出两个探头较小值
       rangeData.range = cmin(dustbox_ult_data[dustbox_rear_right_ult],dustbox_ult_data[dustbox_rear_left_ult]) / 1000.f;
   }
    rangeData.header.frame_id = "dustbox_rear_center";
    dustbox_ult_rear_center_pub.publish(rangeData);
}