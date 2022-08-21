#ifndef MCU_OTA_HPP
#define MCU_OTA_HPP
//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <common_type.hpp>
#include <cmd_type.hpp>
#include <pthread.h>
#include <crc16.hpp>
#include <common_value.hpp>
#include "ota_state.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
//#include "cti_fpga_serial/updateinfo.h"
#include "cti_chassis_msgs/msg/update_info.hpp"
#include <signal.h>
#include <string>
#include "ctilog/log.hpp"
#include "ctilog/loghelper.cpp.hpp"

using namespace cti::log;
constexpr char const *kN = "mcu_ota";

class McuOta {
public:
    McuOta(std::shared_ptr<rclcpp::Node> node_ptr, std::vector<std::string> parameters);
    ~McuOta() {};
    int run();
private:
    std::shared_ptr<rclcpp::Node> m_node_ptr;
    update_option_t update_option_;
    serial_recv_buf_type serial_recv;
    serial_send_buf_type serial_send;
    serial_frame_type serial_frame;
    serial_candidate_frame_type candidate;
    //接收队列数据
    recv_frame_queue_type recv_frame_queue;
    //发送队列数据
    send_frame_queue_type send_frame_queue;
    ros_comm_type ros_recv;
    ros_comm_type ros_send;
    cmd_overtime_clock_type cmd_overtime;
    double update_start_time;

    void initLog(const std::string name);

    //接收相关的函数
    void init_frame_queue();

    void init_recv_buffer();

    void find_serial_frame();

    int query_serial_data();

    int ros_buffer_read(unsigned char *dest, int cnt);

    int find_frame_header(int pos, char *found);

    void parse_data_from_recv(unsigned char *dest, int pos_start, int cnt);

    int get_cmd_data_length(unsigned char cmd_id);

    int parse_recv_frame();

    rclcpp::Subscription<cti_chassis_msgs::msg::UpdateInfo>::SharedPtr manager_sub;

    void ros_recv_data_call_back(const cti_chassis_msgs::msg::UpdateInfo::SharedPtr msg);
    double time_recv_cmd_211;
    //发送处理相关的函数
    int construct_serial_frame(serial_frame_type *frame, unsigned char cmd, void *data);

    std::vector<std::string> input_parameters_;

    int check_user_update_option(std::string input_module);

    int manage_module_updates(int option, int main_option);

    void set_upd_info(update_info_type *info, unsigned char target_id);

    int pack_send_frame_buf_in_queue(serial_frame_type *frame, unsigned short resend);

    void set_cmd_overtime_clock(int magic, double delay_time);

    void send_update_status_cmd_once(unsigned char status, unsigned char target_id);

    int do_single_firmware_update(unsigned int state, OtaState *result);

    int init_firmware_upgrade_params(unsigned long size);

    rclcpp::Publisher<cti_chassis_msgs::msg::UpdateInfo>::SharedPtr manager_pub;

    void send_data(); //发送数据

    int getUsedSize(ros_comm_type *data);
    int getAvailSize(ros_comm_type *data); 

    uint32_t seq_num;

    void module_serial_process();

    void process_received_serial_frames();

    int process_cmd(serial_frame_type *frame);

    void clear_cmd_overtime_clock(int magic);

    void clear_resend_cmd(unsigned int cmd);

    int get_firmware_packet_data(unsigned char *dest, uint32_t file_offset, uint16_t length);

    void clear_resend_param(unsigned char cmd);

    int update_firmware_upgrade_params(unsigned short require_packet);

    void send_serial_frames();
    int send_serial_flow(unsigned char *src, int cnt);
    void check_cmd_overtime_clock();
    int init_firmware_upgrade();
    void send_update_status_cmd(unsigned char status, unsigned char target_id);
    std::string getLocalDate();
    void info_state();
    //状态池
    unsigned char manage_update_state;
    uint16_t firmware_upgrade_flag;
    int firmware_bin_file_fd;
    uint32_t sp_total_crc;
    firmware_packet_type fw_packet;


    //报告
    OtaState *ota_state;

};

#endif