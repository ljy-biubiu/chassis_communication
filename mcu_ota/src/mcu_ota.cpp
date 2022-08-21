#include <mcu_ota.hpp>

McuOta::McuOta(std::shared_ptr <rclcpp::Node> node_ptr, std::vector <std::string> parameters) :
        m_node_ptr(node_ptr),
        input_parameters_(parameters) {
    ros_recv = {0};
    ros_send = {0};
    serial_frame = {0};
    serial_recv = {0};
    serial_send = {0};
    candidate = {0};
    recv_frame_queue = {0};
    send_frame_queue = {0};
    manage_update_state = INIT_UPDATE_LIST;
    firmware_upgrade_flag = INFORM_UPDATE;
    cmd_overtime = {0};
    ota_state = new OtaState(node_ptr);
    firmware_bin_file_fd = -1;
    sp_total_crc = 0;
    fw_packet = {0};
    manager_pub = node_ptr->create_publisher<cti_chassis_msgs::msg::UpdateInfo>("cti/fpga_serial/stmupdate", 20);
    manager_sub = node_ptr->create_subscription<cti_chassis_msgs::msg::UpdateInfo>("cti/fpga_serial/stminfo", 5,
                                                bind(&McuOta::ros_recv_data_call_back, this, std::placeholders::_1));
    seq_num = 1;
    init_frame_queue();
    init_recv_buffer();
    std::string filelog;
    filelog = std::string(std::getenv("HOME")) + std::string("/log/mcu_ota.log");
    initLog(filelog);
}

void McuOta::initLog(const std::string name) 
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
    Logger::getLogger().setMaxSize(200 * 1024 * 1024);
}

void McuOta::init_frame_queue() 
{
    memset(&recv_frame_queue, 0, sizeof(recv_frame_queue));
    memset(&send_frame_queue, 0, sizeof(send_frame_queue));
}

void McuOta::info_state() 
{
    Info("main_module_id: " << ota_state->upgrade_report.main_module_id);
    Info("main_module_name: " << ota_state->upgrade_report.main_module_name);
    Info("module_id: " << ota_state->upgrade_report.module_id);
    Info("module_name: " << ota_state->upgrade_report.module_name);
    Info("bin_file_name: " << ota_state->upgrade_report.bin_file_name);
    Info("update_time: " << ota_state->upgrade_report.update_time);
    Info("result: " << ota_state->upgrade_report.result);
    if (ota_state->upgrade_report.result == UPDATE_START) {
        Info("result_message: " << "UPDATE_START");
    }
    if (ota_state->upgrade_report.result == UPDATE_FAILED) {
        Info("result_message: " << "UPDATE_FAILED");
    }
    if (ota_state->upgrade_report.result == UPDATE_SUCCESSED) {
        Info("result_message: " << "UPDATE_SUCCESSED");
    }
    Info("Estage: " << ota_state->upgrade_report.Estage);
    Info("EStateMessage: " << ota_state->upgrade_report.EStateMessage);
    Info("Ecode: " << ota_state->upgrade_report.Ecode);
    Info("app_version_before: " << ota_state->upgrade_report.app_version_before);
    Info("app_version_after: " << ota_state->upgrade_report.app_version_after);
    if (ota_state->upgrade_report.motor_driver_main_bus_comm_disable) {
        Info("motor_driver_main_bus_comm_disable: true");
    } else {
        Info("motor_driver_main_bus_comm_disable: false");
    }
    if (ota_state->upgrade_report.motor_driver_aux_bus_comm_disable) {
        Info("motor_driver_aux_bus_comm_disable: true");
    } else {
        Info("motor_driver_aux_bus_comm_disable: false");
    }
}

int McuOta::find_frame_header(int pos, char *found) 
{
    int i, match_cnt;
    i = 0;
    match_cnt = 0;
    if ((pos < 0) || (pos >= RECEIVE_BUFFER_SIZE)) {
        return 0;
    }
    while (i < (serial_recv.cnt - FRAME_HEADER_SIZE)) {
        match_cnt = 0;
        while (match_cnt < FRAME_HEADER_SIZE) {
            if (serial_recv.buf[RECEIVE_RING_ADD(pos, i)] != serial_frame_header[match_cnt]) {
                i++;
                break;;
            } else {
                match_cnt++;
                i++;
            }
        }
        if (FRAME_HEADER_SIZE == match_cnt) {
            *found = 1;
            break;
        }
    }
    return i;
}

void McuOta::parse_data_from_recv(unsigned char *dest, int pos_start, int cnt) 
{
    int copy1, copy2;
    if (NULL == dest) {
        return;
    }
    if ((pos_start < 0) || (pos_start >= RECEIVE_BUFFER_SIZE) || cnt > serial_recv.cnt) {
        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "parse param range error.");
        return;
    }
    copy1 = ((RECEIVE_BUFFER_SIZE - pos_start) >= cnt) ? cnt : (RECEIVE_BUFFER_SIZE - pos_start);
    copy2 = cnt - copy1;
    memcpy(dest, serial_recv.buf + pos_start, copy1);
    if (copy2 > 0) {
        memcpy(dest + copy1, serial_recv.buf, copy2);
    }
}

int McuOta::get_cmd_data_length(unsigned char cmd_id) 
{
    int i = 0;
    for (i = 0; i < sizeof(cmd_data_length) / sizeof(cmd_data_length[0]); i++) {
        if (cmd_id == cmd_data_length[i].cmd_id) {
            return cmd_data_length[i].cmd_length;
        }
    }
    return -1;
}

int McuOta::parse_recv_frame() {
    unsigned char tail = (recv_frame_queue.head + recv_frame_queue.cnt) % MAX_RECV_FRAME_QUEUE;
    serial_frame_type *frame = &recv_frame_queue.frame[tail];
    if (recv_frame_queue.cnt >= MAX_RECV_FRAME_QUEUE) {
        return 0;
    }
    memset(frame, 0, sizeof(recv_frame_queue.frame[0]));
    memcpy(frame->header, candidate.header, sizeof(frame->header));
    frame->cmd = candidate.cmd;
    frame->cmd_length = candidate.cmd_length;
    parse_data_from_recv((unsigned char *) &frame->data,
                         (serial_recv.start + sizeof(serial_fixed_header_type)) % RECEIVE_BUFFER_SIZE, \
        frame->cmd_length - sizeof(serial_fixed_header_type));
    frame->crc16 = candidate.crc16;
    recv_frame_queue.cnt += 1;
    return 1;
}

void McuOta::set_upd_info(update_info_type *info, unsigned char target_id) 
{
    info->dest = target_id;
    info->src = MODULE_SELF;
}

int McuOta::construct_serial_frame(serial_frame_type *frame, unsigned char cmd, void *data) 
{
    if (cmd > CMD_MAX) {
        RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(), "Invalid cmd type when init serial frame!!! " << cmd);
        return -1;
    }
    int length = get_cmd_data_length(cmd);
    if ((-1 == length) || (length + sizeof(serial_fixed_header_type) > sizeof(serial_frame_type))) {
        RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(),
                            "Oops, we got a cmd_data_length[cmd] > sizeof(serial_frame.data),cmd_data_length: "
                            << length \
                            << " sizeof(serial_frame.data): " \
                            << (int) sizeof(serial_frame_type) - (int) sizeof(serial_fixed_header_type) \
                            << ",correct the union_data_type!");
        return -1;
    }
    memset(frame, 0, sizeof(serial_frame_type));
    memcpy(frame->header, serial_frame_header, sizeof(frame->header));
    frame->cmd = cmd;
    frame->cmd_length = sizeof(serial_fixed_header_type) + length;
    memcpy((void *) &(frame->data), data, length);
    return 0;
}

int McuOta::pack_send_frame_buf_in_queue(serial_frame_type *frame, unsigned short resend) 
{
    serial_send_buf_type *send_buf;
    if (NULL == frame) {
        return -1;
    }
    if (send_frame_queue.cnt >= MAX_SEND_FRAME_QUEUE) {
        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),
                           "Send frame queue high level detected, cnt: " << send_frame_queue.cnt);
        return -1;
    }
    /*set resend param*/
    send_frame_queue.resend[(send_frame_queue.head + send_frame_queue.cnt) % MAX_SEND_FRAME_QUEUE] = resend;
    /*add buf to the queue tail.*/
    send_buf = &(send_frame_queue.frame[(send_frame_queue.head + send_frame_queue.cnt) % MAX_SEND_FRAME_QUEUE]);
    memset(send_buf->buf, 0, SERIAL_FRAME_SIZE);
    memcpy(send_buf->buf, frame, frame->cmd_length);
    //test
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "send frame buf is packed:");
    for (int i = 0; i < (frame->cmd_length + sizeof(frame->crc16)); i++) {
        std::cout << std::dec << int(*((unsigned char *) frame + i)) << ".";
    }
    std::cout << "" << std::endl;
    init_crc();
    frame->crc16 = calc_crc(send_buf->buf, frame->cmd_length);
    memcpy(send_buf->buf + frame->cmd_length, &frame_crc, sizeof(frame_crc));
    /*the size of current send buf*/
    send_buf->cnt = frame->cmd_length + sizeof(frame->crc16);
    send_frame_queue.cnt++;
    std::cout << "send buf calc crc is " << std::hex << frame->crc16 << std::endl;
    return 0;
}

//清空超时计时器
void McuOta::clear_cmd_overtime_clock(int magic) 
{
    if (magic == cmd_overtime.prev_magic) {
        memset(&cmd_overtime, 0, sizeof(cmd_overtime_clock_type));
    }
}

//清空重发命令
void McuOta::clear_resend_cmd(unsigned int cmd) 
{
    if ((send_frame_queue.resend[send_frame_queue.head] >> 8) == cmd) {
        send_frame_queue.resend[send_frame_queue.head] = 0;
    }
}

//设置超时计时器
void McuOta::set_cmd_overtime_clock(int magic, double delay_time) {
    if ((cmd_overtime.prev_magic != 0) && (cmd_overtime.prev_magic != magic)) {
        clear_cmd_overtime_clock(cmd_overtime.prev_magic);
    }
    cmd_overtime.prev_magic = magic;
    cmd_overtime.delay_time = delay_time;
    cmd_overtime.start_time = m_node_ptr->get_clock()->now().seconds();
    cmd_overtime.enable_check = true;
}

//检查超时计时器
void McuOta::check_cmd_overtime_clock() {
    if (false == cmd_overtime.enable_check) {
        return;
    }
    if (m_node_ptr->get_clock()->now().seconds() - cmd_overtime.start_time > cmd_overtime.delay_time) {
        cmd_overtime.enable_check = false;
        ota_state->set_report_information(UPDATE_FAILED, firmware_upgrade_flag, "Wait cmd response overtime");
        firmware_upgrade_flag = FIRMWARE_ABORT_UPGRADE;
    }
}

void McuOta::send_update_status_cmd_once(unsigned char status, unsigned char target_id) {
    upd_cmd_211_type upd_cmd_211 = {0};
    set_upd_info(&upd_cmd_211.upd_info, target_id);
    upd_cmd_211.status = status;  //upgrade notice to module.
    construct_serial_frame(&serial_frame, UPD_CMD_211, (void *) &upd_cmd_211);
    pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_211 << 8) + 4);
}

int McuOta::init_firmware_upgrade_params(unsigned long size) 
{
    char extra_packet = 0;
    memset(&fw_packet, 0, sizeof(firmware_packet_type));
    //\B6\C1ȡ\CEļ\FE\A3\AC\BB\F1\B5\C3\CEļ\FE\B4\F3С
    fw_packet.firmware_size = size;
    fw_packet.packet_length = FIRMWARE_PACKET_SIZE;
    extra_packet = (fw_packet.firmware_size % fw_packet.packet_length) ? 1 : 0;
    fw_packet.total_packets = fw_packet.firmware_size / fw_packet.packet_length + extra_packet;
    fw_packet.current_packet = extra_packet;
    fw_packet.actual_length = (fw_packet.current_packet == fw_packet.total_packets) ? \
            (fw_packet.firmware_size % fw_packet.packet_length) : fw_packet.packet_length;
    fw_packet.read_offset = 0;
    sp_total_crc = 0;
    //test
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "fw_packet.firmware_size: " << fw_packet.firmware_size);
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "fw_packet.current_packet: " << fw_packet.current_packet);
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "fw_packet.total_packets: " << fw_packet.total_packets);
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "fw_packet.packet_length: " << fw_packet.packet_length);
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "fw_packet.actual_length: " << fw_packet.actual_length);
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "fw_packet.read_offset: " << fw_packet.read_offset);
    return 0;
}

int McuOta::init_firmware_upgrade() 
{
    int fd;
    unsigned long file_size;
    if (-1 == (fd = open(update_option_.bin_file_name.c_str(), O_RDONLY))) {
        RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(), "Open  Error" << update_option_.bin_file_name);
        return fd;
    }
    firmware_bin_file_fd = fd;
    file_size = lseek(fd, 0L, SEEK_END);
    lseek(fd, 0L, SEEK_SET);
    //test
    uint32_t total_crc = 0;
    unsigned long cnt = file_size;
    unsigned char temp = 0;
    while (cnt) {
        if (1 == read(fd, &temp, 1)) {
            total_crc += temp;
            cnt--;
        }
    }
    RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "the total file crc is:" << total_crc);
    lseek(fd, 0L, SEEK_SET);
    init_firmware_upgrade_params(file_size);
    return fd;
}

int McuOta::do_single_firmware_update(unsigned int state, OtaState *result) 
{
    check_cmd_overtime_clock();
    switch (state) {
        case INFORM_UPDATE: {
            RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "send cmd 211.");
            //send cmd 211
            upd_cmd_211_type upd_cmd_211 = {0};
            set_upd_info(&upd_cmd_211.upd_info, MODULE_MOVE_CONTROL_BOARD);
            upd_cmd_211.status = 0;  //upgrade notice to module.          
            construct_serial_frame(&serial_frame, UPD_CMD_211, (void *) &upd_cmd_211);
            pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_211 << 8) + MAX_CMD_RESEND_TIME);
            set_cmd_overtime_clock(UPD_CMD_211, MAX_CMD_OVERTIME_VALUE);
            firmware_upgrade_flag = WAIT_INFORM_UPDATE_RESPOND;
        }
        break;
        case WAIT_INFORM_UPDATE_RESPOND:
            //wait cmd 212
        break;
        case DELAY_SOME_TIME: {
            if (m_node_ptr->get_clock()->now().seconds() - time_recv_cmd_211 > 2.0) {
                firmware_upgrade_flag = SEND_MAIN_MODULE_ID;
            }
        }
        break;
        case SEND_MAIN_MODULE_ID: {
            upd_cmd_212_type upd_cmd_212 = {0};
            upd_cmd_212.update_node = update_option_.main_module_id;
            set_upd_info(&upd_cmd_212.upd_info, MODULE_MOVE_CONTROL_BOARD);
            construct_serial_frame(&serial_frame, UPD_CMD_212, (void *) &upd_cmd_212);
            pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_212 << 8) + MAX_CMD_RESEND_TIME);
            set_cmd_overtime_clock(UPD_CMD_212, MAX_CMD_OVERTIME_VALUE);
            firmware_upgrade_flag = WAIT_MAIN_MODULE_ID_RESPOND;
        }
        break;
        case WAIT_MAIN_MODULE_ID_RESPOND:
        break; //wait 212 respond
        case FIRMWARE_NORMAL: {
            upd_cmd_209_type upd_cmd_209 = {0};
            set_upd_info(&upd_cmd_209.upd_info, update_option_.module_id);
            upd_cmd_209.check = 1;  //notice to check.
            construct_serial_frame(&serial_frame, UPD_CMD_209, (void *) &upd_cmd_209);
            pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_209 << 8) + MAX_CMD_RESEND_TIME);
            set_cmd_overtime_clock(UPD_CMD_209, MAX_CMD_OVERTIME_VALUE);
            firmware_upgrade_flag = FIRMWARE_WAIT_CHECK_BEFORE_VERSION;
        }
        break;
        case FIRMWARE_WAIT_CHECK_BEFORE_VERSION:
        break;//wait 209 respond: 210
        case FIRMWARE_CHECK_VERSION_OK: {
            RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "Detect upgrade firmware file, begin to upgrade firmware.");
            if (-1 != init_firmware_upgrade()) {
                upd_cmd_200_type upd_cmd_200 = {0};
                set_upd_info(&upd_cmd_200.upd_info, update_option_.module_id);
                upd_cmd_200.status = 0;  //notice to upgrade.
                upd_cmd_200.firmware_size = fw_packet.firmware_size;
                construct_serial_frame(&serial_frame, UPD_CMD_200, (void *) &upd_cmd_200);
                pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_200 << 8) + MAX_CMD_RESEND_TIME);
                set_cmd_overtime_clock(UPD_CMD_200, MAX_CMD_OVERTIME_VALUE);
                firmware_upgrade_flag = FIRMWARE_PRE_UPDATE;
                RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "In state FIRMWARE_NORMAL send cmd_200.");
            } 
            else {
                firmware_upgrade_flag = FIRMWARE_ABORT_UPGRADE;
                ota_state->set_report_information(UPDATE_FAILED, FIRMWARE_CHECK_VERSION_OK, "Open upgrade file error.");
            }
        }
        break;
        case FIRMWARE_PRE_UPDATE:
        break;//wait respond cmd_200, turns to FIRMWARE_WAIT_START_UPDATE
        case FIRMWARE_ATHENTICATION:
            break;//athentication  cmd_201
        case FIRMWARE_WAIT_START_UPDATE:
            break;//wait receive cmd_202, turns to FIRMWARE_UPDATING
        case FIRMWARE_UPDATING:
            break;//cmd_203 and wait cmd_204 
        case FIRMWARE_UPDATED_REBOOT:
            break;// send cmd_205 and wait cmd_205
        case FIRMWARE_WAIT_FINISH_UPDATE:
            break;//wait cmd_206
        case FIRMWARE_ABORT_UPGRADE: {
            if (firmware_bin_file_fd != -1) {
                close(firmware_bin_file_fd);
                firmware_bin_file_fd = -1;
            }
            manage_update_state = RECORD_STATUS;
            RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(), "Firmware upgrade failed, abort the module upgrade.");
        }
        break;
        case FIRMWARE_UPGRADE_FINISHED: {
            if (firmware_bin_file_fd != -1) {
                close(firmware_bin_file_fd);
                firmware_bin_file_fd = -1;
            }
            ota_state->set_report_information(UPDATE_SUCCESSED, FIRMWARE_UPGRADE_FINISHED, "Update file finished.");
            manage_update_state = RECORD_STATUS;
        }
            break;
        default:
            break;
    }
    return 0;
}

void McuOta::send_update_status_cmd(unsigned char status, unsigned char target_id) {
    upd_cmd_211_type upd_cmd_211 = {0};
    set_upd_info(&upd_cmd_211.upd_info, target_id);
    upd_cmd_211.status = status;  //upgrade notice to module.
    construct_serial_frame(&serial_frame, UPD_CMD_211, (void *) &upd_cmd_211);
    pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_211 << 8) + MAX_CMD_RESEND_TIME);
}

int McuOta::manage_module_updates(int option, int main_option) 
{
    static double start_time;
    double check_time;
    switch (manage_update_state) {
        case INIT_UPDATE_LIST: {
            manage_update_state = DO_UPDATE;
            ota_state->upgrade_report.main_module_id = update_option_.main_module_id;
            ota_state->upgrade_report.main_module_name = update_option_.main_module_name;
            ota_state->upgrade_report.module_id = update_option_.module_id;
            ota_state->upgrade_report.module_name = update_option_.module_name;
            ota_state->upgrade_report.bin_file_name = update_option_.bin_file_name;
            ota_state->set_report_information(UPDATE_START, firmware_upgrade_flag, "update start.");
            RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "update start");
            Info("update start:");
            info_state();
        }break;
        case DO_UPDATE: {
            do_single_firmware_update(firmware_upgrade_flag, ota_state);
        }break;
        case RECORD_STATUS: {
            manage_update_state = REPORT_STATUS;
            unsigned char ret = 2;  //update success
            unsigned char i;
            if (0 == ota_state->upgrade_report.result) {
                ret = 3; //update fail
            }
            if (true == ota_state->upgrade_report.motor_driver_main_bus_comm_disable) {
                usleep(20000);
                for (int i = MODULE_MOTOR_DRIVER_FRONT_LEFT; i <= MODULE_MOTOR_DRIVER_BACK_RIGHT; i++) {
                    send_update_status_cmd_once(ret, i);  //2 or 3
                    usleep(20000);
                }
                ota_state->upgrade_report.motor_driver_main_bus_comm_disable = false;
            }
            if (true == ota_state->upgrade_report.motor_driver_aux_bus_comm_disable) {
                usleep(20000);
                for (int i = MODULE_MOTOR_TURN_FRONT; i <= MODULE_MOTOR_BREAK_BACK; i++) {
                    send_update_status_cmd_once(ret, i);  //2 or 3
                    usleep(20000);
                }
                ota_state->upgrade_report.motor_driver_aux_bus_comm_disable = false;
            }
            send_update_status_cmd(ret, MODULE_MOVE_CONTROL_BOARD);
            start_time = m_node_ptr->get_clock()->now().seconds();
            break;
        }
        case REPORT_STATUS: {
            if ((m_node_ptr->get_clock()->now().seconds() - start_time) > 5) {
                ota_state->upgrade_report.update_dura_time =
                        m_node_ptr->get_clock()->now().seconds() - update_start_time;
                int ret = ota_state->report_module_update_status();
                info_state();
                Info("update finished, exit mcu_ota!");
                if (ota_state->upgrade_report.result == UPDATE_SUCCESSED) {
                    Info("update result: SUCCESS");
                    return UPDATE_SUCCESSED;
                }
                if (ota_state->upgrade_report.result == UPDATE_FAILED) {
                    Info("update result: FAIL");
                    return UPDATE_FAILED;
                }
            }
            break;
        }
        default:
            break;
    }
    return UPDATE_ING;
}

//recv
void McuOta::ros_recv_data_call_back(const cti_chassis_msgs::msg::UpdateInfo::SharedPtr msg) 
{
    int recv_len = msg->data.size();
    int avail_len = getAvailSize(&ros_recv);
    int write_len = recv_len > avail_len ? avail_len : recv_len;
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),
                       "recv a ros package: seq_num: " << msg->seq_num << " data_size: " << msg->data.size());
    if (avail_len < write_len) {
        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),
                           "ros recv buffer is not enough! discard: " << write_len - avail_len);
    } 
    else {
        for (int i = 0; i < recv_len; i++) {
            ros_recv.buf[ros_recv.end] = msg->data[i];
            ros_recv.end = (ros_recv.end + 1) % ROS_COMM_BUF_SIZE;
        }
    }
}

int McuOta::ros_buffer_read(unsigned char *dest, int cnt) 
{
    if (NULL == dest) {
        return 0;
    }
    int actual_len = (ros_recv.end + ROS_COMM_BUF_SIZE - ros_recv.start) % ROS_COMM_BUF_SIZE;
    int recv_len = cnt > actual_len ? actual_len : cnt;
    if (0 == recv_len) {
        return 0;
    }
    for (int i = 0; i < recv_len; i++) {
        dest[i] = ros_recv.buf[ros_recv.start];
        ros_recv.start = (ros_recv.start + 1) % ROS_COMM_BUF_SIZE;
    }
    RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(),
                        "ros buffer read size: " << recv_len << 
                        ",start: " << ros_recv.start << 
                        ",end: " << ros_recv.end);
    return recv_len;
}

void McuOta::init_recv_buffer() 
{
    serial_recv.start = 0;
    serial_recv.cnt = 0;
    serial_recv.state = RECV_STATE_FINDING_HEADER;
    memset(serial_recv.buf, 0, sizeof(serial_recv.buf));
    init_crc();
}

int McuOta::query_serial_data() 
{
    int pos = 0;
    int remain_buffer_cnt = 0;
    int ret;
    remain_buffer_cnt = RECEIVE_BUFFER_SIZE - serial_recv.cnt;

    /*low available buffer level*/
    if (remain_buffer_cnt < LOW_RECEIVE_BUFFER_THRESHOLD) {
        RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "Low recv buffer level!****cleaning recv buffer!");
        init_recv_buffer();
        ret = 0;
        return ret;
    }
    /*if near buffer end, just read some data to fill buffer tail.*/
    pos = RECEIVE_RING_ADD(serial_recv.start, serial_recv.cnt);
    if (pos >= serial_recv.start) {
        /*remain_buffer_cnt > (RECEIVE_BUFFER_SIZE - pos)*/
        ret = ros_buffer_read(serial_recv.buf + pos, ((RECEIVE_BUFFER_SIZE - pos) > SERIAL_FRAME_SIZE) ? \
                                SERIAL_FRAME_SIZE : (RECEIVE_BUFFER_SIZE - pos));
    } else {
        ret = ros_buffer_read(serial_recv.buf + pos, (remain_buffer_cnt > SERIAL_FRAME_SIZE) ? \
                                SERIAL_FRAME_SIZE : remain_buffer_cnt);
    }
    if (ret > 0) {
        serial_recv.cnt += ret;
    }
    return ret;
}

void McuOta::find_serial_frame() 
{
    int ret, i;
    static int candidate_pos = 0;
    char found_header = 0;
    ret = query_serial_data();
    if (ret < 0) {
        return;
    }
    RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), 
                        "Finding a frame from total " << serial_recv.cnt << 
                        " buffer chars, new recvived " << ret << 
                        " chars, start pos:" << serial_recv.start);
    while (serial_recv.cnt) {
        switch (serial_recv.state) {
            case RECV_STATE_FINDING_HEADER: {
                RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "In state RECV_STATE_FINDING_HEADER");
                if (serial_recv.cnt <= FRAME_HEADER_SIZE) {
                    return;
                }
                found_header = 0;
                ret = find_frame_header(serial_recv.start, &found_header);
                if (found_header) {
                    serial_recv.start = RECEIVE_RING_ADD(serial_recv.start, ret - FRAME_HEADER_SIZE);
                    serial_recv.cnt -= (ret - FRAME_HEADER_SIZE);
                    RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(),
                                        "get candidate frame header. start_pos: " << serial_recv.start << " ret: " << ret);
                    serial_recv.state = RECV_STATE_CHECK_FIXED_HEADER;
                } 
                else {
                    RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "Doesnt find header in recv buffer.");
                    serial_recv.start = RECEIVE_RING_ADD(serial_recv.start, ret);
                    serial_recv.cnt -= ret;
                }
            }
            break;
            case RECV_STATE_CHECK_FIXED_HEADER: {
                RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "In state RECV_STATE_CHECK_FIXED_HEADER");
                if (serial_recv.cnt < sizeof(serial_fixed_header_type)) {
                    RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "Need more data to find fixed header.");
                    return;
                }
                memset(&candidate, 0, sizeof(candidate));
                parse_data_from_recv((unsigned char *) &candidate, serial_recv.start, sizeof(serial_fixed_header_type));
                int length = get_cmd_data_length(candidate.cmd);
                if ((candidate.cmd > CMD_MAX) || (-1 == length) || (candidate.cmd_length != (sizeof(serial_fixed_header_type) + length))) {
                    RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(),
                                        "Received cmd: " << candidate.cmd << 
                                        " and cmd_length: " << candidate.cmd_length << 
                                        " mismatched! But accept when in code developing.");
                    /*not a valid header*/
                    serial_recv.start = RECEIVE_RING_ADD(serial_recv.start, FRAME_HEADER_SIZE);
                    serial_recv.cnt -= FRAME_HEADER_SIZE;

                    serial_recv.state = RECV_STATE_FINDING_HEADER;
                    return;
                } 
                else {
                    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),
                                       "get cmd: " << int(candidate.cmd) << " cmd_length: " << candidate.cmd_length);
                }
                serial_recv.state = RECV_STATE_RECEIVING_CRC;
            }
            break;
            case RECV_STATE_RECEIVING_CRC: {
                RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "In state RECV_STATE_RECEIVING_CRC");
                if (serial_recv.cnt < candidate.cmd_length + 2) //total frame
                {
                    RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "Need more data to find crc.");
                    return;
                }
                candidate_pos = RECEIVE_RING_ADD(serial_recv.start, candidate.cmd_length);
                parse_data_from_recv((unsigned char *) &candidate.crc16, candidate_pos, sizeof(candidate.crc16));
                init_crc();
                for (i = 0; i < candidate.cmd_length; i++) {
                    candidate_pos = RECEIVE_RING_ADD(serial_recv.start, i);
                    calc_crc(&serial_recv.buf[candidate_pos], 1);
                }
                if (frame_crc == candidate.crc16) {
                    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),
                                       "crc check ok, found a valid frame. " << "pos: " << serial_recv.start);
                    if (!parse_recv_frame()) {
                        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),
                                           "Recv frame queue high level detected, cnt: " << recv_frame_queue.cnt);
                    } else {
                        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),
                                           "Get new frame in recv frame queue***, total cnt: " << int(recv_frame_queue.cnt));
                    }
                    serial_recv.start = RECEIVE_RING_ADD(serial_recv.start, candidate.cmd_length + 2);
                    serial_recv.cnt -= (candidate.cmd_length + 2);
                    serial_recv.state = RECV_STATE_FINDING_HEADER;
                } 
                else {
                    RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(),
                                        "bad crc check recv:0x" << candidate.crc16 << " calc:0x" << frame_crc
                                                                << ". Find a fake or damaged frame.");
                    serial_recv.start = RECEIVE_RING_ADD(serial_recv.start, FRAME_HEADER_SIZE);
                    serial_recv.cnt -= FRAME_HEADER_SIZE;
                    serial_recv.state = RECV_STATE_FINDING_HEADER;
                }
            }
            break;
            default: {
                RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(), "Oops!***, wrong recv state: " << serial_recv.state);
                serial_recv.state = RECV_STATE_FINDING_HEADER;
            }
            break;
        }
    }
}

void McuOta::process_received_serial_frames() 
{
    if (recv_frame_queue.cnt >= 3) {
        RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(),
                            "High process_received_serial_frames level detected: " << recv_frame_queue.cnt);
    }
    while (recv_frame_queue.cnt > 0) {
        serial_frame_type *frame = &(recv_frame_queue.frame[recv_frame_queue.head]);
        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "recv frame buf is recved:");
        for (int i = 0; i < (frame->cmd_length + sizeof(frame->crc16)); i++) {
            std::cout << std::dec << int(*((unsigned char *) frame + i)) << ".";
        }
        std::cout << "" << std::endl;
        process_cmd(frame);
        recv_frame_queue.head = (recv_frame_queue.head + 1) % MAX_RECV_FRAME_QUEUE;
        recv_frame_queue.cnt--;
    }
}



int McuOta::get_firmware_packet_data(unsigned char *dest, uint32_t file_offset, uint16_t length) {
    int ret = 0;
    if (NULL == dest) {
        return -1;
    }
    while (length) {
        lseek(firmware_bin_file_fd, file_offset, SEEK_SET);
        ret = read(firmware_bin_file_fd, dest, length);
        ret = (ret >= 0) ? ret : 0;
        dest = dest - ret;
        length = length - ret;
    }
    return 0;
}

void McuOta::clear_resend_param(unsigned char cmd) {
    send_frame_queue.resend[send_frame_queue.head] = 0;
    RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "Catch response, set no resend.");
}

int McuOta::update_firmware_upgrade_params(unsigned short require_packet) 
{
    if ((0 == require_packet) || (require_packet > fw_packet.total_packets)) {
        return -1;
    }
    fw_packet.current_packet = require_packet;
    uint16_t actual_temp = (fw_packet.firmware_size % fw_packet.packet_length) ? \
        (fw_packet.firmware_size % fw_packet.packet_length) : fw_packet.packet_length;
    fw_packet.actual_length = (fw_packet.current_packet < fw_packet.total_packets) ? \
             fw_packet.packet_length : actual_temp;
    fw_packet.read_offset = fw_packet.packet_length * (fw_packet.current_packet - 1);
    //test
    RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "fw_packet.current_packet: " << fw_packet.current_packet);
    RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "total_packets: " << fw_packet.total_packets);
    RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "packet_length: " << fw_packet.packet_length);
    RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "actual_length: " << fw_packet.actual_length);
    RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "read_offset: " << fw_packet.read_offset);
    return 0;
}
//接收指令进程管理
int McuOta::process_cmd(serial_frame_type *frame) 
{
    update_info_type *update_info_recv = (update_info_type * )(&(frame->data));
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),"recv cmd:" << frame->cmd);
    switch (frame->cmd) {
        case UPD_CMD_212: {
            clear_cmd_overtime_clock(UPD_CMD_212);
            clear_resend_cmd(UPD_CMD_212);
            upd_cmd_212_type *upd_cmd_212 = (upd_cmd_212_type * ) & (frame->data.upd_cmd_212);
            if (upd_cmd_212->update_node == update_option_.main_module_id) {
                RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "the main module id respond is right! go to update!");
                firmware_upgrade_flag = FIRMWARE_NORMAL;
            } 
            else {
                ota_state->set_report_information(UPDATE_FAILED, firmware_upgrade_flag, " the main module id respond is wrong! abort update!");
                firmware_upgrade_flag = FIRMWARE_ABORT_UPGRADE;
            }
        }
            break;
        case UPD_CMD_210: {
            clear_cmd_overtime_clock(UPD_CMD_209);
            clear_resend_cmd(UPD_CMD_209);
            upd_cmd_210_type *upd_cmd_210 = (upd_cmd_210_type * ) & (frame->data.upd_cmd_210);
            RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),
                               "*************************GET Firmware Version Info****************************");
            if (upd_cmd_210->run_area < GET_ARRAY_ELE_NUM(run_area_string)) {
                RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "run_area: " << run_area_string[upd_cmd_210->run_area]);
            } else {
                RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "run_area: " << upd_cmd_210->run_area);
            }
            if (upd_cmd_210->update_status < GET_ARRAY_ELE_NUM(target_status_string)) {
                RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),
                                   "update_status: " << target_status_string[upd_cmd_210->update_status]);
            } else {
                RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "update_status: " << upd_cmd_210->update_status);
            }
            RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "boot_ver: " << upd_cmd_210->boot_ver);
            RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "app_ver: " << upd_cmd_210->app_ver);
            RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "update_lib_ver: " << upd_cmd_210->update_lib_ver);
            RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),
                               "*************************GET Firmware Version Info*****************************");
            if (firmware_upgrade_flag == FIRMWARE_WAIT_CHECK_BEFORE_VERSION) {
                ota_state->upgrade_report.app_version_before = (char *) upd_cmd_210->app_ver;
                firmware_upgrade_flag = FIRMWARE_CHECK_VERSION_OK;
            }
            if (firmware_upgrade_flag == FIRMWARE_WAIT_CHECK_AFTER_VERSION) {
                ota_state->upgrade_report.app_version_after = (char *) upd_cmd_210->app_ver;
                firmware_upgrade_flag = FIRMWARE_UPGRADE_FINISHED;
            }
        }
        break;
        case UPD_CMD_200: /*app, to notify a firmware upgrade available.*/
        {
            clear_cmd_overtime_clock(UPD_CMD_200);
            clear_resend_cmd(UPD_CMD_200);
            if ((firmware_upgrade_flag != FIRMWARE_PRE_UPDATE) && (firmware_upgrade_flag != FIRMWARE_WAIT_START_UPDATE)) {
                RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(),
                                    "Receive UPD_CMD_200 with wrong state : " << firmware_upgrade_flag);
                ota_state->set_report_information(UPDATE_FAILED, firmware_upgrade_flag, "Receive cmd_200 in wrong state.");
                //firmware_upgrade_flag = FIRMWARE_ABORT_UPGRADE;
                return -1;
            }
            upd_cmd_200_type *upd_cmd_200 = (upd_cmd_200_type * ) & (frame->data.upd_cmd_200);
            if (1 == upd_cmd_200->status) {
                RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(),
                                    "**********Receive acceptment to firmware upgrade. Change state to FIRMWARE_WAIT_START_UPDATE.**********");
                firmware_upgrade_flag = FIRMWARE_WAIT_START_UPDATE;
            } 
            else {
                RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(),
                                    "**********Receive decline to firmware upgrade: " << upd_cmd_200->status
                                                                                      << frame->data.upd_cmd_200.version[0]);
                ota_state->set_report_information(UPDATE_FAILED, firmware_upgrade_flag, "Receive decline to upgrade.");
                firmware_upgrade_flag = FIRMWARE_ABORT_UPGRADE;
            }
        }
        break;
        case UPD_CMD_202:/*boot, stm32 to pi, to notify it can receive upgrade data now.*/
        {
            clear_cmd_overtime_clock(UPD_CMD_200);
            clear_resend_cmd(UPD_CMD_200);
            if ((firmware_upgrade_flag != FIRMWARE_WAIT_START_UPDATE) &&
                (firmware_upgrade_flag != FIRMWARE_PRE_UPDATE)) {
                RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(),
                                    "Receive UPD_CMD_202 with wrong state: " << firmware_upgrade_flag);
                ota_state->set_report_information(UPDATE_FAILED, firmware_upgrade_flag,
                                                  "Receive cmd_202 in wrong state.");
                //firmware_upgrade_flag = FIRMWARE_ABORT_UPGRADE;
                return -1;
            }
            RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "**********Receive cmd to begin upgrade.**********");
            firmware_upgrade_flag = FIRMWARE_UPDATING;
            /*send first package here*/
            upd_cmd_203_type upd_cmd_203 = {0};
            set_upd_info(&upd_cmd_203.upd_info, update_option_.module_id);
            upd_cmd_203.status = 0;  //notify upgrade data.
            upd_cmd_203.total_packets = fw_packet.total_packets;
            upd_cmd_203.current_packet = fw_packet.current_packet;
            upd_cmd_203.packet_length = fw_packet.packet_length;
            upd_cmd_203.actual_length = fw_packet.actual_length;
            get_firmware_packet_data(upd_cmd_203.data, fw_packet.read_offset, fw_packet.actual_length);
            construct_serial_frame(&serial_frame, UPD_CMD_203, (void *) &upd_cmd_203);
            pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_203 << 8) + MAX_CMD_RESEND_TIME);
            set_cmd_overtime_clock(UPD_CMD_203, MAX_CMD_OVERTIME_VALUE);
        }
        break;
        case UPD_CMD_204: /*boot, stm32 to pi, reply CMD_203 transfer results.*/
        {
            clear_cmd_overtime_clock(UPD_CMD_203);
            clear_resend_cmd(UPD_CMD_203);
            clear_resend_param(frame->cmd);

            upd_cmd_203_type upd_cmd_203 = {0};
            RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "Receive UPD_CMD_204.");
            if (firmware_upgrade_flag != FIRMWARE_UPDATING) {
                RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(),
                                    "Receive UPD_CMD_204 with wrong state: " << firmware_upgrade_flag);
                ota_state->set_report_information(UPDATE_FAILED, firmware_upgrade_flag,
                                                  "Receive cmd_204 in wrong state.");
                //firmware_upgrade_flag = FIRMWARE_ABORT_UPGRADE;
                return -1;
            }
            upd_cmd_204_type *upd_cmd_204 = (upd_cmd_204_type * ) & (frame->data.upd_cmd_204);
            /*get last packet transfer results*/
            /*last packet transfer ok*/
            if (0 == upd_cmd_204->status) {
                RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),
                                   "Receive UPD_CMD_204 ok status, current_packet: " << upd_cmd_204->current_packet);
                RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "fw_packet.total_packets: " << fw_packet.total_packets);

                if (upd_cmd_204->current_packet < fw_packet.total_packets) {
                    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),
                                       "Receive OK result for flashing packet: " << upd_cmd_204->current_packet
                                                                                 << ", send next packet");
                    update_firmware_upgrade_params(upd_cmd_204->current_packet + 1);
                    set_upd_info(&upd_cmd_203.upd_info, update_option_.module_id);
                    upd_cmd_203.status = 0;  //notify upgrade data.
                    upd_cmd_203.total_packets = fw_packet.total_packets;
                    upd_cmd_203.current_packet = fw_packet.current_packet;
                    upd_cmd_203.packet_length = fw_packet.packet_length;
                    upd_cmd_203.actual_length = fw_packet.actual_length;
                    /*read databuf from file here: fw_packet.read_offset, fw_packet.actual_length*/
                    get_firmware_packet_data(upd_cmd_203.data, fw_packet.read_offset, fw_packet.actual_length);
                    construct_serial_frame(&serial_frame, UPD_CMD_203, (void *) &upd_cmd_203);
                    pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_203 << 8) + MAX_CMD_RESEND_TIME);
                    set_cmd_overtime_clock(UPD_CMD_203, MAX_CMD_OVERTIME_VALUE);
                } 
                else {
                    /*when =, this means the last packet is responsed, send reboot command*/
                    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),
                                       "**********Transfer upgrade data complete, send reboot command.**********");
                    upd_cmd_205_type upd_cmd_205 = {0};
                    set_upd_info(&upd_cmd_205.upd_info, update_option_.module_id);
                    upd_cmd_205.status = 0;  //notice to reboot.
                    construct_serial_frame(&serial_frame, UPD_CMD_205, (void *) &upd_cmd_205);
                    pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_205 << 8) + MAX_CMD_RESEND_TIME);
                    set_cmd_overtime_clock(UPD_CMD_205, MAX_CMD_OVERTIME_VALUE * 3);
                    firmware_upgrade_flag = FIRMWARE_UPDATED_REBOOT;
                    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "In state FIRMWARE_UPDATED_REBOOT send upd_cmd_205.");
                }
            } 
            else if (1 == upd_cmd_204->status) {/*mismatch packet order, but cannot deal the unresponse transmit*/
                RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),
                                   "Receive mismatch packet order: " << upd_cmd_204->current_packet
                                                                     << "resend the correct one!!");
                /*pack and send new data, usually is the lastone*/
                update_firmware_upgrade_params(upd_cmd_204->current_packet);
                set_upd_info(&upd_cmd_203.upd_info, update_option_.module_id);
                upd_cmd_203.status = 0;  //notify upgrade data.
                upd_cmd_203.total_packets = fw_packet.total_packets;
                upd_cmd_203.current_packet = fw_packet.current_packet;
                upd_cmd_203.packet_length = fw_packet.packet_length;
                upd_cmd_203.actual_length = fw_packet.actual_length;
                /*read databuf from file here: fw_packet.read_offset, fw_packet.actual_length*/
                get_firmware_packet_data(upd_cmd_203.data, fw_packet.read_offset, fw_packet.actual_length);
                construct_serial_frame(&serial_frame, UPD_CMD_203, (void *) &upd_cmd_203);
                pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_203 << 8) + MAX_CMD_RESEND_TIME);
                set_cmd_overtime_clock(UPD_CMD_203, MAX_CMD_OVERTIME_VALUE);
            } 
            else if (2 == upd_cmd_204->status) { /*packet data length not ok*//*shold not happen, this is not the deal.*/
                RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(),
                                    "Receive mismatch packet length, correct at design time!!!");
                ota_state->set_report_information(UPDATE_FAILED, firmware_upgrade_flag,
                                                  "Receive mismatch packet length.");
                firmware_upgrade_flag = FIRMWARE_ABORT_UPGRADE;
            } 
            else if (3 == upd_cmd_204->status) {  /*write flash CRC failed*//*error, upgrade failed*/
                RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(),
                                    "Receive writing flash crc failed, abandon firmware upgrade!!!");
                ota_state->set_report_information(UPDATE_FAILED, firmware_upgrade_flag, "write flash crc failed.");
                firmware_upgrade_flag = FIRMWARE_ABORT_UPGRADE;
            } 
            else {/*unkown error*/
                RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(), "Receive UPD_CMD_204 unkown status!!!");
                RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(),
                                    "upd_cmd_204->status: " << upd_cmd_204->status << " upd_cmd_204->current_packet: "
                                                            << upd_cmd_204->current_packet);
                ota_state->set_report_information(UPDATE_FAILED, firmware_upgrade_flag,
                                                  "Receive UPD_CMD_204 unkown status.");
                firmware_upgrade_flag = FIRMWARE_ABORT_UPGRADE;
            }
        }
        break;
        case UPD_CMD_205: {
            clear_resend_cmd(UPD_CMD_205);
            firmware_upgrade_flag = FIRMWARE_WAIT_FINISH_UPDATE;
        }
        break;
        case UPD_CMD_206: {
            clear_cmd_overtime_clock(UPD_CMD_205);
            clear_resend_cmd(UPD_CMD_205);
            //wait for target module normal
            sleep(2);
            //check version after finished.
            upd_cmd_209_type upd_cmd_209 = {0};
            set_upd_info(&upd_cmd_209.upd_info, update_option_.module_id);
            upd_cmd_209.check = 1;  //notice to check.
            construct_serial_frame(&serial_frame, UPD_CMD_209, (void *) &upd_cmd_209);
            pack_send_frame_buf_in_queue(&serial_frame, (UPD_CMD_209 << 8) + MAX_CMD_RESEND_TIME);
            set_cmd_overtime_clock(UPD_CMD_209, MAX_CMD_OVERTIME_VALUE);
            firmware_upgrade_flag = FIRMWARE_WAIT_CHECK_AFTER_VERSION;
        }
        break;
        case UPD_CMD_211: {
            clear_resend_cmd(UPD_CMD_211);
            time_recv_cmd_211 = m_node_ptr->get_clock()->now().seconds();
            RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "time_recv_cmd_211:" << time_recv_cmd_211);
            firmware_upgrade_flag = DELAY_SOME_TIME;
        }
        break;
        default:
        break;
    }
}

void McuOta::module_serial_process() 
{
    find_serial_frame();
    process_received_serial_frames();
    send_serial_frames();
    send_data();
}

int McuOta::getUsedSize(ros_comm_type *data) 
{
    if (NULL == data) {
        return 0;
    }
    int buf_size = sizeof(data->buf);
    int actual_len = (data->end + buf_size - data->start) % buf_size;
    return actual_len;
}
int McuOta::getAvailSize(ros_comm_type *data) 
{
    if (NULL == data) {
        return 0;
    }
    int buf_size = sizeof(data->buf);
    int actual_len = (data->end + buf_size - data->start) % buf_size;
    return (buf_size - actual_len);
}

//--
void McuOta::send_serial_frames() 
{
    static double time_timeout = 0;
    if (send_frame_queue.cnt >= (MAX_SEND_FRAME_QUEUE / 4) * 3) {
        RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "High send_frame_queue level detected!!!");
    }
    while (send_frame_queue.cnt > 0) {
        serial_send_buf_type *frame = &(send_frame_queue.frame[send_frame_queue.head]);
        int resend = send_frame_queue.resend[send_frame_queue.head] & 0xff;
        if(send_frame_queue.state == NORMAL_SEND){
            send_frame_queue.state = resend > 0 ? TRY_RESEND : END_SEND;
            send_serial_flow(frame->buf, frame->cnt);
            RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "send out a frame buf.");
            time_timeout = m_node_ptr->get_clock()->now().seconds();
        }
        else if(send_frame_queue.state == TRY_RESEND){
            if(resend > 0 && (m_node_ptr->get_clock()->now().seconds() - time_timeout) > 1.5){
                time_timeout = m_node_ptr->get_clock()->now().seconds();
                send_serial_flow(frame->buf, frame->cnt);
                send_frame_queue.resend[send_frame_queue.head]--; 
                RCLCPP_DEBUG_STREAM(m_node_ptr->get_logger(), "resend out a frame buf.");
            }
            else if(resend <= 0){
                send_frame_queue.state = END_SEND;
                RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "The proc get response, to complete.");
            }
            else{
                return;
            }
        }
        else if(send_frame_queue.state == END_SEND){
            RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "set no resend. to complete.");
            send_frame_queue.resend[send_frame_queue.head] = 0;
            send_frame_queue.head = (send_frame_queue.head + 1) % MAX_SEND_FRAME_QUEUE;
            send_frame_queue.cnt--;
            send_frame_queue.state = NORMAL_SEND;
        }
        else{
            RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "send_frame_queue state error!");
            send_frame_queue.resend[send_frame_queue.head] = 0;
            send_frame_queue.state = NORMAL_SEND;
        }
    }
}
//发送数据
void McuOta::send_data() 
{
    int send_buf_size = getUsedSize(&ros_send);
    send_buf_size = send_buf_size > 1024 ? 1024 : send_buf_size;
    if (send_buf_size > 0) {
        auto msg = cti_chassis_msgs::msg::UpdateInfo();
        msg.seq_num = seq_num++;
        for (int i = 0; i < send_buf_size; i++) {
            msg.data.push_back(ros_send.buf[ros_send.start]);
            ros_send.start = (ros_send.start + 1) % ROS_COMM_BUF_SIZE;
        }
        manager_pub->publish(msg);
        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),"send msg, seq_num: " << msg.seq_num << " size: " << msg.data.size());
    }
}

int McuOta::send_serial_flow(unsigned char *src, int cnt) 
{
    if (NULL == src) {
        return 0;
    }
    int avail_len = getAvailSize(&ros_send);
    if(cnt > avail_len){
        RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(),"send buf data full.");
        return 0;
    }
    for (int i = 0; i < cnt; i++) {
        ros_send.buf[ros_send.end] = src[i];
        ros_send.end = (ros_send.end + 1) % ROS_COMM_BUF_SIZE;
    }
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),"ros_buffer_write size: " << cnt << 
                                                ",ros_send start: " << ros_send.start << 
                                                ",end: " << ros_send.end);
    return cnt;
}
//当前时间
std::string McuOta::getLocalDate() 
{
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S", localtime(&timep));
    return tmp;
}

//user string to id 
int McuOta::check_user_update_option(std::string input_module) 
{
    for (int i = 0; i < MODULE_PARAM_TABLE_ELE_NUM; i++) {
        if (input_module == module_param_table[i].module_name) {
            return module_param_table[i].module_id;
        }
    }
    return -1;
}

int McuOta::run() 
{
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "**************MCU OTA START*************");
    Info("**************MCU OTA START*************");
    ota_state->upgrade_report.update_time = getLocalDate();
    if (4 != input_parameters_.size()) {
        RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(), "Input param error.");
        Info("Input param error.");
        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),
                           "--Usage: rosrun mcu_ota mcu_ota PORT_NAME MAIN_MODULE_NAME MODULE_NAME BIN_FILE_NAME");
        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "PORT_NAME: force to DEFAULT_PORT");
        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "MAIN_MODULE_NAME: main module name to upgrade");
        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "MODULE_NAME: module name to upgrade");
        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "MAIN_MODULE_NAME and MODULE_NAME belongs to the list:");
        for (int i = 0; i < MODULE_PARAM_TABLE_ELE_NUM; i++) {
            RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), module_param_table[i].module_name);
        }
        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "BIN_FILE_NAME: the bin file for update.");
        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "exit mcu_ota!");
        Info("exit mcu_ota!");
        return UPDATE_FAILED;
    }
    update_option_.main_module_name = input_parameters_[1];
    update_option_.module_name = input_parameters_[2];
    update_option_.bin_file_name = input_parameters_[3];
    //--
    update_option_.main_module_id = check_user_update_option(update_option_.main_module_name);
    if (update_option_.main_module_id < 0) {
        RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(), "User input main module name invalid, exit mcu_ota!");
        Info("User input main module name invalid, exit mcu_ota!");
        return UPDATE_FAILED;
    }
    update_option_.module_id = check_user_update_option(update_option_.module_name);
    if (update_option_.module_id < 0) {
        RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(), "User input module name invalid, exit mcu_ota!");
        Info("User input module name invalid, exit mcu_ota!");
        return UPDATE_FAILED;
    }
    if (access(update_option_.bin_file_name.c_str(), F_OK) != 0){
        RCLCPP_ERROR_STREAM(m_node_ptr->get_logger(), "User input bin file no exists, exit mcu_ota!");
        Info("User input bin file no exists, exit mcu_ota!");
        return UPDATE_FAILED;
    }
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "-------------input parameters list-------------");
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "main_module_name: " << update_option_.main_module_name);
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "main_module_id: " << update_option_.main_module_id);
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "module_name: " << update_option_.module_name);
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "module_id: " << update_option_.module_id);
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "bin_file_name: " << update_option_.bin_file_name);
    Info("-------------input parameters list-------------");
    Info("main_module_name: " << update_option_.main_module_name);
    Info("main_module_id: " << update_option_.main_module_id);
    Info("module_name: " << update_option_.module_name);
    Info("module_id: " << update_option_.module_id);
    Info("bin_file_name: " << update_option_.bin_file_name);

    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "Waiting for ros node sync, start in 3 seconds");
    for (int i = 0; i < 3; i++) {
        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "Start in " << (3 - i) << " seconds");
        sleep(1);
    }
    rclcpp::Rate loop_rate(200);
    int manage_module_update_ret;
    update_start_time = m_node_ptr->get_clock()->now().seconds();
    while (rclcpp::ok()) {
        manage_module_update_ret = manage_module_updates(update_option_.module_id, update_option_.main_module_id);
        if (UPDATE_ING != manage_module_update_ret) {
            break;
        }
        module_serial_process();
        rclcpp::spin_some(m_node_ptr);
        loop_rate.sleep();
    }
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "manage_module_update_ret: " << manage_module_update_ret);
    return manage_module_update_ret;
}
