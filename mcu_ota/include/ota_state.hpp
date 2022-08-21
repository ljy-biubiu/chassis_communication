#ifndef OTA_STATE_HPP
#define OTA_STATE_HPP

#include "common_type.hpp"
#include "common_value.hpp"
#include "cmd_type.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>

class OtaState {
public:
    OtaState(std::shared_ptr<rclcpp::Node> node_ptr);
    upgrade_report_type upgrade_report;
    void set_report_information(int result, int Estage, std::string Ecode);
    int  report_module_update_status();
private:
    std::shared_ptr<rclcpp::Node> m_node_ptr;
};

#endif