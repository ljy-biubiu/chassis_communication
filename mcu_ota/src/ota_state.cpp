#include "ota_state.hpp"

OtaState::OtaState(std::shared_ptr<rclcpp::Node> node_ptr) : m_node_ptr(node_ptr) 
{
}

void OtaState::set_report_information(int result, int Estage, std::string Ecode) 
{
    upgrade_report.result = result;
    upgrade_report.Estage = Estage;
    upgrade_report.Ecode = Ecode;
    if (upgrade_report.result == UPDATE_FAILED) {
        if (upgrade_report.Estage < GET_ARRAY_ELE_NUM(Control_fail_stage_string)) {
            upgrade_report.EStateMessage = (char *) Control_fail_stage_string[upgrade_report.Estage];
        } else {
            upgrade_report.EStateMessage = "unknown stage: " + std::to_string(upgrade_report.Estage);
        }
    }
}

int OtaState::report_module_update_status() 
{
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),
                       "**********************Report update module firmware results*************************");
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "main module name: " << upgrade_report.main_module_name);
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "main module id: " << upgrade_report.main_module_id);
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "module name: " << upgrade_report.module_name);
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "module id: " << upgrade_report.module_id);
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "update bin file name: " << upgrade_report.bin_file_name);
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "update time: " << upgrade_report.update_time);
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "update dura time: " << upgrade_report.update_dura_time << " seconds");
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "update result: " << ((upgrade_report.result == UPDATE_SUCCESSED) ? "SUCCESS" : "FAIL"));
    int ret = (upgrade_report.result != UPDATE_FAILED) ? 0 : -1;
    if (UPDATE_FAILED == upgrade_report.result) {
        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "Fail Estage: " << upgrade_report.Estage);
        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "Fail EStateMessage: " << upgrade_report.EStateMessage);
        RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "Fail Ecode: " << upgrade_report.Ecode);
    }
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "APP version before update : " << upgrade_report.app_version_before);
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(), "APP version after update : " << upgrade_report.app_version_after);
    RCLCPP_INFO_STREAM(m_node_ptr->get_logger(),
                       "**********************Report update module firmware results*************************");
    return ret;
}
