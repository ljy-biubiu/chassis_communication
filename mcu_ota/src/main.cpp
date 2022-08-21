#include "mcu_ota.hpp"

static void SigsHandler(int sig) 
{
    rclcpp::shutdown();
}

void init_signal() 
{
    ::signal(SIGCHLD, SIG_IGN);// SIGCHLD
    ::signal(SIGINT, SigsHandler);// ^ + C
    ::signal(SIGTERM, SigsHandler);// 请求中断
    ::signal(SIGKILL, SigsHandler);// 强制中断
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto mcu_ota_node = std::make_shared<rclcpp::Node>("mcu_ota_node");

    for (int i = 0; i < argc; i++) {
        RCLCPP_INFO_STREAM(mcu_ota_node->get_logger(), "input: " << i << " " << argv[i]);
    }
    std::vector <std::string> parameters;
    std::string parameter;
    for (int i = 1; i < argc; i++) {
        parameter = argv[i];
        parameters.push_back(parameter);
    }
    init_signal();
    McuOta MO(mcu_ota_node, parameters);
    int ret = MO.run();
    RCLCPP_INFO_STREAM(mcu_ota_node->get_logger(), "mcu_ota_node quit! ret: " << ret);
    return ret; //UPDATE_FAILED 1; UPDATE_SUCCESSED 2
}


