#include <cti_chassis_data/cti_chassis_data.h>
//--KN:name in log
//constexpr char const* kN = "fpga-data";
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
    init_signal();
    rclcpp::spin(std::make_shared<CtiFpgaData>());
    rclcpp::shutdown();

    Error("cti_fpga_data_node quit!");
    return 0;
}