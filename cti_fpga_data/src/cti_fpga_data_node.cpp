#include <cti_fpga_data/cti_fpga_data.h>
//--KN:name in log
//constexpr char const* kN = "fpga-data";
static void SigsHandler(int sig)
{
    ros::shutdown();
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
    ros::init(argc, argv, "cti_fpga_data_node");
    init_signal();
    CtiFpgaData cfd;
    ros::Rate loop_rate(50);
    //----------------------------
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    Error("cti_fpga_data_node quit!");
    return 0;
}

