#include "cti_chassis_handdle/cti_chassis_handdle.hpp"

//************************************** 主函数
int main(int argc, char **argv) {

  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  std::shared_ptr<ChassisCmncBase> chassisCmncBase =
      std::make_shared<ChassisCmncReal>();
  rclcpp::spin(std::make_shared<ChassisHanddle>(chassisCmncBase));
  rclcpp::shutdown();
  return 0;
}
