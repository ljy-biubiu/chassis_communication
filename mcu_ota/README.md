这是迁移的之前ros1下的mcu_ota代码

根据3.0的升级需求取消掉了之前的大小驱动更新时的特殊流程，目前大小驱动更新流程和其他模块一样，预留了滚刷板子的更新地址20，暂时屏蔽掉了发布升级状态的代码，后续有需要可以打开，但是要添加对应的msg。

使用命令：ros2 run mcu_ota mcu_ota_node DEFAULT_PORT MODULE_MOVE_CONTROL_BOARD MODULE_MOTOR_DRIVER_FRONT_LEFT DRVB.bin(根据要升级的板子更改该命令)
                                                                                需要升级的模块名称             升级文件的路径
                                                                                
后续需要提供给嵌入式新的升级脚本，估计只要把旧脚本中的命令替换掉即可，然后需要嵌入式记得把升级文件传到相应位置并进入ade中运行脚本
