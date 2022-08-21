from launch import LaunchDescription
import launch_ros.actions
import ament_index_python

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='cti_chassis_handdle',
            executable='cti_chassis_handdle_node',
            parameters=[
              {'port_name': '/dev/ttyUSB0'},
              {'port_type': 'serial'},
              {'udp_ip': '192.168.1.102'},
              {'udp_port': 8887},
              {'udp_ip_dest': '192.168.1.222'},
              {'udp_port_dest': 8888},
              {'filenamelog': ament_index_python.get_package_prefix("cti_chassis_handdle") + '/control.log'},
              {'LIN_ult_installed': 1},
              {'robot_type': 1},
              {'car_wheel_base': 0.8},
              {'timer2_duration': 1.0},
              {'cmd_answer_timeout': 0.02},
              {'version_topic': '/cti/fpga_serial/operationControlVersion'},
              {'cmd_answer_topic': '/cti/fpga_serial/cmd_answer_cnt'},
              {'max_control_board_version_head': 3},
              {'min_control_board_version_head': 1},
              {'max_control_board_version_mid': 1},
              {'min_control_board_version_mid': 8},
              {'max_control_board_version_end': 5},
              {'min_control_board_version_end': 0},
              {'lift_test_switch': False},
              {'default_sweeper_id': 0},
              {'default_dustbox_fanspeed': 80},
              {'default_damboard_control': 1},
              {'default_side_brush_transform': 50},
              {'default_side_brush_speed': 70},
              {'water_out_per_sec': 0.0139},
              {'water_in_per_sec': 0.2},
              {'localization_limit': True},
              {'obstatle_disobs_limit': True},
              {'chassis_chat_timeout_secs': 10.0},
              {'box_chat_timeout_secs': 10.0},
              {'msg_max_range': 1.5},
              {'CTI_RUN_VER': 'v7.0'},
            ],
            output='screen',
            respawn=True

        ),

        launch_ros.actions.Node(
            package='cti_chassis_data',
            executable='cti_chassis_data_node',
            parameters=[
            {'filenamelog': ament_index_python.get_package_prefix("cti_chassis_data") + '/chassis_data.log'},
              {'ult_min_range': 0.25},
              {'ult_max_range': 2.5},
              {'rear_ult_detect_offset': 0.05},
              {'rear_ult_detect_max': 2.0},
              {'rear_ult_detect_min': 0.5},
              {'front_ult_detect_offset': 0.05},
              {'front_ult_detect_max': 2.0},
              {'front_ult_detect_min': 0.5},
              {'vehicle_width': 1200.0},
              {'rear_ult_dist': 620.0},
              {'front_ult_dist': 820.0},
              {'dustbox_LIN_ult_installed': 0},
              {'msg_max_range': 1.5},
              {'kalman_enable': True},
              {'kalman_R': 0.2},
              {'kalman_Q': 0.01},
            ],
            output='screen',
        ),

    ])
