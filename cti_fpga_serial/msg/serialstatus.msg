# topic to publish serialstatus
# data: flag for the all status  
# others: 0:ok -1:bad
Header  header
uint16  data
int8   recv_ctrl
int8   recv_ctrl_rate
int8   recv_rfid
int8   recv_ult
int8   recv_cmd_answer
int8   recv_imu_nan
int8   recv_pthread_recv
int8   recv_pthread_crc
int8   control_board_version
int8   control_clean_mechine
int8   recv_hall_turn_front_left
int8   recv_hall_turn_front_center
int8   recv_hall_turn_front_right
int8   recv_hall_turn_rear_left
int8   recv_hall_turn_rear_center
int8   recv_hall_turn_rear_right
int8   recv_box_pos_reach1
int8   recv_box_pos_reach2
int8   recv_box_pos_reach3
int8   recv_lift_pressure
int8   send_localizer
#环卫车清扫功能监测兼容
uint16  recv_from_dustbin_state_old_cnt
uint16  callback_dustbin_cnt_old_cnt
uint16  recv_from_dustbin_state_new_cnt
uint16  callback_dustbin_cnt_new_cnt
uint16  dustbin_state_old_size
uint16  dustbin_state_new_size
uint16  dustbin_state_recv_size
#吸尘箱兼容检测
uint16  recv_from_dustbox_state_old_cnt
uint16  callback_dustbox_cnt_old_cnt
uint16  recv_from_dustbox_state_new_cnt
uint16  callback_dustbox_cnt_new_cnt
uint16  dustbox_state_old_size
uint16  dustbox_state_new_size
uint16  dustbox_state_recv_size