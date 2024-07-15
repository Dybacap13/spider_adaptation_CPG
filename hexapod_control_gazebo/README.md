**_hexapod_controller_gazebo_**

Данная нода отвечает за публикацию расчитанных углов в симулятор газебо. Необходимо запускать, если используется симулятор Gazebo. 

*Subscribed Topics*

     joint_states_to_gazebo (sensor_msgs::JointState) - считывает рассчитанное положение суставов и публикует в Gazebo

*Published Topics*

     /spider/j_c1_lf_position_controller/command
     /spider/j_c1_lm_position_controller/command
     /spider/j_c1_lr_position_controller/command
     /spider/j_c1_rf_position_controller/command
     /spider/j_c1_rm_position_controller/command
     /spider/j_c1_rr_position_controller/command
     /spider/j_thigh_lf_position_controller/command
     /spider/j_thigh_lm_position_controller/command
     /spider/j_thigh_lr_position_controller/command
     /spider/j_thigh_rf_position_controller/command
     /spider/j_thigh_rm_position_controller/command
     /spider/j_thigh_rr_position_controller/command
     /spider/j_tibia_lf_position_controller/command
     /spider/j_tibia_lm_position_controller/command
     /spider/j_tibia_lr_position_controller/command
     /spider/j_tibia_rf_position_controller/command
     /spider/j_tibia_rm_position_controller/command
     /spider/j_tibia_rr_position_controller/command

Для движения какого-то отдельного сустава можно опубликовать значения угла ( в радианах ) в соответствующий топик
