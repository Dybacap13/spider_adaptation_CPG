## Данный репозиторий включает в себя: модель робота-гексапода, управление контроллерами, алгоритм обучения и адаптации на основе CPG (центрального генератора паттернов)

### Вдохновители
<a href=https://github.com/HumaRobotics/phantomx_gazebo/tree/master>PhantomX</a>
<br> </n>
<a href=https://github.com/KevinOchs/hexapod_ros>Golem</a>

### Зависимости

     Ubuntu 20.04 
     ros noetic 


### Установка необходимых пакетов
`rosdep install --from-paths src --ignore-src -r -y`

### Описание пакетов


**_hexapod_controller_**

Главная нода, отвечает за расчёт обратной кинематики, генерирование триподной походки. В данной ноде происходит именно расчёт углов для сервоприводов, но не публикация в симулятор Gazebo. Эту ноду необходимо запускать при любых тестах.

*Subscribed Topics*

     cmd_vel (geometry_msgs/Twist)  - задание скорости движения гексапода
     state (std_msgs::Bool)  - true -робот встанет
     
*Published Topics*

    joint_states_to_gazebo (sensor_msgs::JointState) - публикует рассчитанное положение суставов.
    
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


**_spider_description_**

Хранит модель робота-гексапода и его конфиги.





