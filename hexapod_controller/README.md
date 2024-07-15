## Запуск

Запускается всегда

     rosrun hexapod_controller hexapod_controller


**_hexapod_controller_**

Главная нода, отвечает за расчёт обратной кинематики, генерирование триподной походки. В данной ноде происходит именно расчёт углов для сервоприводов, но не публикация в симулятор Gazebo. Эту ноду необходимо запускать при любых тестах.

*Subscribed Topics*

     cmd_vel (geometry_msgs/Twist)  - задание скорости движения гексапода
     state (std_msgs::Bool)  - true -робот встанет
     
*Published Topics*

    joint_states_to_gazebo (sensor_msgs::JointState) - публикует рассчитанное положение суставов.


**_control_**

*Published Topics*

     /odometry/calculated - расчёт одометрии
     /twist - показания скорости 


* gait  - расчёт походки
* ik - обратная кинематика, расчёт углов
* servo_driver - драйвер для сервоприводов ( не использется, если работаете в симуляторе)



