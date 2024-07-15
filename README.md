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

### Шаги по запуску 

* Необходимые ( думаю все в ланчи объединить, но пока так )
  
     1. `roslaunch spider_gazebo srider_xacro.launch`
     2. `rosrun hexapod_controller hexapod_controller_gazebo`
     3. `rosrun hexapod_controller hexapod_controller`
     4. `rosrun reward_learning_gyro calculator_reward_gyro`
     5. `rosrun reward_learning_odom calculator_reward_odom`
        
* Если нужно, чтоб робот просто походил
  
     1. Публикуем в топик  `/state(std_msgs/Bool)` - true
     2. Публикуем в топик  `/cmd_vel(geometry_msgs/Twist)` - скорость
        
* Алгоритм обучения
     1. `rosrun move_feet move_feet`
     2. Включить симулятор ( там в газебо кнопка запуска)
     3. Публикуем в топик  `/state(std_msgs/Bool)` - true
     4. Публикуем в топик  `/move_feet/move_mode(std_msgs/Bool)` - true
     5. `rosrun reinforcement_learning_shooter reinforcement_learning`
        
* Алгоритм адаптации
      
     1. `rosrun move_feet_adaptation move_feet_adaptation`
     2. Включить симулятор ( там в газебо кнопка запуска)
     3. Публикуем в топик  `/state(std_msgs/Bool)` - true
     4. Публикуем в топик  `/move_feet/move_mode(std_msgs/Bool)` - true
     5. `rosrun adaptation_algorithm adaptation_algorithm`
        
Для запуска других алгоритмов ( или их модификаций ) поменять пункт 5 на соответствующий:

* `rosrun adaptation_multi_armed_bandit adaptation_multi_armed_bandit`
* `rosrun multi_armed_bandit multi_armed_bandit`
* `rosrun reinforcement_learning reinforcement_learning`

     
### Описание пакетов (более подробную информацию см в пакете)

**_hexapod_controller_**

Главная нода, отвечает за расчёт обратной кинематики, генерирование триподной походки. В данной ноде происходит именно расчёт углов для сервоприводов, но не публикация в симулятор Gazebo. Эту ноду необходимо запускать при любых тестах.

    
**_hexapod_controller_gazebo_**

Данная нода отвечает за публикацию расчитанных углов в симулятор газебо. Необходимо запускать, если используется симулятор Gazebo. 


**_spider_description_**

Хранит модель робота-гексапода и его конфиги.

**_adaptation_algorithm_**

Алгоритм адаптации CPG ( язык C++ )

**_adaptation_multi_armed_bandit_**

Классический эпсилон-жадный алгоритм адаптации ( язык Python)

**_move_feet_**

Здесь хранится сервис, который реализует расчёт цикла движения робота при алгоритме `обучения`. Здесь нет публикатора в симулятор, только расчёт! Необходимо включать пакет `hexapod_controller_gazebo`

**_move_feet_adaptation_**

Здесь хранится сервис, который реализует расчёт цикла движения робота при алгоритме `адаптации`. Здесь нет публикатора в симулятор, только расчёт! Необходимо включать пакет `hexapod_controller_gazebo`


**_multi_armed_banditn_**

Классический эпсилон-жадный алгоритм обучения ( язык Python )

**_reinforcement_learning_**

Алгоритм обучения CPG без модификаций, классический

**_reinforcement_learning_shooter_**

Алгоритм обучения CPG с возбуждающим нейронов

**_reward_learning_**

* reward_learning_gyro - расчёт награды по гироскопу ( и адаптация, и обучения )
* reward_learning_odom - расчёт награды по одометрии ( и адаптация, и обучения )
  






