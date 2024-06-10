#include <move_feet.h>

// RR -> 0
// RM -> 1
// RF -> 2
// LR -> 3
// LM -> 4
// LF -> 5

MoveFeet::MoveFeet(ros::NodeHandle nh_) : nh(nh_) {
  ros::param::get("FEMUR_ANGLE", FEMUR_ANGLE);
  ros::param::get("FEMUR_AXIS", FEMUR_AXIS);
  ros::param::get("COXA_AXIS", COXA_AXIS);
  ros::param::get("INTERPOLATION_COEFFICIENT", INTERPOLATION_COEFFICIENT);
  ros::param::get("DELTA", DELTA);
  ros::param::get("NUMBER_OF_LEGS", NUMBER_OF_LEGS);

  move_feet_mode_sub = nh_.subscribe("/move_legs/mode", 1,
                                     &MoveFeet::moveFeetModeCallback, this);

  joint_states_sub = nh_.subscribe("/joints_to_gazebo", 1,
                                   &MoveFeet::jointStatesCallback, this);
  joint_states_pub =
      nh_.advertise<sensor_msgs::JointState>("/joints_to_gazebo", 1000);

  block_feet_sub =
      nh_.subscribe("/move_legs/block", 1, &MoveFeet::blockCallback, this);
  zero_leg_sub =
      nh_.subscribe("/move_legs/zero_leg", 1, &MoveFeet::zeroLegCallback, this);
  zero_leg_reset_sub = nh_.subscribe("/move_legs/zero_leg_reset", 1,
                                     &MoveFeet::zeroLegResetCallback, this);

  client_gyro_learning = nh_.serviceClient<hexapod_msgs::RewardGyro>(
      "/calculator_reward_gyroscope");
  client_odom_learning = nh_.serviceClient<hexapod_msgs::RewardOdom>(
      "/calculator_reward_odometry");

  client_gyro_adaptation = nh_.serviceClient<hexapod_msgs::RewardGyro>(
      "/adaptation_reward_gyroscope");
  client_odom_adaptation = nh_.serviceClient<hexapod_msgs::RewardOdom>(
      "/adaptation_reward_odometry");

  service_check =
      nh_.advertiseService("/check_gyroscope", &MoveFeet::check_service, this);

  service_ = nh_.advertiseService("/move_feet_learning",
                                  &MoveFeet::init_service, this);
  ROS_INFO("MoveFeet ready");
}

bool MoveFeet::init_service(hexapod_msgs::MoveFeetLearning::Request &req,
                            hexapod_msgs::MoveFeetLearning::Response &res) {
  int count = 0;
  int REWARD_KOEF = 2;
  hexapod_msgs::MoveFeet reverce;
  ros::ServiceClient client_odom = client_odom_learning;
  ros::ServiceClient client_gyro = client_gyro_learning;

  // получаем вектор ног и скорость
  auto move_feet = req;
  cmd_vel = move_feet.legs.cmd_vel;
  std::cout << "________________" << std::endl;
  std::cout << "current_time = " << move_feet.current_time << std::endl;
  if (move_feet.need_adaptation) {
    client_odom = client_odom_adaptation;
    client_gyro = client_gyro_adaptation;
    REWARD_KOEF = 0;
  }

  // запрос на получение награды от ГИРОСКОПА № 1 !!!
  hexapod_msgs::RewardGyro srv_gyro_1;
  srv_gyro_1.request.legs = move_feet.legs;
  srv_gyro_1.request.current_time = move_feet.current_time;

  // здесь мы считаем ноги и создаём инвентированную команду
  for (auto number_leg = 0; number_leg < move_feet.legs.legs.size();
       number_leg++) {
    last_command[number_leg] = move_feet.legs.legs[number_leg];
    if (block[number_leg] == true) last_command[number_leg] = false;

    if (last_command[number_leg]) {
      count++;
      reverce.legs[number_leg] = false;

    } else {
      reverce.legs[number_leg] = true;
    }

    if (block[number_leg] == true) {
      reverce.legs[number_leg] = false;
    }

    std::cout << last_command[number_leg] << "   ";
  }
  std::cout << " " << std::endl;
  ;

  std::cout << "count_true = " << count << std::endl;
  ;

  std::vector<bool> reverse_command = {!last_command[0], !last_command[1],
                                       !last_command[2], !last_command[3],
                                       !last_command[4], !last_command[5]};

  if (yes_zero_legs == true) {
    last_command[zero_legs] = false;
    reverse_command[zero_legs] = false;
    reverce.legs[zero_legs] = false;
    std::cout << "ZERO = " << zero_legs << std::endl;
  }

  // НАЧИНАЕМ

  // 1 Поднимаем ноги
  interpolationOfAngles(current_state, upLegs(last_command));
  ros::Duration(3.0).sleep();

  // отправляем запрос ГИРОСКОПУ НОМЕР 1

  if (!client_gyro.call(srv_gyro_1)) {
    std::cout << "Failed to call service /calculator_reward_gyroscope"
              << std::endl;
    interpolationOfAngles(current_state, downLegs(last_command));
    res.result = "error";
    res.reward_general = 0.0;
    res.reward_odometry = 0.0;
    res.reward_gyroscope = 0.0;
    return true;
  }

  std::cout << "reward_gyroscope_1 = " << srv_gyro_1.response.reward_gyroscope
            << std::endl;

  // Награда от одометрии 1
  hexapod_msgs::RewardOdom srv_odom1;
  srv_odom1.request.count_legs = count;
  srv_odom1.request.current_time = move_feet.current_time;

  if (!client_odom.call(srv_odom1)) {
    std::cout << "Failed to call service /calculator_reward_odometry"
              << std::endl;
    res.result = "error";
    res.reward_general = 0.0;
    res.reward_odometry = 0.0;
    res.reward_gyroscope = 0.0;
    return true;
  }

  if (srv_odom1.response.result == "wait") {
    ROS_INFO("INIT SERVER COMPLETE");
    client_odom.call(srv_odom1);
  }

  // получаем награду одометрии
  reward_odometry = srv_odom1.response.reward_odometry;

  if (count == 0) {
    res.reward_general = srv_odom1.response.reward_odometry;
    res.reward_odometry = srv_odom1.response.reward_odometry;
    res.reward_gyroscope = 0.0;
    res.result = "zero";
    return true;
  }

  // Если мы отклонились на первом этапе
  if (srv_gyro_1.response.reward_gyroscope <= REWARD_KOEF) {
    // сщхраняем награду ( пусть будет_
    reward_gyroscope = srv_gyro_1.response.reward_gyroscope;
    interpolationOfAngles(current_state, downLegs(last_command));

    // отправляем награду обучению
    res.reward_general = srv_gyro_1.response.reward_gyroscope +
                         srv_odom1.response.reward_odometry;
    res.reward_odometry = srv_odom1.response.reward_odometry;
    res.reward_gyroscope = srv_gyro_1.response.reward_gyroscope;
    res.result = srv_gyro_1.response.result;

    return true;
  }

  std::cout << "HORM" << std::endl;
  // все норм на ПЕРВОМ этапе мы устояли

  // двигаем лапки
  interpolationOfAngles(current_state, moveLegs(last_command));
  interpolationOfAngles(current_state, downLegs(last_command));

  // теперь поднимаем ОБРАТНЫЕ лапки

  //************************
  hexapod_msgs::RewardGyro srv_gyro_2;
  if (!move_feet.need_adaptation || yes_zero_legs == true) {
    interpolationOfAngles(current_state, upLegs(reverse_command));
    ros::Duration(3.0).sleep();

    // отправляем запрос на получение награды ГИРОСКОПА - 2

    srv_gyro_2.request.legs = reverce;
    srv_gyro_2.request.current_time = move_feet.current_time;

    if (!client_gyro.call(srv_gyro_2)) {
      std::cout << "Failed to call service /calculator_reward_gyroscope"
                << std::endl;
      interpolationOfAngles(current_state, downLegs(reverse_command));
      res.result = "error";
      res.reward_general = 0.0;
      res.reward_odometry = 0.0;
      res.reward_gyroscope = 0.0;
      return true;
    }
  }
  //************************

  // Награда от одометрии 2
  hexapod_msgs::RewardOdom srv_odom2;
  srv_odom2.request.count_legs = count;
  srv_odom2.request.current_time = move_feet.current_time;

  if (!client_odom.call(srv_odom2)) {
    std::cout << "Failed to call service /calculator_reward_odometry"
              << std::endl;
    res.result = "error";
    res.reward_general = 0.0;
    res.reward_odometry = 0.0;
    res.reward_gyroscope = 0.0;
    return true;
  }

  if (srv_odom2.response.result == "wait") {
    ROS_INFO("INIT SERVER COMPLETE");
    client_odom.call(srv_odom2);
  }

  // получаем награду одометрии
  reward_odometry = srv_odom2.response.reward_odometry;

  //************************
  if (!move_feet.need_adaptation || yes_zero_legs == true) {
    // Если мы отклонились на ВТОРОМ этапе
    if (srv_gyro_2.response.reward_gyroscope <= REWARD_KOEF) {
      // сщхраняем награду ( пусть будет_
      reward_gyroscope = srv_gyro_2.response.reward_gyroscope;

      interpolationOfAngles(current_state, downLegs(reverse_command));
      interpolationOfAngles(current_state, reverseTrueLegs(last_command));

      // отправляем награду обучению
      res.reward_general = srv_gyro_2.response.reward_gyroscope +
                           srv_odom2.response.reward_odometry;
      res.reward_odometry = srv_odom2.response.reward_odometry;
      ;
      res.reward_gyroscope = srv_gyro_2.response.reward_gyroscope;
      res.result = srv_gyro_2.response.result;
      return true;
    }
  }
  //************************
  std::cout << "HORM 2" << std::endl;

  // Все норм мы устояли на втором этапе!!!
  // двигаем обратно лапки

  interpolationOfAngles(current_state, reverseTrueLegs(last_command));
  std::cout << " Duration" << std::endl;
  // ros::Duration(3.0).sleep();

  //************************
  if (!move_feet.need_adaptation || yes_zero_legs == true) {
    interpolationOfAngles(current_state, downLegs(reverse_command));
  }
  //************************

  // ЗДЕСЬ МЫ ПРОСИМ НАГРАДУ У ОДОМЕТРИИ !!
  hexapod_msgs::RewardOdom srv_odom;
  srv_odom.request.count_legs = count;
  srv_odom.request.current_time = move_feet.current_time;

  if (!client_odom.call(srv_odom)) {
    std::cout << "Failed to call service /calculator_reward_odometry"
              << std::endl;
    res.result = "error";
    res.reward_general = 0.0;
    res.reward_odometry = 0.0;
    res.reward_gyroscope = 0.0;
    return true;
  }

  if (srv_odom.response.result == "wait") {
    ROS_INFO("INIT SERVER COMPLETE");
    client_odom.call(srv_odom);
  }

  // получаем награду одометрии
  reward_odometry = srv_odom.response.reward_odometry;

  //************************
  if (!move_feet.need_adaptation || yes_zero_legs == true) {
    reward = srv_odom.response.reward_odometry +
             srv_gyro_2.response.reward_gyroscope;
    reward_odometry = srv_odom.response.reward_odometry;
    reward_gyroscope = srv_gyro_2.response.reward_gyroscope;
    res.result = srv_gyro_2.response.result;

  } else {
    reward = srv_odom.response.reward_odometry +
             srv_gyro_1.response.reward_gyroscope;
    reward_odometry = srv_odom.response.reward_odometry;
    reward_gyroscope = srv_gyro_1.response.reward_gyroscope;
    res.result = srv_gyro_1.response.result;
  }
  //************************

  res.reward_general = reward;
  res.reward_odometry = reward_odometry;
  res.reward_gyroscope = reward_gyroscope;

  std::cout << "reward_gyroscope = " << reward_gyroscope << std::endl;
  std::cout << "reward_odometry = " << reward_odometry << std::endl;
  std::cout << "reward = " << reward << std::endl;
  return true;
}

void MoveFeet::moveFeetModeCallback(std_msgs::Bool msg) {
  if (!current_state_bool) {
    ROS_WARN("Wait while the current angles are calculated");
    return;
  }
  move_feet_mode = msg.data;
}

void MoveFeet::jointStatesCallback(sensor_msgs::JointState msg) {
  current_state = msg;
  current_state_bool = true;
}

void MoveFeet::interpolationOfAngles(sensor_msgs::JointState current,
                                     sensor_msgs::JointState target) {
  sensor_msgs::JointState interpolation_angles_pub;
  interpolation_angles_pub = current;

  while (ros::ok()) {
    if (comparisonJointStates(
            target, interpolation_angles_pub))  // цикл прекратится, когда
                                                // старт. значение = желаемому
      break;

    for (int i = 0; i < current_state.name.size(); i++) {
      interpolation_angles_pub.position[i] =
          target.position[i] * INTERPOLATION_COEFFICIENT +
          interpolation_angles_pub.position[i] *
              (1 - INTERPOLATION_COEFFICIENT);
    }

    jointStatesPublisher(interpolation_angles_pub);

    current_state = interpolation_angles_pub;
  }
}

bool MoveFeet::comparisonJointStates(sensor_msgs::JointState first,
                                     sensor_msgs::JointState second) {
  if (first.name.size() != second.name.size()) {
    ROS_ERROR("ERROR IN comparisonJointStates ");
    // ros::Duration(1.0).sleep();
    return false;
  }

  for (int i = 0; i < first.name.size(); i++) {
    if (abs(first.position[i] - second.position[i]) > DELTA) {
      return false;
    }
  }
  return true;
}

void MoveFeet::jointStatesPublisher(sensor_msgs::JointState msg_pub) {
  joint_states_pub.publish(msg_pub);
  ros::Duration(0.1).sleep();
}



sensor_msgs::JointState MoveFeet::upLegs(std::vector<bool> command) {
  sensor_msgs::JointState target_state = current_state;
  std::cout << "1) upLegs" << std::endl;

  for (auto number_leg = 0; number_leg < NUMBER_OF_LEGS; number_leg++) {
    if (!command[number_leg]) {
      continue;
    }

    // здесь код для ног true
    // поднимаем
    target_state.position[number_leg * 3 + 1] =
        current_state.position[number_leg * 3 + 1] +
        (FEMUR_ANGLE * FEMUR_AXIS[number_leg]);
  }

  return target_state;
}

sensor_msgs::JointState MoveFeet::moveLegs(std::vector<bool> command) {
  sensor_msgs::JointState target_state = current_state;
  std::cout << "2) moveLegs" << std::endl;

  for (auto number_leg = 0; number_leg < NUMBER_OF_LEGS; number_leg++) {
    if (!command[number_leg]) {
      continue;
    }

    // здесь двигаем coxa
    target_state.position[number_leg * 3] =
        current_state.position[number_leg * 3] +
        (cmd_vel * COXA_AXIS[number_leg]);
  }

  return target_state;
}

sensor_msgs::JointState MoveFeet::downLegs(std::vector<bool> command) {
  sensor_msgs::JointState down_leg = current_state;
  std::cout << "3) downLegs" << std::endl;
  for (auto number_leg = 0; number_leg < NUMBER_OF_LEGS; number_leg++) {
    if (command[number_leg]) {
      down_leg.position[number_leg * 3 + 1] =
          current_state.position[number_leg * 3 + 1] -
          (FEMUR_ANGLE * FEMUR_AXIS[number_leg]);
    }
  }

  return down_leg;
}

sensor_msgs::JointState MoveFeet::reverseTrueLegs(std::vector<bool> command) {
  sensor_msgs::JointState target_state = current_state;
  std::cout << "4) reverseTrueLegs" << std::endl;
  for (auto number_leg = 0; number_leg < NUMBER_OF_LEGS; number_leg++) {
    if (command[number_leg]) {
      target_state.position[number_leg * 3] =
          current_state.position[number_leg * 3] -
          (cmd_vel * COXA_AXIS[number_leg]);
    }
  }

  return target_state;
}

void MoveFeet::blockCallback(hexapod_msgs::MoveFeet msg) {
  sensor_msgs::JointState block_leg = current_state;
  for (auto number_leg = 0; number_leg < msg.legs.size(); number_leg++) {
    block[number_leg] = msg.legs[number_leg];
  }
  for (auto number_leg = 0; number_leg < NUMBER_OF_LEGS; number_leg++) {
    if (block[number_leg]) {
      block_leg.position[number_leg * 3 + 1] =
          current_state.position[number_leg * 3 + 1] +
          (1 * FEMUR_AXIS[number_leg]);
    }
  }
  interpolationOfAngles(current_state, block_leg);
}

bool MoveFeet::check_service(hexapod_msgs::CheckGait::Request &req,
                             hexapod_msgs::CheckGait::Response &res) {
  std::cout << "_______________Check Gait_______________" << std::endl;
  // получаем
  if (!req.need_check) {
    res.result = false;
    res.result_str = "refusal";
    return true;
  }

  hexapod_msgs::MoveFeet first;
  hexapod_msgs::MoveFeet second;
  first.legs[0] = false;
  first.legs[1] = true;
  first.legs[2] = false;
  first.legs[3] = true;
  first.legs[4] = false;
  first.legs[5] = true;

  second.legs[0] = true;
  second.legs[1] = false;
  second.legs[2] = true;
  second.legs[3] = false;
  second.legs[4] = true;
  second.legs[5] = false;

  std::vector<bool> one = {false, true, false, true, false, true};
  std::vector<bool> two = {true, false, true, false, true, false};

  // Поднимаем ножки
  interpolationOfAngles(current_state, upLegs(one));
  ros::Duration(3.0).sleep();

  // запрос на получение награды от ГИРОСКОПА № 1 !!!
  hexapod_msgs::RewardGyro srv_gyro_1;
  srv_gyro_1.request.legs = first;
  srv_gyro_1.request.current_time = 0;

  // отправляем запрос ГИРОСКОПУ НОМЕР 1
  if (!client_gyro_learning.call(srv_gyro_1)) {
    std::cout << "Failed to call service /calculator_reward_gyroscope"
              << std::endl;
    interpolationOfAngles(current_state, downLegs(one));
    res.result_str = "error";
    res.result = false;
    return true;
  }

  std::cout << "reward_gyroscope_1 = " << srv_gyro_1.response.reward_gyroscope
            << std::endl;

  // Если мы отклонились на первом этапе
  if (srv_gyro_1.response.reward_gyroscope <= 2.0) {
    interpolationOfAngles(current_state, downLegs(one));

    // отправляем ответ
    res.result_str = "left";
    res.result = true;
    return true;
  }

  // все норм на ПЕРВОМ этапе мы устояли
  interpolationOfAngles(current_state, downLegs(one));

  // теперь поднимаем ОБРАТНЫЕ лапки
  interpolationOfAngles(current_state, upLegs(two));
  ros::Duration(3.0).sleep();

  // отправляем запрос на получение награды ГИРОСКОПА - 2
  hexapod_msgs::RewardGyro srv_gyro_2;
  srv_gyro_2.request.legs = second;
  srv_gyro_2.request.current_time = 0;

  if (!client_gyro_learning.call(srv_gyro_2)) {
    std::cout << "Failed to call service /calculator_reward_gyroscope"
              << std::endl;
    interpolationOfAngles(current_state, downLegs(two));
    res.result_str = "error";
    res.result = false;

    return true;
  }

  // Если мы отклонились на ВТОРОМ этапе
  if (srv_gyro_2.response.reward_gyroscope <= 2.0) {
    interpolationOfAngles(current_state, downLegs(two));
    // отправляем ответ
    res.result_str = "right";
    res.result = true;
    return true;
  }
  interpolationOfAngles(current_state, downLegs(two));

  // success
  res.result_str = "success";
  res.result = false;

  return true;
}

void MoveFeet::zeroLegCallback(std_msgs::Float32 msg) {
  yes_zero_legs = true;
  zero_legs = msg.data;
}

void MoveFeet::zeroLegResetCallback(std_msgs::Bool msg) {
  yes_zero_legs = false;
}
