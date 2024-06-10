
#include <gazebo_msgs/ModelStates.h>
#include <hexapod_msgs/CheckGait.h>
#include <hexapod_msgs/RewardGyro.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

class RewardGyroscope {
 public:
  RewardGyroscope(ros::NodeHandle nh_);
  std::string calculatorRewardGyroscope(int count, int side);

 private:
  ros::NodeHandle nh;
  ros::Subscriber gyroscope_sub;
  ros::Publisher reward_gyroscope_pub;
  ros::ServiceServer service_;
  ros::ServiceServer service_adaptation;

  ros::Subscriber block;

  void gyroscopeCallback(sensor_msgs::Imu gyroscope_msg);
  bool init_service(hexapod_msgs::RewardGyro::Request &req,
                    hexapod_msgs::RewardGyro::Response &res);

  bool serviceAdaptation(hexapod_msgs::RewardGyro::Request &req,
                         hexapod_msgs::RewardGyro::Response &res);

  void blockCallback(std_msgs::Float32 msg);

  sensor_msgs::Imu gyroscope;
  sensor_msgs::Imu gyroscope_last;

  std::vector<double> THRESHOLD_GYROSCOPE;
  double THRESHOLD_COORDINATES;
  double TIME_ITERATION;
  double reward_gyroscope = 0.0;
  double reward_odometry = 0.0;
  double reward = 0.0;
  bool gyroscope_bool = false;
  bool odometry_bool = false;
  bool gyroscope_last_bool = false;
  bool odometry_last_bool = false;

  double current_time = -1.0;

  std::string result_str;
  int time_t = 100;

  int block_legs;
};

void RewardGyroscope::blockCallback(std_msgs::Float32 msg) {
  block_legs = msg.data;
}

RewardGyroscope::RewardGyroscope(ros::NodeHandle nh_) : nh(nh_) {
  gyroscope_sub = nh_.subscribe("/spider/gyroscope_data", 5,
                                &RewardGyroscope::gyroscopeCallback, this);

  block = nh_.subscribe("/block", 5, &RewardGyroscope::blockCallback, this);

  reward_gyroscope_pub =
      nh_.advertise<std_msgs::Float32>("/reward/reward_gyroscope", 1000);

  service_ = nh_.advertiseService("/calculator_reward_gyroscope",
                                  &RewardGyroscope::init_service, this);

  service_adaptation =
      nh_.advertiseService("/adaptation_reward_gyroscope",
                           &RewardGyroscope::serviceAdaptation, this);

  ros::param::get("THRESHOLD_GYROSCOPE", THRESHOLD_GYROSCOPE);
  ros::param::get("THRESHOLD_COORDINATES", THRESHOLD_COORDINATES);
  ros::param::get("TIME_ITERATION", TIME_ITERATION);

  gyroscope_last.orientation.x = 0.0;
  gyroscope_last.orientation.y = 0.0;
  gyroscope_last.orientation.z = 0.0;

  ROS_INFO("RewardGyroscope ready");
}
bool RewardGyroscope::serviceAdaptation(
    hexapod_msgs::RewardGyro::Request &req,
    hexapod_msgs::RewardGyro::Response &res) {
  // получаем
  hexapod_msgs::MoveFeet vector_legs = req.legs;
  current_time = req.current_time;
  std::cout << "______________________ " << std::endl;

  int count = 0;
  int side = 0;

  // хотим узнать сколько ног подвинуть
  for (int i = 0; i < 6; i++) {
    if (vector_legs.legs[i] == true) {
      count++;
      std::cout << i << " = true" << "  ";
    } else
      std::cout << i << " = false" << "  ";
  }

  std::cout << " " << std::endl;
  // vector_legs.legs[block_legs] = true;

  // подстрахуем датчик ( и мой диплом в целом)
  if (vector_legs.legs[0] == true && vector_legs.legs[1] == true &&
      vector_legs.legs[2] == true) {
    side = 1;
    std::cout << "PRAV" << std::endl;
  }

  if (vector_legs.legs[3] == true && vector_legs.legs[4] == true &&
      vector_legs.legs[5] == true) {
    side = 1;
    std::cout << "LEFT" << std::endl;
  }

  if (vector_legs.legs[2] == true && vector_legs.legs[5] == true) {
    side = 1;
    std::cout << "FRONT" << std::endl;
  }
  if (vector_legs.legs[0] == true && vector_legs.legs[3] == true) {
    side = 1;
    std::cout << "ZAD" << std::endl;
  }

  // это нужно ибо мы два раза обращаемся к гироскопу и нужно чтоб награда была
  // по первому количеству ног
  if (time_t == current_time) {
    count = 6 - count;
  }
  time_t = current_time;

  std::cout << "count = " << count << std::endl;
  std::cout << "side = " << side << std::endl;

  std::cout << "current_time = " << current_time << std::endl;
  // расчет гироскопа
  std::cout << "gyroscope.orientation.x = " << gyroscope.orientation.x
            << std::endl;
  std::cout << "gyroscope.orientation.y = " << gyroscope.orientation.y
            << std::endl;
  std::cout << "gyroscope.orientation.z = " << gyroscope.orientation.z
            << std::endl;
  ros::Duration(1.5).sleep();

  if (abs(gyroscope_last.orientation.x - gyroscope.orientation.x) >
          THRESHOLD_GYROSCOPE[0] ||
      abs(gyroscope_last.orientation.y - gyroscope.orientation.y) >
          THRESHOLD_GYROSCOPE[1] ||
      abs(gyroscope_last.orientation.z - gyroscope.orientation.z) >
          THRESHOLD_GYROSCOPE[2]) {
    std::cout << "---------------BALANCE LOST--------------- " << std::endl;

    result_str = "balance lost max";
    reward_gyroscope = -2.0;

    // balansce saved
  } else {
    if (count == 6 || side == 1) {
      result_str = "balance lost max";
      reward_gyroscope = -3.0;

    } else {
      reward_gyroscope = 3.0;
      result_str = "balance saved";

      if (count == 0) {
        reward_gyroscope = 0.0;
        result_str = "zero";
      }
    }
  }

  std_msgs::Float32 msg;
  msg.data = reward_gyroscope;
  reward_gyroscope_pub.publish(msg);
  std::cout << "reward_gyroscope = " << reward_gyroscope << std::endl;
  std::cout << "______________________ " << std::endl;
  res.reward_gyroscope = reward_gyroscope;
  res.result = result_str;
  return true;
}

//***************************************************************
bool RewardGyroscope::init_service(hexapod_msgs::RewardGyro::Request &req,
                                   hexapod_msgs::RewardGyro::Response &res) {
  // получаем
  hexapod_msgs::MoveFeet vector_legs = req.legs;
  current_time = req.current_time;

  int count = 0;
  int side = 0;
  std::cout << "______________________ " << std::endl;

  // хотим узнать сколько ног подвинуть
  for (int i = 0; i < 6; i++) {
    if (vector_legs.legs[i] == true) {
      count++;
      std::cout << i << " = true" << "  ";
    } else
      std::cout << i << " = false" << "  ";
  }

  std::cout << " " << std::endl;

  // подстрахуем датчик ( и мой диплом в целом)
  if (vector_legs.legs[0] == true && vector_legs.legs[1] == true &&
      vector_legs.legs[2] == true) {
    side = 1;
    std::cout << "PRAV" << std::endl;
  }

  if (vector_legs.legs[3] == true && vector_legs.legs[4] == true &&
      vector_legs.legs[5] == true) {
    side = 1;
    std::cout << "LEFT" << std::endl;
  }

  if (vector_legs.legs[2] == true && vector_legs.legs[5] == true) {
    side = 1;
    std::cout << "FRONT" << std::endl;
  }
  if (vector_legs.legs[0] == true && vector_legs.legs[3] == true) {
    side = 1;
    std::cout << "ZAD" << std::endl;
  }

  // это нужно ибо мы два раза обращаемся к гироскопу и нужно чтоб награда была
  // по первому количеству ног
  if (time_t == current_time) {
    count = 6 - count;
  }
  time_t = current_time;

  std::cout << "count = " << count << std::endl;
  std::cout << "side = " << side << std::endl;

  std::string result_reward;

  result_reward = calculatorRewardGyroscope(count, side);
  std::cout << "______________________ " << std::endl;

  res.reward_gyroscope = reward_gyroscope;
  res.result = result_str;

  return true;
}

void RewardGyroscope::gyroscopeCallback(sensor_msgs::Imu gyroscope_msg) {
  gyroscope = gyroscope_msg;
  gyroscope_bool = true;
}

std::string RewardGyroscope::calculatorRewardGyroscope(int count, int side) {
  std::cout << "current_time = " << current_time << std::endl;
  // расчет гироскопа
  std::cout << "gyroscope.orientation.x = " << gyroscope.orientation.x
            << std::endl;
  std::cout << "gyroscope.orientation.y = " << gyroscope.orientation.y
            << std::endl;
  std::cout << "gyroscope.orientation.z = " << gyroscope.orientation.z
            << std::endl;
  ros::Duration(1.5).sleep();

  if (abs(gyroscope_last.orientation.x - gyroscope.orientation.x) >
          THRESHOLD_GYROSCOPE[0] ||
      abs(gyroscope_last.orientation.y - gyroscope.orientation.y) >
          THRESHOLD_GYROSCOPE[1] ||
      abs(gyroscope_last.orientation.z - gyroscope.orientation.z) >
          THRESHOLD_GYROSCOPE[2]) {
    result_str = "balance lost";

    std::cout << "---------------BALANCE LOST--------------- " << std::endl;
    // отклонились
    if (count > 3) {
      reward_gyroscope = -2.0;
      result_str = "balance lost max";
    }

    if (count <= 3) reward_gyroscope = 2.0;
    // if(count == 3)
    //   reward_gyroscope =  0.0;

    // balansce saved
  } else {
    // шесть ног
    if (count > 5) {
      std::cout << "Aaa 6!" << std::endl;
      reward_gyroscope = -2.0;
      result_str = "balance lost max";
    }
    // ноль ног
    if (count == 0) {
      reward_gyroscope = 0.0;
      result_str = "zero";
    }
    // все норм

    if (count < 5 && count != 0 && side == 0) {
      reward_gyroscope = 5.0;
      result_str = "balance saved";
    } else {
      // не всё норм
      if (count <= 3) {
        result_str = "balance lost";
        reward_gyroscope = 2.0;
      }

      //  if (count == 3){
      //     result_str = "balance lost";
      //     reward_gyroscope =   0.0;
      // }
      if (count > 3) {
        result_str = "balance lost max";
        reward_gyroscope = -2.0;
      }
    }
  }

  std_msgs::Float32 msg;
  msg.data = reward_gyroscope;
  reward_gyroscope_pub.publish(msg);
  std::cout << "reward_gyroscope = " << reward_gyroscope << std::endl;
  return "success";
}
