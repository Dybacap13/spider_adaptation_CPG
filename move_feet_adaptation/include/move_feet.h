#include <hexapod_msgs/CheckGait.h>
#include <hexapod_msgs/MoveFeet.h>
#include <hexapod_msgs/MoveFeetLearning.h>
#include <hexapod_msgs/RewardGyro.h>
#include <hexapod_msgs/RewardOdom.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

class MoveFeet {
 public:
  MoveFeet(ros::NodeHandle nh_);

 private:
  ros::NodeHandle nh;
  ros::ServiceServer service_;
  ros::Subscriber joint_states_sub;
  ros::Subscriber move_feet_mode_sub;
  ros::Subscriber block_feet_sub;
  ros::Subscriber zero_leg_sub;
  ros::Subscriber zero_leg_reset_sub;
  ros::Publisher joint_states_pub;
  ros::ServiceClient client_odom_learning;
  ros::ServiceClient client_gyro_learning;
  ros::ServiceClient client_odom_adaptation;
  ros::ServiceClient client_gyro_adaptation;
  ros::ServiceServer service_check;

  sensor_msgs::JointState current_state;
  sensor_msgs::JointState target_state;
  sensor_msgs::JointState last_state;

  bool move_feet_mode = false;
  bool current_state_bool = false;
  bool last_state_bool = false;

  double FEMUR_ANGLE;
  std::vector<int> FEMUR_AXIS;
  std::vector<int> COXA_AXIS;
  double INTERPOLATION_COEFFICIENT;
  double DELTA;
  int NUMBER_OF_LEGS;
  double cmd_vel = 0;
  std::vector<bool> last_command = {false, false, false, false, false, false};
  std::vector<bool> block = {false, false, false, false, false, false};
  int zero_legs;
  bool yes_zero_legs = false;

  void jointStatesCallback(sensor_msgs::JointState);
  void moveFeetModeCallback(std_msgs::Bool);
  void blockCallback(hexapod_msgs::MoveFeet);
  void zeroLegCallback(std_msgs::Float32 msg);
  void zeroLegResetCallback(std_msgs::Bool msg);
  bool comparisonJointStates(sensor_msgs::JointState first,
                             sensor_msgs::JointState second);
  void interpolationOfAngles(sensor_msgs::JointState, sensor_msgs::JointState);
  void jointStatesPublisher(sensor_msgs::JointState);
  bool init_service(hexapod_msgs::MoveFeetLearning::Request &req,
                    hexapod_msgs::MoveFeetLearning::Response &res);

  bool check_service(hexapod_msgs::CheckGait::Request &req,
                     hexapod_msgs::CheckGait::Response &res);
  // void downLegs(bool reverse);

  sensor_msgs::JointState moveLegs(std::vector<bool>);
  sensor_msgs::JointState upLegs(std::vector<bool>);
  sensor_msgs::JointState downLegs(std::vector<bool>);
  sensor_msgs::JointState reverseTrueLegs(std::vector<bool>);
  // sensor_msgs::JointState reverseTrueLegsAndUpFalseLegs(std::vector<bool>);
  double reward = 0.0;
  double reward_gyroscope = 0.0;
  double reward_odometry = 0.0;
};
