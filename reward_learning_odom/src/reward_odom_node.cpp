#include <reward_odom.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "calculator_reward_odom");
  ros::NodeHandle nh;
  RewardOdometry reward_odom(nh);
  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
