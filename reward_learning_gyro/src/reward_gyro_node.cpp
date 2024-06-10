#include <reward_gyro.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "calculator_reward_gyroscope");
  ros::NodeHandle nh;
  RewardGyroscope reward_gyro(nh);
  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
