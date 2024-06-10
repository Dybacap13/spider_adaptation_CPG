#include <math.h>
#include <reinforcement_learning.h>

#include <iostream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "reinforcement_learning");
  ros::NodeHandle nh;
  ReinforcementLearning reinforcement_learning(nh);
  ros::Duration(1.0).sleep();
  reinforcement_learning.algoritm();
  ros::spin();
  return 0;
}
