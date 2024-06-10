#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <iostream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaaptation_algoritm");
  ros::NodeHandle nh_;
  ros::Publisher coxa = nh_.advertise<std_msgs::Float64>(
      "/spider/j_c1_lf_position_controller/command", 1000);
  ros::Publisher femur = nh_.advertise<std_msgs::Float64>(
      "/spider/j_thing_lf_position_controller/command", 1000);
  ros::Publisher tibia = nh_.advertise<std_msgs::Float64>(
      "/spider/j_tibia_lf_position_controller/command", 1000);

  ros::Duration(7.0).sleep();
  std_msgs::Float64 msg;
  msg.data = -1.0;
  femur.publish(msg);
  ros::Duration(2.0).sleep();
  msg.data = -0.8;
  tibia.publish(msg);
  ros::Duration(2.0).sleep();
  int k = 1.0;
  for (int i; i < 20; i++) {
    msg.data = 0.5 * k;
    coxa.publish(msg);
    ros::Duration(5.0).sleep();
    k = k * (-1.0);
  }

  return 0;
}
