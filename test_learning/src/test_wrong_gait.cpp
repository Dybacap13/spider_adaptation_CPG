#include <hexapod_msgs/MoveFeet.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_wrong_gait");
  ros::NodeHandle nh;
  ros::Publisher legs_pub =
      nh.advertise<hexapod_msgs::MoveFeet>("/move_legs/legs", 1000);

  hexapod_msgs::MoveFeet left;
  hexapod_msgs::MoveFeet right;
  hexapod_msgs::MoveFeet wrong;
  hexapod_msgs::MoveFeet wrong1;
  left.legs = {false, true, false, true, false, true};
  left.cmd_vel = 0.5;

  right.legs = {true, false, true, false, true, false};
  right.cmd_vel = 0.5;

  wrong.legs = {true, true, true, false, false, false};
  wrong.cmd_vel = 0.5;
  wrong1.legs = {false, false, false, false, true, true};
  wrong1.cmd_vel = 0.5;

  while (ros::ok()) {
    legs_pub.publish(left);
    ros::Duration(1.0).sleep();
    legs_pub.publish(right);
    ros::Duration(1.0).sleep();
    legs_pub.publish(wrong);
    ros::Duration(1.0).sleep();
    legs_pub.publish(wrong1);
    ros::Duration(1.0).sleep();

    ros::spinOnce();
  }

  return 0;
}
