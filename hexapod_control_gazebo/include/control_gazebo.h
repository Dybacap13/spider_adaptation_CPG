#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <string>
#include <hexapod_msgs/MoveFeet.h>

#include <ctime>
#include <iostream>
#include <map>
#include <typeinfo>
#include <vector>

float DELTA = 0.01;



std::vector<std::string> joints_names = {
    "j_c1_rr",    //0
    "j_thigh_rr" ,//1
    "j_tibia_rr" ,//2

    "j_c1_rm" ,   //3
    "j_thigh_rm" ,//4
    "j_tibia_rm" ,//5

    "j_c1_rf" ,   //6
    "j_thigh_rf" ,//7
    "j_tibia_rf" ,//8

    "j_c1_lr" ,   //9
    "j_thigh_lr" ,//10
    "j_tibia_lr" ,//11

    "j_c1_lm",    //12
    "j_thigh_lm" ,//13
    "j_tibia_lm" ,//14

    "j_c1_lf" ,   //15
    "j_thigh_lf" ,//16
    "j_tibia_lf" ,//17

};
std::vector<int> coxa = {0, 3, 6, 9, 12, 15};
std::vector<int> femur = {1, 4, 7, 10, 13, 16};
std::vector<int> tibia = {2, 5, 8, 11, 14, 17};



struct JointState {
  std::vector<std::string> joints;
  std::vector<double> angles;
  bool yes = false;
};

class ControlGazebo {
public:
  ControlGazebo( void );
  ~ControlGazebo() = default;

  JointState joints_state_node;
  void phublisherJointsStates();

private:
  ros::Subscriber sub_joints_state;
  ros::Publisher pub_in_controller;
  ros::NodeHandle nh_;

  sensor_msgs::JointState current_state;



  std::string name__space = "/spider/";
  std::map<std::string, ros::Publisher>
      publisher_joints_to_controller; // словарь ключ - контроллер, значение -
                                      // паблишер



  void jointsToGazeboCallback(sensor_msgs::JointState); // считывает состояние суставов


};


ControlGazebo::ControlGazebo(void)  {
  sub_joints_state = nh_.subscribe("/joints_to_gazebo", 10, &ControlGazebo::jointsToGazeboCallback, this);


   for (int i = 0; i < 18; i++) {
     pub_in_controller = nh_.advertise<std_msgs::Float64>(name__space + joints_names[i] + "_position_controller/command", 1000);
     publisher_joints_to_controller[joints_names[i]] = pub_in_controller;
     std::cout <<  name__space + joints_names[i] + "_position_controller/command" << std::endl;
   }
}



void ControlGazebo::jointsToGazeboCallback(sensor_msgs::JointState msg_joint_state){
    std_msgs::Float64 msg;

    for (auto i:femur){
        msg.data = msg_joint_state.position[i]  ;
        std::cout <<  msg_joint_state.name[i] <<" = "<< msg_joint_state.position[i] << std::endl;
        publisher_joints_to_controller[joints_names[i]].publish(msg);
    }

    for (auto i:tibia){
        msg.data = msg_joint_state.position[i]  ;
        std::cout <<  msg_joint_state.name[i] <<" = "<< msg_joint_state.position[i] << std::endl;
        publisher_joints_to_controller[joints_names[i]].publish(msg);
    }

    for (auto i:coxa){
        msg.data = msg_joint_state.position[i]  ;
        std::cout <<  msg_joint_state.name[i] <<" = "<< msg_joint_state.position[i] << std::endl;
        publisher_joints_to_controller[joints_names[i]].publish(msg);
    }

    // for(int i = 0 ; i < 18; i++){
    // msg.data = msg_joint_state.position[i] * (-1) ;
    // std::cout <<  msg_joint_state.name[i] <<" = "<< msg_joint_state.position[i] << std::endl;
    // publisher_joints_to_controller[joints_names[i]].publish(msg);

    //}

    std::cout <<  "__________________________" << std::endl;
    current_state =  msg_joint_state;

}




