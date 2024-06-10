#include <move_feet.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "legs_control_gazebo");
    ros::NodeHandle nh;
     //ros::AsyncSpinner spinner( 2 ); // Using 2 threads
     //spinner.start();
    MoveFeet move_feet(nh);
    ros::spin();
    return 0;

}
