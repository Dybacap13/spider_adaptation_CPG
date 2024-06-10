#include <control_gazebo.h>

int main( int argc, char **argv )
 {
    ros::init(argc, argv, "hexapod_control_gazebo");
    ControlGazebo control_gazebo;
    ros::spin();
     return 0;


}
