#include <math.h>

#include <iostream>

#include "adaptation.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaaptation_algoritm");
  ros::NodeHandle nh;
  AdaptationAlgorithm adaptation_algorithm(nh);
  ros::Duration(1.0).sleep();
  adaptation_algorithm.algoritm();
  ros::spin();

  return 0;
}
