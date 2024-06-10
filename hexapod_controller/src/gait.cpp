
// ROS Hexapod Controller Node
// Copyright (c) 2016, Kevin M. Ochs
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the Kevin Ochs nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL KEVIN OCHS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Author: Kevin M. Ochs

#include <gait.h>

static const double PI = atan(1.0) * 4.0;

//==============================================================================
//  Constructor: Initialize gait variables
//==============================================================================

Gait::Gait(void) {
  ros::param::get("CYCLE_LENGTH", CYCLE_LENGTH);
  ros::param::get("LEG_LIFT_HEIGHT", LEG_LIFT_HEIGHT);
  ros::param::get("NUMBER_OF_LEGS", NUMBER_OF_LEGS);
  ros::param::get("GAIT_STYLE", GAIT_STYLE);
  cycle_period_ = 25;
  is_travelling_ = false;
  in_cycle_ = false;
  extra_gait_cycle_ = 1;
  current_time_ = ros::Time::now();
  last_time_ = ros::Time::now();
  gait_factor = 1.0;
  cycle_leg_number_ = {1, 0, 1, 0, 1, 0};
  if (GAIT_STYLE == "RIPPLE") {
    gait_factor = 0.5;
    cycle_leg_number_ = {1, 0, 2, 0, 2, 1};
  }
  period_distance = 0;
  period_height = 0;
  k = 0;
  smooth_base_.x = 0;
  smooth_base_.y = 0;
  smooth_base_.theta = 0;
}

//=============================================================================
// step calculation
//=============================================================================

void Gait::cyclePeriod(const geometry_msgs::Pose2D &base,
                       hexapod_msgs::FeetPositions *feet,
                       geometry_msgs::Twist *gait_vel) {
  // std::cout << "cycle_period_ = " << cycle_period_<< std::endl;
  period_height = sin(49 * PI / CYCLE_LENGTH);

  // Calculate current velocities for this period of the gait
  // This factors in the sinusoid of the step for accurate odometry
  current_time_ = ros::Time::now();
  double dt = (current_time_ - last_time_).toSec();
  gait_vel->linear.x =
      ((PI * base.x) / CYCLE_LENGTH) * period_height * (1.0 / dt);
  gait_vel->linear.y =
      ((-PI * base.y) / CYCLE_LENGTH) * period_height * (1.0 / dt);
  gait_vel->angular.z =
      ((PI * base.theta) / CYCLE_LENGTH) * period_height * (1.0 / dt);
  last_time_ = current_time_;

  // feet->foot[0].position.x = 0.086;

  // feet->foot[0].position.y = 0.0;

  //   feet->foot[2].position.y = 0;
  //   feet->foot[2].position.x = 0;

  // feet->foot[1].position.x = -0.086;
  // feet->foot[1].position.y = -0;

  // feet->foot[0].position.x = -0.104214;
  //  feet->foot[0].position.y = 0;

  // // feet->foot[0].position.y = 0.065297;
  // // feet->foot[0].position.x = 0;

  //  feet->foot[2].position.y = -0.086;
  //  feet->foot[2].position.x = -0.086;

  // // feet->foot[3].position.y = 0.065297;
  // // feet->foot[3].position.x = 0.0;

  // feet->foot[5].position.y = 0;
  // feet->foot[5].position.x = 0.104214;

  // feet->foot[4].position.x = 0.065297;
  // feet->foot[4].position.y = 0;
  // // double middle = 0.086;
  // // double krau = 0.0;
  // // double krauNO0 = 0.086;
  //  double nnn = 0.00;

  // // if ( k == 0){
  //      std::cout <<"AAA" <<std::endl;

  // //     //RR
  // //     //хуй

  // //     //RM
  // //     feet->foot[1].position.x = -middle;
  // //     feet->foot[1].position.z = LEG_LIFT_HEIGHT * period_height;

  //     //RF
  //     //feet->foot[2].position.x = krauNO0;
  //     feet->foot[2].position.x = nnn;

  // //     //LR
  //     //feet->foot[3].position.x = krau;
  //     feet->foot[3].position.x = nnn;
  //     feet->foot[3].position.y = nnn;
  //     //feet->foot[3].position.z = LEG_LIFT_HEIGHT * period_height;

  //     //LM
  //     feet->foot[4].position.x = middle;

  //     //LF
  //     feet->foot[5].position.z = LEG_LIFT_HEIGHT * period_height;
  //      k = 675;

  // }

  // for( int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++ )
  // {

  //     if( cycle_leg_number_[leg_index] == 0 && is_travelling_ == true )
  //     {
  //         feet->foot[leg_index].position.z = 0;

  //         // RR LR
  //         if (leg_index == 0 || leg_index == 3 )
  //         feet->foot[leg_index].position.x = krauNO0 ;

  //         // RM LM
  //         if ( leg_index == 1 || leg_index == 4)
  //         feet->foot[leg_index].position.x = middle ;

  //         // RF LF
  //         if ( leg_index == 2 || leg_index == 5)
  //         //feet->foot[leg_index].position.x = krau;
  //         feet->foot[leg_index].position.x = -nnn;

  //     }

  //     if( cycle_leg_number_[leg_index] == 1 )
  //     {

  //         feet->foot[leg_index].position.z = LEG_LIFT_HEIGHT * period_height;

  //         // RR LR
  //         if (leg_index == 0 || leg_index == 3 )
  //         //feet->foot[leg_index].position.x = krau ;
  //         feet->foot[leg_index].position.x = nnn;

  //         // RM LM
  //         if ( leg_index == 1 || leg_index == 4)
  //         feet->foot[leg_index].position.x = -middle ;

  //         // RF LF
  //         if ( leg_index == 2 || leg_index == 5)
  //         feet->foot[leg_index].position.x = -krauNO0;

  //     }
  // }

  for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++) {
    // Lifts the leg and move it forward
    if (cycle_leg_number_[leg_index] == 0 && is_travelling_ == true) {
      period_distance = cos(cycle_period_ * PI / CYCLE_LENGTH);
      feet->foot[leg_index].position.x = base.x * period_distance;
      feet->foot[leg_index].position.y = base.y * period_distance;
      feet->foot[leg_index].position.z = LEG_LIFT_HEIGHT * period_height;
      feet->foot[leg_index].orientation.yaw = base.theta * period_distance;

      // std::cout << "cycle_period_ = " << cycle_period_  << std::endl;
      // std::cout << "X = " << feet->foot[leg_index].position.x << std::endl;
    }
    // Moves legs backward pushing the body forward
    if (cycle_leg_number_[leg_index] == 1) {
      period_distance = cos(cycle_period_ * PI * gait_factor / CYCLE_LENGTH);
      feet->foot[leg_index].position.x = -base.x * period_distance;
      feet->foot[leg_index].position.y = -base.y * period_distance;
      feet->foot[leg_index].position.z = 0;
      feet->foot[leg_index].orientation.yaw = -base.theta * period_distance;

      // std::cout << "cycle_period_ = " << cycle_period_  << std::endl;
      // std::cout << "X = " << feet->foot[leg_index].position.x << std::endl;
    }
    if (cycle_leg_number_[leg_index] == 2) {
      period_distance =
          cos((CYCLE_LENGTH + cycle_period_) * PI * gait_factor / CYCLE_LENGTH);
      feet->foot[leg_index].position.x = -base.x * period_distance;
      feet->foot[leg_index].position.y = -base.y * period_distance;
      feet->foot[leg_index].position.z = 0;
      feet->foot[leg_index].orientation.yaw = -base.theta * period_distance;
    }
    // if (cycle_period_ == 25) cycle_period_=49;
  }
}

//=============================================================================
// Gait Sequencing
//=============================================================================

void Gait::gaitCycle(const geometry_msgs::Twist &cmd_vel,
                     hexapod_msgs::FeetPositions *feet,
                     geometry_msgs::Twist *gait_vel) {
  // Convert velocities into actual distance for gait/foot positions
  geometry_msgs::Pose2D base;
  base.x = cmd_vel.linear.x / PI * CYCLE_LENGTH;
  base.y = cmd_vel.linear.y / PI * CYCLE_LENGTH;
  base.theta = cmd_vel.angular.z / PI * CYCLE_LENGTH;
  // std::cout << "base.x = " << base.x << std::endl;

  // Low pass filter on the values to avoid jerky movements due to rapid value
  // changes
  smooth_base_.x = base.x * 0.05 + (smooth_base_.x * (1.0 - 0.05));
  smooth_base_.y = base.y * 0.05 + (smooth_base_.y * (1.0 - 0.05));
  smooth_base_.theta = base.theta * 0.05 + (smooth_base_.theta * (1.0 - 0.05));
  // std::cout << "smooth_base_.x = " << smooth_base_.x << std::endl;

  // Check to see if we are actually travelling
  if ((std::abs(smooth_base_.y) > 0.001) ||            // 1 mm
      (std::abs(smooth_base_.x) > 0.001) ||            // 1 mm
      (std::abs(smooth_base_.theta) > 0.00436332313))  // 0.25 degree
  {
    is_travelling_ = true;
  } else {
    is_travelling_ = false;
    // Check to see if the legs are in a non rest state
    for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++) {
      if ((std::abs(feet->foot[leg_index].position.x) > 0.001) ||  // 1 mm
          (std::abs(feet->foot[leg_index].position.y) > 0.001) ||  // 1 mm
          (std::abs(feet->foot[leg_index].orientation.yaw) >
           0.034906585) ||                                     // 2 degrees
          std::abs(feet->foot[leg_index].position.z) > 0.001)  // 1 mm
      {
        // If so calculate the rest of the cycle and add another complete cycle
        // This forces another cycle to allow all legs to set down after travel
        // is stopped
        extra_gait_cycle_ = CYCLE_LENGTH - cycle_period_ + CYCLE_LENGTH;
        break;
      } else {
        extra_gait_cycle_ = 1;
      }
    }

    // countdown for in_cycle state
    if (extra_gait_cycle_ > 1) {
      extra_gait_cycle_--;
      in_cycle_ = !(extra_gait_cycle_ == 1);
    }
  }

  // If either is true we consider the gait active
  if (is_travelling_ == true || in_cycle_ == true) {
    cyclePeriod(smooth_base_, feet, gait_vel);
    cycle_period_++;
  } else {
    // Reset period to start just to be sure. ( It should be here anyway )
    cycle_period_ = 0;
  }

  // Loop cycle and switch the leg groupings for cycle
  if (cycle_period_ == CYCLE_LENGTH) {
    cycle_period_ = 0;
    // std::cout << "end cycle_period_" << std::endl;
    // ros::Duration( 3 ).sleep();
    sequence_change(cycle_leg_number_);  // sequence change
  }
}

//=============================================================================
// Gait Sequence Change
//=============================================================================

void Gait::sequence_change(std::vector<int> &vec) {
  for (int i = 0; i < vec.size(); i++) {
    if (vec[i] == 0)
      vec[i] = 1;
    else if (vec[i] == 1 && GAIT_STYLE == "RIPPLE")
      vec[i] = 2;
    else
      vec[i] = 0;
  }
}
