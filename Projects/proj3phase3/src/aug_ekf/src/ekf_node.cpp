#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>


#include <ekf_filter.h>

using namespace std;
using namespace Eigen;

// for debug
#include <backward.hpp>
#define BACKWARD_HAS_DW 1
namespace backward {
backward::SignalHandling sh;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "aug_ekf");
  ros::NodeHandle n("~");

  ekf_imu_vision::EKFImuVision aug_ekf;
  aug_ekf.init(n);

  ros::spin();
}
