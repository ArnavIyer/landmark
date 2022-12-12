#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>

#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/Localization2DMsg.h"
#include "gflags/gflags.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"

#include "std_msgs/String.h"

#include "mapping.h"

using amrl_msgs::Localization2DMsg;
using math_util::DegToRad;
using math_util::RadToDeg;
using mapping::Mapping;
using ros::Time;
using ros_helpers::Eigen3DToRosPoint;
using ros_helpers::Eigen2DToRosPoint;
using ros_helpers::RosPoint;
using ros_helpers::SetRosVector;
using std::string;
using std::vector;
using Eigen::Vector2f;

// Create command line arguments
DEFINE_string(laser_topic, "scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "odom", "Name of ROS topic for odometry data");
DEFINE_string(imu_topic, "vectornav/IMU", "Name of ROS IMU topic");
DEFINE_string(loc_topic, "localization", "Name of ROS topic for localization");
DEFINE_string(init_topic,
              "initialpose",
              "Name of ROS topic for initialization");
DEFINE_string(map, "GDC1", "Name of vector map file");

bool run_ = true;
sensor_msgs::LaserScan last_laser_msg_;
ros::Publisher localization_publisher_;
amrl_msgs::Localization2DMsg localization_msg_;
Mapping* mapping_ = nullptr;

Eigen::Vector3f ToEulerAngles(const Eigen::Quaternionf& q) {
    Eigen::Vector3f angles;    //yaw pitch roll
    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[2] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles[0] = std::atan2(siny_cosp, cosy_cosp);
    return angles;
}

void IMUCallback(const sensor_msgs::Imu& msg) {
  Eigen::Quaternionf q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  Eigen::Vector3f YPR = ToEulerAngles(q);
  mapping_->UpdatePitch(YPR[1], -2.0 * atan2(msg.orientation.z, msg.orientation.w));
}

// void LaserCallback(const sensor_msgs::LaserScan& msg) {
//   if (FLAGS_v > 0) {
//     printf("Laser t=%f, dt=%f\n",
//            msg.header.stamp.toSec(),
//            GetWallTime() - msg.header.stamp.toSec());
//   }
//   // Location of the laser on the robot. Assumes the laser is forward-facing.
//   const Vector2f kLaserLoc(0.305,0);

//   const int num_rays = static_cast<int>(
//       1.0 + (msg.angle_max - msg.angle_min) /
//       msg.angle_increment);

//   vector<Vector2f> point_cloud_(num_rays);
//   point_cloud_.clear();

//   // Convert the LaserScan to a point cloud
//   float range;
//   float angle;
//   Vector2f laser_loc;
//   for (int i = 0; i < num_rays; i++) {
//     range = msg.ranges[i];
//     if (range > 2.5) {
//       continue;
//     }
//     if (range >= msg.range_min && range <= msg.range_max) {
//       angle = msg.angle_min + i * msg.angle_increment;
//       laser_loc = Vector2f(range * cos(angle), range * sin(angle)) + kLaserLoc;
//       // if ((laser_loc - kLaserLoc).x() < 4 && (laser_loc - kLaserLoc).y() < 2.5)
//       point_cloud_.push_back(laser_loc);
//     }
//   }
//   mapping_->ObservePointCloud(point_cloud_, msg.header.stamp.toSec());
//   last_laser_msg_ = msg;
// }

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f, dt=%f\n",
           msg.header.stamp.toSec(),
           GetWallTime() - msg.header.stamp.toSec());
  }
  // Location of the laser on the robot. Assumes the laser is forward-facing.
  const Vector2f kLaserLoc(0.2, 0);

  mapping_->UpdatePointCloud(msg.ranges,
        msg.range_min,
        msg.range_max,
        msg.angle_min,
        msg.angle_max,
        msg.angle_increment);
}


void PublishLocation() {
  gtsam::Pose2 pose = mapping_->GetLocation();
  localization_msg_.header.stamp = ros::Time::now();
  localization_msg_.map = "EmptyMap";
  localization_msg_.pose.x = pose.x();
  localization_msg_.pose.y = pose.y();
  localization_msg_.pose.theta = pose.theta();
  localization_publisher_.publish(localization_msg_);
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  mapping_->UpdateOdometry(
      Vector2f(msg.pose.pose.position.x, msg.pose.pose.position.y),
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
      Vector2f(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
      msg.twist.twist.angular.z);
  PublishLocation();
}

void GoToCallback(const geometry_msgs::PoseStamped& msg) {
  const Vector2f loc(msg.pose.position.x, msg.pose.position.y);
  const float angle =
      2.0 * atan2(msg.pose.orientation.z, msg.pose.orientation.w);
  printf("Goal: (%f,%f) %f\u00b0\n", loc.x(), loc.y(), angle);
  mapping_->SetNavGoal(loc, angle);
}

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "mapping", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  mapping_ = new Mapping(FLAGS_map, &n);

  ros::Subscriber velocity_sub =
      n.subscribe(FLAGS_odom_topic, 1, &OdometryCallback);
  ros::Subscriber laser_sub =
      n.subscribe(FLAGS_laser_topic, 1, &LaserCallback);
  ros::Subscriber imu_sub =
      n.subscribe(FLAGS_imu_topic, 1, &IMUCallback);
  localization_publisher_ =
      n.advertise<amrl_msgs::Localization2DMsg>(FLAGS_loc_topic, 1);
  ros::Subscriber goto_sub =
      n.subscribe("/move_base_simple/goal", 1, &GoToCallback);

  RateLoop loop(20.0);
  while (run_ && ros::ok()) {
    ros::spinOnce();
    mapping_->Run();
    loop.Sleep();
  }

  mapping_->PrintLandmarks();

  delete mapping_;
  return 0;
}
