//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "pitch_test.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using math_util::AngleDiff;
using math_util::AngleMod;

using namespace math_util;
using namespace ros_helpers;
using namespace std;

bool PRINT_SLAM_INFO = true;
bool PRINT_DEBUG = false;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace pitch_test {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

PitchTest::PitchTest(const string& map_name, ros::NodeHandle* n) :
    vectornav_initialized_(false),
    robot_angle_(0),
    robot_pitch_(0),
    initial_pitch_(0),
    odom_angle_(0) {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void PitchTest::UpdateIMU(float pitch, float yaw) {
  robot_pitch_ = pitch;

  if (!vectornav_initialized_) {
    initial_pitch_ = 
    odom_angle_ = yaw;
    vectornav_initialized_ = true;
    return;
  }

  float delta_angle = AngleDiff(yaw, odom_angle_);
  robot_angle_ = AngleMod(robot_angle_ + delta_angle);
  odom_angle_ = yaw;
  robot_pitch_ = pitch;
}

// void PitchTest::ExtractLandmarks(const vector<Vector2f>& cloud) {
//   double epsilon = 0.05;
//   circles_.clear();
//   for (size_t i = 1; i < point_cloud_.size(); i++) {

//     vector<Vector2f> circle;
//     Vector2f avg(0,0);
//     while (i < point_cloud_.size() && (point_cloud_[i] - point_cloud_[i-1]).norm() < epsilon) {
//       circle.push_back(point_cloud_[i]);
//       avg += cloud[i];
//       i++;
//     }
//     avg /= circle.size();

//     if (circle.size() < 15) {
//       continue;
//     }

//     for (size_t j = 0; j < circle.size(); j++) {
//       circle[j] -= avg;
//     }
//     double xx = 0; // a1
//     double yy = 0; // b2
//     double xy = 0; // b1, a2
//     double xxx = 0;
//     double yyy = 0;
//     double xyy = 0;
//     double xxy = 0;
//     for (size_t j = 0; j < circle.size(); j++) {
//       xx  += circle[j].x()*circle[j].x();
//       yy  += circle[j].y()*circle[j].y();
//       xy  += circle[j].y()*circle[j].x();
//       xxx += circle[j].x()*circle[j].x()*circle[j].x();
//       yyy += circle[j].y()*circle[j].y()*circle[j].y();
//       xyy += circle[j].x()*circle[j].y()*circle[j].y();
//       xxy += circle[j].x()*circle[j].x()*circle[j].y();
//     }
//     double c1 = 0.5*(xxx+xyy);
//     double c2 = 0.5*(yyy+xxy);
//     double a1 = xx;
//     double a2 = xy;
//     double b1 = xy;
//     double b2 = yy;

//     // solve for circle equation here
//     double determinant = a1*b2 - b1*a2;
//     double xc = (c1*b2 - b1*c2)/determinant;
//     double yc = (a1*c2 - c1*a2)/determinant;
//     // Vector2f center(xc+avg.x(), yc+avg.y());
//     double radius = sqrt(xc*xc + yc*yc + (xx+yy)/circle.size());
//     // vector<double> circ = {xc+avg.x(), yc+avg.y(), radius};
//     double dist = std::hypot(xc+avg.x(), yc+avg.y()) * std::cos(robot_pitch_); // hypotenuse, to get actual distance, you cos(pitch) * it
//     Vector2f center = Vector2f(xc+avg.x(), yc+avg.y()).normalized()*dist;
//     circles_.push_back(Circle(center.x(), center.y(), radius));
//   }
//   if (PRINT_DEBUG)
//     cout << "circles_.size(): " << circles_.size() << endl;
// }

void PitchTest::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;
    // project every point down to the ground based on the pitch and publish
}

void PitchTest::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!vectornav_initialized_) return;
  std::cout << robot_pitch_ - initial_pitch_ << std::endl;
  for (size_t i = 0; i < point_cloud_.size(); i++) {
    // std::cout << cos(robot_pitch_) << std::endl;
    // visualization::DrawPoint(point_cloud_[i], 0x550055, local_viz_msg_);
    visualization::DrawPoint(point_cloud_[i]*std::cos(robot_pitch_), 0x550055, local_viz_msg_);
  }
  visualization::DrawArc(Vector2f(0,0), 0.1, 0, 6.28, 0x550055, global_viz_msg_);
  visualization::DrawLine(Vector2f(0,0), Vector2f(0.1*cos(robot_angle_), 0.1*sin(robot_angle_)), 0x550055, global_viz_msg_);

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  // drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
