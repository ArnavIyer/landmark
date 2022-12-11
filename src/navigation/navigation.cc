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
#include "navigation.h"
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

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    prev_robot_loc_(0, 0),
    prev_robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    pose_ctr_(0),
    landmark_ctr_(0) {
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

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {

}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }

  // add to odom_loc_ and odom_angle_
  Vector2f delta_loc = loc - odom_loc_;
  float delta_angle = AngleDiff(angle, odom_angle_);

  Eigen::Rotation2Df r1(-odom_angle_);
  Eigen::Rotation2Df r2(robot_angle_);

  robot_loc_ += r2*r1*delta_loc;
  robot_angle_ = AngleMod(robot_angle_ + delta_angle);

  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ExtractLandmarks(const vector<Vector2f>& cloud) {
  double epsilon = 0.05;
  circles_.clear();
  for (size_t i = 1; i < point_cloud_.size(); i++) {

    vector<Vector2f> circle;
    Vector2f avg(0,0);
    while (i < point_cloud_.size() && (point_cloud_[i] - point_cloud_[i-1]).norm() < epsilon) {
      circle.push_back(point_cloud_[i]);
      avg += cloud[i];
      i++;
    }
    avg /= circle.size();

    if (circle.size() < 15) {
      continue;
    }

    for (size_t j = 0; j < circle.size(); j++) {
      circle[j] -= avg;
    }
    double xx = 0; // a1
    double yy = 0; // b2
    double xy = 0; // b1, a2
    double xxx = 0;
    double yyy = 0;
    double xyy = 0;
    double xxy = 0;
    for (size_t j = 0; j < circle.size(); j++) {
      xx  += circle[j].x()*circle[j].x();
      yy  += circle[j].y()*circle[j].y();
      xy  += circle[j].y()*circle[j].x();
      xxx += circle[j].x()*circle[j].x()*circle[j].x();
      yyy += circle[j].y()*circle[j].y()*circle[j].y();
      xyy += circle[j].x()*circle[j].y()*circle[j].y();
      xxy += circle[j].x()*circle[j].x()*circle[j].y();
    }
    double c1 = 0.5*(xxx+xyy);
    double c2 = 0.5*(yyy+xxy);
    double a1 = xx;
    double a2 = xy;
    double b1 = xy;
    double b2 = yy;

    // solve for circle equation here
    double determinant = a1*b2 - b1*a2;
    double xc = (c1*b2 - b1*c2)/determinant;
    double yc = (a1*c2 - c1*a2)/determinant;
    // Vector2f center(xc+avg.x(), yc+avg.y());
    double radius = sqrt(xc*xc + yc*yc + (xx+yy)/circle.size());
    // vector<double> circ = {xc+avg.x(), yc+avg.y(), radius};
    circles_.push_back(Circle(xc+avg.x(), yc+avg.y(), radius));
    all_circles_.push_back(Vector2f(xc+avg.x(), yc+avg.y()));
  }
  if (PRINT_DEBUG)
    cout << "circles_.size(): " << circles_.size() << endl;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;
  Navigation::ExtractLandmarks(cloud);
}

void Navigation::UpdateLandmarks() {
  // match circles_ to landmarks_
  size_t num_landmarks = landmarks_.size();
  if (PRINT_DEBUG)
  cout << "num landmarks: " << num_landmarks << endl;
  for (size_t i = 0; i < circles_.size(); i++) {
    // calculate absolute position of circle
    Eigen::Rotation2Df rot(robot_angle_);
    Vector2f circle = robot_loc_ + rot*circles_[i].center; // match to what is in landmarks
    // landmarks_.push_back(Circle(circle, circles_[i].radius));

    float min_dist = 100000;
    int landmark_match = 0;
    for (size_t j = 0; j < num_landmarks; j++) {
      // match circle in point cloud to landmarks
      float diff = (circle - landmarks_[j].center).norm();
      if (min_dist > diff) {
        min_dist = diff;
        landmark_match = j;
      }
    }
    string landmark;
    if (min_dist < 0.6) {
      if (PRINT_SLAM_INFO)
        landmark = "LANDMARK " + to_string(pose_ctr_) + " " + to_string(landmark_match) + " " + to_string(atan2(circles_[i].center.y(), circles_[i].center.x())) + " " + to_string(circles_[i].center.norm());
      landmarks_[landmark_match] = Circle(circle, circles_[i].radius);
    } else {
      if (PRINT_SLAM_INFO)
        landmark = "LANDMARK " + to_string(pose_ctr_) + " " + to_string(landmarks_.size()) + " " + to_string(atan2(circles_[i].center.y(), circles_[i].center.x())) + " " + to_string(circles_[i].center.norm());
      landmarks_.push_back(Circle(circle, circles_[i].radius));
    }
    if (PRINT_SLAM_INFO) {
      cout << landmark << endl;
    }
  }
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // for (auto circle : circles_) {
  //   visualization::DrawArc(circle.center, circle.radius, 0, 6.28, 0x555555, global_viz_msg_);
  // }

  for (auto circle : all_circles_) {
    visualization::DrawArc(circle, 0.11, 0, 6.28, 0x550055, global_viz_msg_);
  }

  visualization::DrawArc(robot_loc_, 0.1, 0, 6.28, 0x00FFFF, global_viz_msg_);
  for (auto pose : robot_poses_) {
    visualization::DrawArc(pose, 0.1, 0, 6.28, 0x555555, global_viz_msg_);
    visualization::DrawLine(robot_loc_, robot_loc_ + Vector2f(0.1*cos(robot_angle_), 0.1*sin(robot_angle_)), 0x555555, global_viz_msg_);
  }

  if ((prev_robot_loc_ - robot_loc_).norm() > 0.1) {
    robot_poses_.push_back(robot_loc_);

    // add robot_loc_ and robot_angle_ as poses to the pose graph
    Eigen::Rotation2Df r1(-prev_robot_angle_);
    Eigen::Vector2f delta_pose = r1*(robot_loc_ - prev_robot_loc_);
    float delta_angle = AngleDiff(robot_angle_, prev_robot_angle_);
    // VERTEX_SE2 poseidx x y t mapx mapy
    string pose = "VERTEX_SE2 " + to_string(pose_ctr_) + " " + to_string(delta_pose.x()) + " " + to_string(delta_pose.y()) + " " + to_string(delta_angle) + " " + to_string(robot_loc_.x()) + " " + to_string(robot_loc_.y()) + " " + to_string(robot_angle_);
    if (PRINT_SLAM_INFO)
      cout << pose << endl;

    UpdateLandmarks();
    
    // find correspondences between circles
    // std::vector<std::pair<Circle, int>> circle_matches;
    // int num_matches = 0;
  
    // for (size_t i = 0; i < circles_.size(); i++) {
    //   bool match = false;
    //   for (size_t j = 0; j < prev_circles_.size(); j++) {
    //     // bool match = (circles_[i].center - (prev_circles_[j].first.center - delta_pose)).norm() < 0.10;
    //     match = (circles_[i].center - prev_circles_[j].first.center).norm() < 0.5;
    //     if (match) {
    //       // LANDMARK poseidx idx 
    //       string landmark = "LANDMARK " + to_string(pose_ctr_) + " " + to_string(prev_circles_[j].second) + " " + to_string(atan2(circles_[i].center.y(), circles_[j].center.x())) + " " + to_string(circles_[i].center.norm());
    //       if (PRINT_SLAM_INFO)
    //         cout << landmark << endl;
    //       circle_matches.push_back({circles_[i], prev_circles_[j].second});
    //       num_matches++;
    //       break;
    //     }
    //   }
    //   if (match)
    //     continue;
    //   // new landmark discovered
    //   circle_matches.push_back({circles_[i], landmark_ctr_});
    //   string landmark = "LANDMARK " + to_string(pose_ctr_) + " " + to_string(landmark_ctr_) + " " + to_string(atan2(circles_[i].center.y(), circles_[i].center.x())) + " " + to_string(circles_[i].center.norm());
    //   if (PRINT_SLAM_INFO)
    //     cout << landmark << endl;
    //   landmark_ctr_++;
    // }


    prev_robot_loc_ = robot_loc_;
    prev_robot_angle_ = robot_angle_;
    pose_ctr_++;
  }

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
