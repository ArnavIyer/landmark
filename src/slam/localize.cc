// kind of like mapping but you have a list of landmarks, and you don't create more.
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
#include "localize.h"
#include "visualization/visualization.h"
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using math_util::AngleDiff;
using math_util::AngleMod;

using namespace math_util;
using namespace gtsam;
using namespace ros_helpers;
using namespace std;

bool PRINT_DEBUG = false;

namespace {
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
// Epsilon value for handling limited numerical precision.
} //namespace

namespace localize {

Localize::Localize(const string& map_name, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    imu_initialized_(false),
    imu_initial_pitch_(0),
    robot_loc_(0, 0),
    robot_pitch_(0),
    robot_angle_(0),
    prev_robot_loc_(0, 0),
    prev_robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    pose_ctr_(1),
    next_optim_pose_(1),
    prior_noise_(gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.1))),
    odometry_noise_(gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1))),
    landmark_noise_(noiseModel::Diagonal::Sigmas(Vector2(0.05, 0.01))) {
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  // landmarks_ = {{1.73072, -2.22105},{3.13361, 1.11729},{-0.133115, 1.7206},{5.18386, -1.08665},{6.87062, 0.759768},{5.97777, 3.40261},{4.58412, 4.11657},{1.85644, 3.83728}};
  //landmarks_ = {{2.7107, -1.67634},{3.73002, -0.923185},{2.33804, 1.44365},{4.94061, 1.90544},{6.34828, 0.511924},{5.7771, -2.15982},{8.7442, -1.65182},{8.63734, 1.34932}};

// 1.bag 2.6.bag jerboa
    // landmarks_ = {{2.10333, -1.11834},{1.25204, 0.880852},{3.82995, 0.544558},{2.48576, 2.66615},{4.00329, -1.5338},{5.65511, -0.765878},{4.20644, 2.65874},{5.89414, 1.48037}};

// sheep
  landmarks_ = {{2.4805, -0.315669},{0.988034, 0.835815},{0.763803, 2.51688},{2.47384, 2.35874},{4.28563, -0.262881},{3.96567, 1.82931},{5.9226, 0.712173},{5.96346, 2.25122},{3.57068, 3.74683},{5.27624, 4.33304},{3.37519, 5.21245},{1.6105, 4.7866}};



  for (size_t i = 0; i < landmarks_.size(); i++) {
    landmark_ids_.push_back(Symbol('l', i));
  }
  for (size_t i = 0; i < landmarks_.size(); i++) {
    Point2 ld(landmarks_[i][0], landmarks_[i][1]);
    estimates_.insert(landmark_ids_[i], ld);
    graph_.add(NonlinearEquality<Point2>(landmark_ids_[i], ld));
  }

  gtsam::Pose2 prior_mean_(0.0, 0.0, 0.0);
  pose_ids_.push_back(Symbol('x', 0));
  estimates_.insert(pose_ids_[0], prior_mean_);
  graph_.add(PriorFactor<gtsam::Pose2>(pose_ids_[0], prior_mean_, prior_noise_));
}


void Localize::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Localize::UpdateOrientation(float roll, double pitch, double yaw) {
  if (!imu_initialized_) {
      imu_initial_pitch_ = pitch;
      imu_initial_roll_ = roll;
      robot_pitch_ = 0;
      robot_roll_ = 0;
      imu_initialized_ = true;
      return;
  }
  
  robot_pitch_ = pitch - imu_initial_pitch_;
  robot_roll_ = roll - imu_initial_roll_;
}

void Localize::UpdateOdometry(const Vector2f& loc,
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

gtsam::Pose2 Localize::GetLocation() {
  if (pose_ids_.size() == 0)
    return gtsam::Pose2(0,0,0);
  return gtsam::Pose2(robot_loc_.x(), robot_loc_.y(), robot_angle_);
}

void Localize::UpdatePointCloud(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max,
                    float angle_increment) {
  
  const Eigen::Vector3f laser_pos(0.305, 0, 0.38);

  const int num_rays = static_cast<int>(
      1.0 + (angle_max - angle_min) /
      angle_increment);

  point_cloud_.clear();

  // Convert the LaserScan to a point cloud
  float range;
  float angle;
  Eigen::Vector3f base_link_loc;
  for (int i = 0; i < num_rays; i++) {
    range = ranges[i];
    if (range >= range_min && range <= range_max) {
      angle = angle_min + i * angle_increment;
      base_link_loc = Eigen::AngleAxisf(robot_roll_, Eigen::Vector3f::UnitX()) * 
                      Eigen::AngleAxisf(robot_pitch_, Eigen::Vector3f::UnitY()) * 
                      Eigen::Vector3f(range * cos(angle), range * sin(angle), 0) + laser_pos;
      if (abs(base_link_loc.x()) < 4 && abs(base_link_loc.y()) < 3)
        point_cloud_.push_back(Vector2f(base_link_loc.x(), base_link_loc.y()));
    }
  }

  extractor_.ExtractLandmarks2(circles_, point_cloud_);
}

void Localize::MatchLandmarks() {
  // loop through list of circles, match them to one of 4 landmarks in landmarks_;
  // cout << "circles_.size(): " << circles_.size() << endl;
  for (size_t i = 0; i < circles_.size(); i++) {
    Eigen::Rotation2Df rot(robot_angle_);
    Vector2f circle = robot_loc_ + rot*circles_[i].center;

    float min_dist = 100000;
    int landmark_match = 0;
    for (size_t j = 0; j < landmarks_.size(); j++) {
      // match circle in point cloud to landmarks
      float diff = (circle - landmarks_[j]).norm();
      if (min_dist > diff) {
        min_dist = diff;
        landmark_match = j;
      }
    }
    if (min_dist < 1.0) {
      // the current circle is the same as landmark j;
      graph_.add(gtsam::BearingRangeFactor<gtsam::Pose2, Point2>(pose_ids_[pose_ctr_], landmark_ids_[landmark_match], Rot2::fromAngle(atan2(circles_[i].center.y(), circles_[i].center.x())), circles_[i].center.norm(), landmark_noise_));
    } else {
      // not a landmark we can match to
    }
  }
}

void Localize::AddNewPose() {

    // add robot_loc_ and robot_angle_ as poses to the pose graph
    Eigen::Rotation2Df r1(-prev_robot_angle_);
    Eigen::Vector2f delta_pose = r1*(robot_loc_ - prev_robot_loc_);
    float delta_angle = AngleDiff(robot_angle_, prev_robot_angle_);

    pose_ids_.push_back(Symbol('x', pose_ctr_));
    graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(pose_ids_[pose_ctr_ - 1], pose_ids_[pose_ctr_], gtsam::Pose2(delta_pose.x(), delta_pose.y(), delta_angle), odometry_noise_));
    estimates_.insert(pose_ids_[pose_ctr_], gtsam::Pose2(robot_loc_.x(), robot_loc_.y(), robot_angle_));
    // match seen landmarks to poses
    MatchLandmarks();

    prev_robot_loc_ = robot_loc_;
    prev_robot_angle_ = robot_angle_;
    pose_ctr_++;
}

void Localize::OptimizeEstimates() {
    gtsam::LevenbergMarquardtOptimizer optimzer(graph_, estimates_);
    estimates_ = optimzer.optimize();

    // set robot pose and angle to optimized values
    gtsam::Pose2 robot_pose = estimates_.at<gtsam::Pose2>(pose_ids_.back());
    robot_loc_ = Vector2f(robot_pose.x(), robot_pose.y());
    robot_angle_ = robot_pose.theta();
}

void Localize::UpdateVisualization() {

  // draw point cloud
  for (auto pt : point_cloud_) {
    visualization::DrawPoint(pt, 0x000000, local_viz_msg_);
  }

  visualization::DrawLine(Eigen::Rotation2Df(robot_pitch_) * Vector2f(-4,0), Eigen::Rotation2Df(robot_pitch_) * Vector2f(4,0), 0x000000, global_viz_msg_);

  // draw landmarks
  for (auto ld : landmarks_) {
    visualization::DrawCross(Vector2f(ld[0], ld[1]), 0.08, 0x0d8000, global_viz_msg_);
  }

  // draw robot poses
  for (size_t i = 0; i < pose_ids_.size(); i++) {
    Pose2 p = estimates_.at<Pose2>(pose_ids_[i]);
    Vector2f loc(p.x(), p.y());
    visualization::DrawArc(loc, 0.06, 0, 6.28, 0x000000, global_viz_msg_);
    visualization::DrawLine(loc, loc + Vector2f(0.1*cos(p.theta()), 0.1*sin(p.theta())), 0x000000, global_viz_msg_);
  }

  // draw circles seen
  for (auto circle : circles_) {
    visualization::DrawArc(circle.center, circle.radius, 0, 6.28, 0xd49100, local_viz_msg_);
  }
}

void Localize::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);
  // std::cout << "before imu init" << std::endl;

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;
  if (!imu_initialized_) return;

  // std::cout << "imu init" << std::endl;

  UpdateVisualization();

  // add a new pose to the factor graph every time the robot moves x distance
  // if ((prev_robot_loc_ - robot_loc_).norm() > 0.1) {
    AddNewPose();
  // }

  // every pose added to the map, optimize the estimates of robot poses
  if (pose_ctr_ == next_optim_pose_) {
    next_optim_pose_ += 1;
    OptimizeEstimates();
  }
  
  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();

  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
}

}  // namespace localize
