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
#include "mapping.h"
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
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace mapping {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Mapping::Mapping(const string& map_name, ros::NodeHandle* n) :
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
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    pose_ctr_(1),
    landmark_ctr_(0),
    next_optim_pose_(10),
    prior_noise_(gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.1))),
    odometry_noise_(gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1))),
    landmark_noise_(noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.2))) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);

  // for (size_t i = 0; i < landmarks_.size(); i++) {
  //   landmark_ids_.push_back(Symbol('l', i));
  // }
  // for (size_t i = 0; i < landmarks_.size(); i++) {
  //   Point2 ld(landmarks_[i][0], landmarks_[i][1]);
  //   estimates_.insert(landmark_ids_[i], ld);
  //   graph_.add(NonlinearEquality<Point2>(landmark_ids_[i], ld));
  // }

  gtsam::Pose2 prior_mean_(0.0, 0.0, 0.0);
  pose_ids_.push_back(Symbol('x', 0));
  estimates_.insert(pose_ids_[0], prior_mean_);
  graph_.add(PriorFactor<gtsam::Pose2>(pose_ids_[0], prior_mean_, prior_noise_));
}

void Mapping::SetNavGoal(const Vector2f& loc, float angle) {

}

void Mapping::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Mapping::UpdatePitch(double pitch, double yaw) {
    robot_pitch_ = pitch;

    if (!imu_initialized_) {
        imu_initial_pitch_ = pitch;
        imu_initialized_ = true;
    }
}

void Mapping::UpdateOdometry(const Vector2f& loc,
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

gtsam::Pose2 Mapping::GetLocation() {
  if (pose_ids_.size() == 0)
    return gtsam::Pose2(0,0,0);
  return gtsam::Pose2(robot_loc_.x(), robot_loc_.y(), robot_angle_);
}

void Mapping::UpdatePointCloud(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max,
                    float angle_increment) {
  
  const Eigen::Vector3f laser_pos(0.2, 0, 0.38);

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
      base_link_loc = Eigen::AngleAxisf(robot_pitch_, Eigen::Vector3f::UnitY()) * Eigen::Vector3f(range * cos(angle), range * sin(angle), 0) + laser_pos;
      if (abs(base_link_loc.x()) < 4 && abs(base_link_loc.y()) < 2.5)
        point_cloud_.push_back(Vector2f(base_link_loc.x(), base_link_loc.y()));
    }
  }

  ExtractLandmarks();
}

void Mapping::ExtractLandmarks2(const vector<Vector2f>& cloud) {
  // find circles

  // find closest point on each circle

  // landmark is the distance to closest point plus diameter of tube
}

// TODO: change to find closest point on circle and add radius of cylinder to get center
void Mapping::ExtractLandmarks() {
  double epsilon = 0.05;
  circles_.clear();
  for (size_t i = 1; i < point_cloud_.size(); i++) {

    vector<Vector2f> circle;
    Vector2f avg(0,0);
    float max_diff = 0;
    while (i < point_cloud_.size() && (point_cloud_[i] - point_cloud_[i-1]).norm() < epsilon) {
      circle.push_back(point_cloud_[i]);
      avg += point_cloud_[i];
      max_diff = max(max_diff, (avg - point_cloud_[i]).norm());
      i++;
    }
    avg /= circle.size();
    // if (circle.size() < 10 || circle.size() > 50 || max_diff > 0.2) {
    //   continue;
    // }
    if (circle.size() < 12) {
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
    circles_.push_back(Circle(xc+avg.x(), yc+avg.y(), radius));
  }
  // // if (PRINT_DEBUG)
  //   cout << "circles_.size(): " << circles_.size() << endl;
}

void Mapping::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;
  Mapping::ExtractLandmarks();
}

void Mapping::MatchLandmarks() {
  // loop through list of circles, match them to one of 4 landmarks in landmarks_;
  // cout << "circles_.size(): " << circles_.size() << endl;
  for (size_t i = 0; i < circles_.size(); i++) {
    Eigen::Rotation2Df rot(robot_angle_);
    Vector2f circle = robot_loc_ + rot*circles_[i].center;

    float min_dist = 100000;
    int landmark_match = 0;
    for (size_t j = 0; j < landmark_ids_.size(); j++) {
      // match circle in point cloud to landmarks
      Point2 ld(estimates_.at<gtsam::Point2>(landmark_ids_[j]));
      float diff = (circle - Vector2f(ld.x(), ld.y())).norm();
      if (min_dist > diff) {
        min_dist = diff;
        landmark_match = j;
      }
    }
    if (min_dist < 0.4) {
      // the current circle is the same as landmark j;
      graph_.add(gtsam::BearingRangeFactor<gtsam::Pose2, Point2>(pose_ids_[pose_ctr_], landmark_ids_[landmark_match], Rot2::fromAngle(atan2(circles_[i].center.y(), circles_[i].center.x())), circles_[i].center.norm(), landmark_noise_));
    } else {
      // not a landmark we can match to
      landmark_ids_.push_back(gtsam::Symbol('l', landmark_ctr_));
      Vector2f ld_pos = circles_[i].center + robot_loc_;
      estimates_.insert(landmark_ids_[landmark_ctr_], Point2(ld_pos.x(), ld_pos.y()));
      graph_.add(gtsam::BearingRangeFactor<gtsam::Pose2, Point2>(pose_ids_[pose_ctr_], landmark_ids_[landmark_ctr_], Rot2::fromAngle(atan2(circles_[i].center.y(), circles_[i].center.x())), circles_[i].center.norm(), landmark_noise_));
      landmark_ctr_++;
    }
  }
}

void Mapping::AddNewPose() {

    // add robot_loc_ and robot_angle_ as poses to the pose graph
    Eigen::Rotation2Df r1(-prev_robot_angle_);
    Eigen::Vector2f delta_pose = r1*(robot_loc_ - prev_robot_loc_);
    float delta_angle = AngleDiff(robot_angle_, prev_robot_angle_);

    pose_ids_.push_back(Symbol('x', pose_ctr_));
    graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(pose_ids_[pose_ctr_ - 1], pose_ids_[pose_ctr_], gtsam::Pose2(delta_pose.x(), delta_pose.y(), delta_angle), odometry_noise_));
    estimates_.insert(pose_ids_[pose_ctr_], gtsam::Pose2(robot_loc_.x(), robot_loc_.y(), robot_angle_));
    // match seen landmarks to poses
    MatchLandmarks();
    // UpdateLandmarks();

    prev_robot_loc_ = robot_loc_;
    prev_robot_angle_ = robot_angle_;
    pose_ctr_++;
}

void Mapping::OptimizeEstimates() {
    gtsam::LevenbergMarquardtOptimizer optimzer(graph_, estimates_);
    estimates_ = optimzer.optimize();

    // set robot pose and angle to optimized values
    gtsam::Pose2 robot_pose = estimates_.at<gtsam::Pose2>(pose_ids_.back());
    robot_loc_ = Vector2f(robot_pose.x(), robot_pose.y());
    robot_angle_ = robot_pose.theta();
}

void Mapping::UpdateVisualization() {

  // draw point cloud
  for (auto pt : point_cloud_) {
    visualization::DrawPoint(pt, 0x000000, local_viz_msg_);
  }

  // draw landmarks
  for (size_t i = 0; i < landmark_ids_.size(); i++) {
    gtsam::Point2 pt = estimates_.at<gtsam::Point2>(landmark_ids_[i]);
    visualization::DrawCross(Vector2f(pt.x(), pt.y()), 0.08, 0x0d8000, global_viz_msg_);
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

void Mapping::PrintLandmarks() {
  cout << "{";
  for (size_t i = 0; i < landmark_ids_.size(); i++) {
    gtsam::Point2 pt(estimates_.at<gtsam::Point2>(landmark_ids_[i]));
    cout << "{" << pt.x() << ", " << pt.y() << "}";
    if (i != landmark_ids_.size() - 1)
      cout << "," << flush;
  }
  cout << "}" << endl;
}

void Mapping::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;
  // if (!imu_initialized_) return;

  UpdateVisualization();

  // add a new pose to the factor graph every time the robot moves x distance
  if ((prev_robot_loc_ - robot_loc_).norm() > 0.1) {
    AddNewPose();
  }

  // every five poses added to the map, optimize the estimates of robot poses
  if (pose_ctr_ == next_optim_pose_) {
    next_optim_pose_ += 8;
    // cout << "starting optimize: " << estimates_.size() << endl;
    OptimizeEstimates();
    // cout << "finished optimize" << endl;
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

}  // namespace mapping
