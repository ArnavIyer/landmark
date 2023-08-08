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
#include "localize_fkd.h"
#include "visualization/visualization.h"
#include "model_wrapper.h"
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <assert.h>

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

DEFINE_bool(fkd, false, "Use FKD for between factors");

namespace {
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
// Epsilon value for handling limited numerical precision.
} //namespace

namespace localize_fkd {

LocalizeFKD::LocalizeFKD(const string& map_name, ros::NodeHandle* n) :
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
    next_fkd_pose_(31),
    next_optim_pose_(31),
    prior_noise_(gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.1))),
    odometry_noise_(gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1))),
    landmark_noise_(noiseModel::Diagonal::Sigmas(Vector2(0.05, 0.01))),
    imu_buf_(250),
    joy_buf_(20),
    fkd_model_("/home/arnav/landmark/src/localize_fkd/chimp_model_pickle_vesc_trace.pt") {
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");

  // landmarks_ = {{1.30969, -0.719146},{0.426063, 1.27177},{1.00355, -2.59787},{3.00666, 0.996227},{3.22956, -1.07872},{4.86396, -0.249704},{5.02364, 2.01169},{3.35893, 3.11546},{1.59498, 3.06992}};
  // landmarks_ = {{2.10333, -1.11834},{1.25204, 0.880852},{3.82995, 0.544558},{2.48576, 2.66615},{4.00329, -1.5338},{5.65511, -0.765878},{4.20644, 2.65874},{5.89414, 1.48037}}; // jerboa
  landmarks_ = {{2.4805, -0.315669},{0.988034, 0.835815},{0.763803, 2.51688},{2.47384, 2.35874},{4.28563, -0.262881},{3.96567, 1.82931},{5.9226, 0.712173},{5.96346, 2.25122},{3.57068, 3.74683},{5.27624, 4.33304},{3.37519, 5.21245},{1.6105, 4.7866}}; // sheep

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


void LocalizeFKD::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void LocalizeFKD::UpdateOrientation(float roll, double pitch, double yaw, Eigen::Quaternionf q) {
  if (!imu_initialized_) {
      imu_initial_pitch_ = pitch;
      imu_initial_roll_ = roll;
      robot_pitch_ = 0;
      robot_roll_ = 0;
      imu_initialized_ = true;
      return;
  }

  // fkd takes vx, vy, w, x, y, sintheta, costheta
  // imu_buf_.add({q.x(), q.y(), q.z(), q.w()});

  robot_pitch_ = pitch - imu_initial_pitch_;
  robot_roll_ = roll - imu_initial_roll_;
}

void LocalizeFKD::UpdateCommand(float time, float v, float w) {
    joy_buf_.add({v, w, time});
}

void LocalizeFKD::UpdateRSOdom(vector<float>& data) {
  imu_buf_.add(data);
}

void LocalizeFKD::UpdateOdometry(const Vector2f& loc,
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

gtsam::Pose2 LocalizeFKD::GetLocation() {
  if (pose_ids_.size() == 0)
    return gtsam::Pose2(0,0,0);
  return gtsam::Pose2(robot_loc_.x(), robot_loc_.y(), robot_angle_);
}

void LocalizeFKD::UpdatePointCloud(const std::vector<float>& ranges,
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

void LocalizeFKD::MatchLandmarks() {
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

void LocalizeFKD::AddNewPoseNoEst() {
    pose_ids_.push_back(Symbol('x', pose_ctr_));

    estimates_.insert(pose_ids_[pose_ctr_], gtsam::Pose2(robot_loc_.x(), robot_loc_.y(), robot_angle_));

    // These two lines are now done by the Pytorch model instead of odometry in AddBetweenFactors
    Eigen::Rotation2Df r1(-prev_robot_angle_);
    Eigen::Vector2f delta_pose = r1*(robot_loc_ - prev_robot_loc_);
    float delta_angle = AngleDiff(robot_angle_, prev_robot_angle_);
    graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(pose_ids_[pose_ctr_ - 1], pose_ids_[pose_ctr_], gtsam::Pose2(delta_pose.x(), delta_pose.y(), delta_angle), odometry_noise_));
    // match seen landmarks to poses
    MatchLandmarks();


    prev_robot_loc_ = robot_loc_;
    prev_robot_angle_ = robot_angle_;
    pose_ctr_++;
}

void LocalizeFKD::AddNewPose() {

    // add robot_loc_ and robot_angle_ as poses to the pose graph

    pose_ids_.push_back(Symbol('x', pose_ctr_));
    Eigen::Rotation2Df r1(-prev_robot_angle_);
    Eigen::Vector2f delta_pose = r1*(robot_loc_ - prev_robot_loc_);
    float delta_angle = AngleDiff(robot_angle_, prev_robot_angle_);
    estimates_.insert(pose_ids_[pose_ctr_], gtsam::Pose2(robot_loc_.x(), robot_loc_.y(), robot_angle_));

    // These two lines are now done by the Pytorch model instead of odometry in AddBetweenFactors
    if (pose_ids_.size() < 32) {
      graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(pose_ids_[pose_ctr_ - 1], pose_ids_[pose_ctr_], gtsam::Pose2(delta_pose.x(), delta_pose.y(), delta_angle), odometry_noise_));
    }
    // match seen landmarks to poses
    MatchLandmarks();

    prev_robot_loc_ = robot_loc_; 
    prev_robot_angle_ = robot_angle_;
    pose_ctr_++;
}

float clip(float cost) {
  return (cost > 1) ? 1 : ((cost < -1) ? -1 : cost);
}

// TODO: test this 
float LocalizeFKD::cosAndSinToTheta(float cost, float sint) {
  cost = clip(cost);
  sint = clip(sint);

  float thetac = acos(cost);
  float thetas = asin(sint);

  // first quadrant
  if (thetac > 0 && thetas > 0) {
    return thetac;
  }

  // second quadrant
  if (thetac < 0 && thetas > 0) {
    return thetac;
  }

  // third wuadrant
  if (thetac < 0 && thetas < 0) {
    return (thetac + thetas) / 2;
  }

  // fourth quadrant
  if (thetac > 0 && thetas < 0) {
    return thetas;
  }

  return 0;
}

// fills out the input array with the following info:
// future 10 commands (-10 to -1)
// past 10 vx, vy, w from IMU
// past 10 x, y, theta from estimates_
void LocalizeFKD::GetFKDInput(float* input) {
  float time = joy_buf_.last_n(input, 10); // gets the last 10 steps of commands from vesc_drive

  // time is the time we want to start searching for imu data
  time -= 0.05;
  time += 0.14; // delay estimate

  // fill in velocities and ang vel
  float imudata[60];
  imu_buf_.sample_last_n(imudata, 10);

  for (int i = 0; i < 10; i++) {
    input[20+i] = imudata[i];
    input[30+i] = imudata[10+i];
    input[40+i] = imudata[20+i];
  }

  // get prev pose data from slam estimates
  Pose2 ref = estimates_.at<Pose2>(pose_ids_[pose_ids_.size() - 11]);
  input[59] = 0;            //x
  input[69] = 0;            //y
  input[79] = cos(0); // sin heading
  input[89] = sin(0); // cos heading

  for (int i = 0; i < 9; i++) {
    int pose_idx_ = pose_ids_.size() - 20 + i;

    Pose2 pose = estimates_.at<Pose2>(pose_ids_[pose_idx_]);

    float x = pose.x() - ref.x();
    float y = pose.y() - ref.y();
    float ref_ang = ref.theta();
    x = x*cos(-ref_ang)-y*sin(-ref_ang);
    y = x*sin(-ref_ang)+y*cos(-ref_ang);
    float theta = AngleMod(pose.theta() - ref_ang);

    input[50+i] = x;
    input[60+i] = y;
    input[70+i] = cos(theta);
    input[80+i] = sin(theta);
  }

}

// this function uses FKD to get the last 0.5 seconds worth of poses (10). 
void LocalizeFKD::AddBetweenFactors() {
    // std::cout << "before abf" << std::endl;

    assert(pose_ids_.size() >= 22);
    // assemble pytorch tensors

    float input[90]; // each 10: 0: cmdx, 10: cmdy, 20: vx, 30: vy,
                     // 40: w, 50: x, 60: y, 70: costheta, 80: sintheta
    float output[70];
    const Pose2& ref = estimates_.at<Pose2>(pose_ids_[pose_ids_.size() - 11]);

    // std::cout << "ref " << ref << std::endl;

    GetFKDInput(input);

    // get output x, y, t
    fkd_model_.evaluate(input, output);

    // add between factors for next 10 poses
    for (int i = 0; i < 10; i++) {
        // poses in frame of most recent pose, 
        uint32_t pose_id = pose_ids_.size() - (10 - i);

        if (i > 0) {
          // transform from ref frame to prev frame
          float dx = output[30+i] - output[30+i-1];
          float dy = output[40+i] - output[40+i-1];
          float ref_ang = cosAndSinToTheta(output[50+i-1], output[60+i-1]);
          float prev_ang = cosAndSinToTheta(output[50+i], output[60+i]);
          dx = dx*cos(-ref_ang)-dy*sin(-ref_ang);
          dy = dx*sin(-ref_ang)+dy*cos(-ref_ang);
          float dtheta = AngleMod(prev_ang - ref_ang); // TODO: verify this subtraction is happening in the right order
          graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(pose_ids_[pose_id - 1], pose_ids_[pose_id], gtsam::Pose2(dx, dy, dtheta), odometry_noise_));
        } else {
          graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(pose_ids_[pose_id - 1], pose_ids_[pose_id], gtsam::Pose2(output[30], output[40], cosAndSinToTheta(output[50], output[60])), odometry_noise_));
        }

        float ref_theta = (float) ref.theta();
        float x = output[30+i]*cos(ref_theta)-output[40+i]*sin(ref_theta) + ref.x();
        float y = output[30+i]*sin(ref_theta)+output[40+i]*cos(ref_theta) + ref.y();
        float theta = AngleMod(cosAndSinToTheta(output[50+i], output[60+i]) + ref_theta);
        estimates_.update(pose_ids_[pose_id], gtsam::Pose2(x, y, theta));
    }

}

void LocalizeFKD::OptimizeEstimates() {
    gtsam::LevenbergMarquardtOptimizer optimzer(graph_, estimates_);
    estimates_ = optimzer.optimize();

    // set robot pose and angle to optimized values
    gtsam::Pose2 robot_pose = estimates_.at<gtsam::Pose2>(pose_ids_.back());
    robot_loc_ = Vector2f(robot_pose.x(), robot_pose.y());
    robot_angle_ = robot_pose.theta();
}

void LocalizeFKD::UpdateVisualization() {

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
    if (estimates_.exists<Pose2>(pose_ids_[i])) {
      Pose2 p = estimates_.at<Pose2>(pose_ids_[i]);
      Vector2f loc(p.x(), p.y());
      visualization::DrawArc(loc, 0.06, 0, 6.28, 0x000000, global_viz_msg_);
      visualization::DrawLine(loc, loc + Vector2f(0.1*cos(p.theta()), 0.1*sin(p.theta())), 0x000000, global_viz_msg_);
    }
  }

  // draw circles seen
  for (auto circle : circles_) {
    visualization::DrawArc(circle.center, circle.radius, 0, 6.28, 0xd49100, local_viz_msg_);
  }
}

void LocalizeFKD::RunWithFKD() {
  // this if statement is just to not add poses when the robot is stationary at the beginning of the bag file
  if (robot_loc_.norm() > 0.1)
    AddNewPose(); // match new landmarks and add poses but no between factors, MUST HAPPEN EVERY CYCLE

  // every 0.5 seconds (or 10 poses), optimize prev 10 poses
  if (pose_ctr_ == next_fkd_pose_) {
    // std::cout << pose_ctr_ << std::endl;
    AddBetweenFactors();
    // OptimizeEstimates();
    next_fkd_pose_ += 10;
  }

  if (pose_ctr_ == next_optim_pose_) {
    // std::cout << pose_ctr_ << std::endl;
    OptimizeEstimates();
    next_optim_pose_ += 20;
  }
}

void LocalizeFKD::RunWithOdom() {
  // this if statement is just to not add poses when the robot is stationary at the beginning of the bag file
  if (robot_loc_.norm() > 0.1)
    AddNewPoseNoEst(); // match new landmarks and add poses but no between factors, MUST HAPPEN EVERY CYCLE

  if (pose_ctr_ == next_optim_pose_) {
    // std::cout << pose_ctr_ << std::endl;
    OptimizeEstimates();
    next_optim_pose_ += 20;
  }
}

void LocalizeFKD::Run() {
  // This function gets called 20 times a second to form the control loop.

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;
  if (!imu_initialized_) return;

  UpdateVisualization();

  if (FLAGS_fkd)
    RunWithFKD();
  else
    RunWithOdom();
  
  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();

  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
}

}  // namespace localize_fkd
