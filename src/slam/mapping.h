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
\file    mapping.h
\brief   Interface for reference Mapping class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"

#include "landmark_extractor.h"
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>

#ifndef MAPPING_H
#define MAPPING_H

namespace ros {
  class NodeHandle;
}  // namespace ros

using ld_extractor::LandmarkExtractor;
using ld_extractor::Circle;

namespace mapping {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Mapping {
 public:

   // Constructor
  explicit Mapping(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);
  void UpdatePitch(double pitch, double yaw);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);
  void MatchLandmarks();
  void OptimizeEstimates();
  gtsam::Pose2 GetLocation();
  void UpdateVisualization();
  void UpdatePointCloud(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max,
                    float angle_increment);
  void AddNewPose();
  void PrintLandmarks();

 private:
  LandmarkExtractor extractor_;
  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;

  bool imu_initialized_;
  double imu_initial_pitch_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  double robot_pitch_;
  // Current robot orientation.
  float robot_angle_;
  // Prev robot location.
  Eigen::Vector2f prev_robot_loc_;
  // prev robot orientation.
  float prev_robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;
  std::vector<Circle> circles_;

  int pose_ctr_;
  int landmark_ctr_;
  int next_optim_pose_;
  // Map of the environment.

  // slam things
  gtsam::NonlinearFactorGraph graph_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr landmark_noise_;
  std::vector<gtsam::Symbol> landmark_ids_;
  std::vector<gtsam::Symbol> pose_ids_;
  gtsam::Values estimates_;
};

}  // namespace mapping

#endif  // NAVIGATION_H
