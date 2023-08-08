#include <vector>

#include "eigen3/Eigen/Dense"

#include "../slam/landmark_extractor.h"
#include "model_wrapper.h"
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>

#ifndef LOCALIZE_H
#define LOCALIZE_H

using std::vector;

namespace ros {
  class NodeHandle;
}  // namespace ros

using Circle = ld_extractor::Circle;
using LandmarkExtractor = ld_extractor::LandmarkExtractor;

namespace localize_fkd {

class Buffer {
public:
    vector<vector<float>> b;
    size_t curr;
    bool saturated;

    Buffer() = delete;

    Buffer(int size) {
        b = vector<vector<float>>(size);
        curr = 0;
        saturated = false;
    }

    ~Buffer() = default;

    void add(vector<float> a) {
        b[curr] = a;
        if (++curr == b.size()) {
            saturated = true;
            curr = 0;
        }
    }

    // fills data with the last n points of data
    // returns time of earliest command
    float last_n(float* data, int n) {
        assert(saturated);

        int idx = curr;
        for (int i = n-1; i >= 0; i--) {
            if (--idx == -1) {
                idx = b.size() - 1;
            }
            for (size_t j = 0; j < b[0].size() - 1; j++) {
                data[j*n+i] = b[idx][j];
            }
        }

        return b[idx][2];  
    }

    // currently has a lot of hardcoded stuff for realsense odom info
    void sample_last_n(float* data, int n) {
        // iterate till 0.05 seconds have passed, only then grab data
        int idx = curr;
        if (--idx == -1) {
            idx = b.size() - 1;
        }
        float time = b[idx][6] - 0.5;
        int data_ctr = 9;
        while (data_ctr >= 0) {
            if (time - b[idx][6] > 0) {
                time -= 0.05;
                // add idx to data

                for (int i = 0; i < 6; i++) {
                    data[i*n + data_ctr] = b[idx][i];
                }
                data_ctr -= 1;
            }

            if (--idx == -1) {
                idx = b.size() - 1;
            }
        }
    }

};

class LocalizeFKD {
 public:

   // Constructor
  explicit LocalizeFKD(const std::string& map_file, ros::NodeHandle* n);

  ~LocalizeFKD() = default;

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);
  void UpdateOrientation(float roll, double pitch, double yaw, Eigen::Quaternionf q);
  void UpdateCommand(float time, float v, float w);
  void AddBetweenFactors();
//   void FKDOutputToPoses(float* output);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);
    void UpdateRSOdom(vector<float>& data);

//    void GetFKDInputRS(float* input);


  // Main function called continously from main
  void Run();
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
  void AddNewPoseNoEst();

  float cosAndSinToTheta(float, float);

  void AddNextBetweenFactor();
  void GetFKDInput(float*);

  void RunWithFKD();
  void RunWithOdom();

 private:

  LandmarkExtractor extractor_;

  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;

  bool imu_initialized_;
  double imu_initial_pitch_;
  float imu_initial_roll_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  double robot_pitch_;
  float robot_roll_;
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
  std::vector<Eigen::Vector2f> landmarks_;  

  int pose_ctr_;
  int next_fkd_pose_;
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

  Buffer imu_buf_;
  Buffer joy_buf_;
  model::ModelWrapper fkd_model_;
};

}  // namespace localize_fkd

#endif  // NAVIGATION_H
