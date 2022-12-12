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
\file    navigation.h
\brief   Interface for reference PitchTest class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"

#include "vector_map/vector_map.h"

#ifndef PITCHTEST_H
#define PITCHTEST_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace pitch_test {

struct Circle {
  Eigen::Vector2f center;
  float radius;

  Circle() = default;
  Circle(float x, float y, float r) : center(x, y), radius(r) {}
  Circle(Eigen::Vector2f c, float r) : center(c), radius(r) {}
};

class PitchTest {
 public:

   // Constructor
  explicit PitchTest(const std::string& map_file, ros::NodeHandle* n);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
//   void ExtractLandmarks(const std::vector<Eigen::Vector2f>& cloud);
  void UpdateIMU(float pitch, float yaw); // yaw is heading, pitch is angle with the ground

 private:

  // Whether odometry has been initialized.
  bool vectornav_initialized_;

  // Current robot orientation.
  float robot_angle_;
  float robot_pitch_;
  float initial_pitch_;
  // Prev robot location.
  float odom_angle_;

  std::vector<Eigen::Vector2f> point_cloud_;

  // Map of the environment.
  vector_map::VectorMap map_;
};

}  // namespace pitch_test

#endif  // PITCHTEST_H
