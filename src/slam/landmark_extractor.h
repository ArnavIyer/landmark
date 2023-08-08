#include <vector>

#include "eigen3/Eigen/Dense"

#ifndef EXTRACTOR_H
#define EXTRACTOR_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace ld_extractor {

struct Circle {
  Eigen::Vector2f center;
  float radius;

  Circle() = default;
  Circle(float x, float y, float r) : center(x, y), radius(r) {}
  Circle(Eigen::Vector2f c, float r) : center(c), radius(r) {}
};

using namespace std;

class LandmarkExtractor {
 public:
  static constexpr float LD_DIA = 0.159;
  static constexpr float LD_TOL = 0.05;
  static constexpr float RENDER_DIST = 3;

  static void ExtractLandmarks2(std::vector<Circle>& circles, const std::vector<Eigen::Vector2f>& cloud) {
    double epsilon = 0.1;
    circles.clear();
    // cout << "Circles Info: " << endl;
    for (size_t i = 1; i < cloud.size(); i++) {
      // cout << "  circle " << i << ":" << endl;
      std::vector<Eigen::Vector2f> circle;
      Eigen::Vector2f avg(0,0);
      float max_diff = 0;
      while (i < cloud.size() && (cloud[i] - cloud[i-1]).norm() < epsilon) {
        circle.push_back(cloud[i]);
        avg += cloud[i];
        max_diff = std::max(max_diff, (avg/circle.size() - cloud[i]).norm());
        i++;
      }
      avg /= circle.size();
      // cout << "    max_diff: " << max_diff << endl;
      // cout << "    circle size: " << circle.size() << endl;
      if (circle.size() < 10 || circle.size() > 50 || max_diff > LD_TOL + LD_DIA) {
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
      double radius = std::sqrt(xc*xc + yc*yc + (xx+yy)/circle.size());
      // cout << "    circle diameter: " << radius*2 << endl;
      if (radius*2 < LD_DIA - LD_TOL || radius*2 > LD_DIA + LD_TOL)
        continue;
      circles.push_back(Circle(xc+avg.x(), yc+avg.y(), radius));
    }
  }

  static void ExtractLandmarks(std::vector<Circle>& circles, const std::vector<Eigen::Vector2f>& cloud) {
    circles.clear();
    double epsilon = 0.05;
    for (size_t i = 1; i < cloud.size(); i++) {

      std::vector<Eigen::Vector2f> circle;
      Eigen::Vector2f avg(0,0);
      float max_diff = 0;
      while (i < cloud.size() && (cloud[i] - cloud[i-1]).norm() < epsilon) {
        circle.push_back(cloud[i]);
        avg += cloud[i];
        max_diff = std::max(max_diff, (avg - cloud[i]).norm());
        i++;
      }
      avg /= circle.size();
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
      double radius = std::sqrt(xc*xc + yc*yc + (xx+yy)/circle.size());
      circles.push_back(Circle(xc+avg.x(), yc+avg.y(), radius));
    }
  }
};

}  // namespace ld_frontend

#endif  // EXTRACTOR_H
