#pragma once

#include <cmath>

namespace teb_local_planner
{
namespace detail
{
inline int computeCohenSutherlandOutCode(
  double x, double y,
  double xmin, double ymin, double xmax, double ymax)
{
  
  constexpr int LEFT = 1;
  constexpr int RIGHT = 2;
  constexpr int BOTTOM = 4;
  constexpr int TOP = 8;
  int code = 0;
  if (x < xmin) {
    code |= LEFT;
  } else if (x > xmax) {
    code |= RIGHT;
  }
  if (y < ymin) {
    code |= BOTTOM;
  } else if (y > ymax) {
    code |= TOP;
  }
  return code;
}

inline bool clipWorldSegmentToAlignedRect(
  double & x0,
  double & y0,
  double & x1,
  double & y1,
  double xmin,
  double ymin,
  double xmax,
  double ymax)
{
  
  // Cohen-Sutherland 线段裁剪：将世界系线段裁到轴对齐矩形窗口内。
  constexpr double kEpsilonSeg = 1e-14;
  int out_code0 =
    computeCohenSutherlandOutCode(x0, y0, xmin, ymin, xmax, ymax);
  int out_code1 =
    computeCohenSutherlandOutCode(x1, y1, xmin, ymin, xmax, ymax);

  for (int iterations = 0; iterations < 64; ++iterations) {
    if (!(out_code0 | out_code1)) {
      const double dx = x1 - x0;
      const double dy = y1 - y0;
      return (dx * dx + dy * dy > kEpsilonSeg * kEpsilonSeg);
    }
    if (out_code0 & out_code1) {
      return false;
    }

    const int outside = out_code0 ? out_code0 : out_code1;
    double x_intersect = x0;
    double y_intersect = y0;
    constexpr int TOP = 8;
    constexpr int BOTTOM = 4;
    constexpr int RIGHT = 2;
    constexpr int LEFT = 1;

    if (outside & TOP) {
      const double denom = y1 - y0;
      if (std::fabs(denom) > kEpsilonSeg) {
        x_intersect = x0 + (x1 - x0) * (ymax - y0) / denom;
      } else {
        x_intersect = x0;
      }
      y_intersect = ymax;
    } else if (outside & BOTTOM) {
      const double denom = y1 - y0;
      if (std::fabs(denom) > kEpsilonSeg) {
        x_intersect = x0 + (x1 - x0) * (ymin - y0) / denom;
      } else {
        x_intersect = x0;
      }
      y_intersect = ymin;
    } else if (outside & RIGHT) {
      const double denom = x1 - x0;
      if (std::fabs(denom) > kEpsilonSeg) {
        y_intersect = y0 + (y1 - y0) * (xmax - x0) / denom;
      } else {
        y_intersect = y0;
      }
      x_intersect = xmax;
    } else if (outside & LEFT) {
      const double denom = x1 - x0;
      if (std::fabs(denom) > kEpsilonSeg) {
        y_intersect = y0 + (y1 - y0) * (xmin - x0) / denom;
      } else {
        y_intersect = y0;
      }
      x_intersect = xmin;
    }

    if (outside == out_code0) {
      x0 = x_intersect;
      y0 = y_intersect;
      out_code0 =
        computeCohenSutherlandOutCode(x0, y0, xmin, ymin, xmax, ymax);
    } else {
      x1 = x_intersect;
      y1 = y_intersect;
      out_code1 =
        computeCohenSutherlandOutCode(x1, y1, xmin, ymin, xmax, ymax);
    }
  }
  return false;
}

}  // namespace detail
}  // namespace teb_local_planner
