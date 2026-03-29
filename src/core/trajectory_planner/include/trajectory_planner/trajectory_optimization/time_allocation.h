/**
 * @file time_allocation.h
 * @brief Simple per-segment time allocation utilities.
 *
 * This implementation is original to this repository.
 * It provides a trapezoidal (acc-limited) time allocation commonly used for
 * waypoint-to-waypoint parameterization.
 */
#ifndef RPP_TRAJECTORY_OPTIMIZATION_TIME_ALLOCATION_H_
#define RPP_TRAJECTORY_OPTIMIZATION_TIME_ALLOCATION_H_

#include <algorithm>
#include <cmath>
#include <vector>

namespace rpp
{
namespace trajectory_optimization
{

class TimeAllocator
{
public:
  TimeAllocator() = default;
  ~TimeAllocator() = default;

  template <typename Point>
  static void trapezoidalAllocation(const std::vector<Point>& waypoints, double v_max, double a_max,
                                    std::vector<double>& time_allocations)
  {
    time_allocations.clear();
    if (waypoints.size() < 2)
    {
      return;
    }

    const double v = (std::isfinite(v_max) && v_max > 1e-6) ? v_max : 1.0;
    const double a = (std::isfinite(a_max) && a_max > 1e-6) ? a_max : 1.0;

    time_allocations.reserve(waypoints.size() - 1);

    // Trapezoidal profile per segment (no coupling between segments).
    // If segment is short, fall back to a triangular profile.
    const double t_acc = v / a;
    const double d_acc = 0.5 * a * t_acc * t_acc;  // = 0.5 * v^2 / a

    for (std::size_t i = 0; i + 1 < waypoints.size(); ++i)
    {
      const double dx = waypoints[i + 1].x() - waypoints[i].x();
      const double dy = waypoints[i + 1].y() - waypoints[i].y();
      const double d = std::hypot(dx, dy);

      double t = 0.0;
      if (!std::isfinite(d) || d <= 1e-9)
      {
        t = 0.0;
      }
      else if (d < 2.0 * d_acc)
      {
        // Triangular: accelerate then decelerate without reaching v.
        t = 2.0 * std::sqrt(d / a);
      }
      else
      {
        // Trapezoidal: accel, cruise, decel.
        const double d_cruise = d - 2.0 * d_acc;
        t = 2.0 * t_acc + d_cruise / v;
      }

      // Avoid zeros to prevent divide-by-zero in downstream normalization.
      const double t_min = 1e-3;
      time_allocations.push_back(std::max(t, t_min));
    }
  }
};

}  // namespace trajectory_optimization
}  // namespace rpp

#endif
