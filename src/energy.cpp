// SPDX-License-Identifier: MIT
#include "ump/energy.hpp"
 
#include <algorithm>
#include <cmath>
 
namespace ump {
 
LegCost legCost(const AircraftModel& ac, const Vec3& a, const Vec3& b) {
  const double d_h = distXY(a, b);
  const double dz = b.z - a.z;
 
  const double t_horizontal = d_h / std::max(ac.cruise_speed_mps, 1e-6);
  const double rate = dz > 0.0 ? ac.max_climb_rate_mps : ac.max_sink_rate_mps;
  const double t_vertical = std::fabs(dz) / std::max(rate, 1e-6);
  const double t = std::max(t_horizontal, t_vertical);
 
  double power = ac.cruise_power_w;
  if (dz > 1e-6) power = ac.climb_power_w;
  else if (dz < -1e-6) power = ac.descent_power_w;
 
  return LegCost{t, power * t / 3600.0};
}
 
double minEnergyPerMeterWh(const AircraftModel& ac) {
  const double p_min = std::min({ac.cruise_power_w, ac.climb_power_w, ac.descent_power_w});
  return p_min / (std::max(ac.cruise_speed_mps, 1e-6) * 3600.0);
}
 
}  // namespace ump
 
