// SPDX-License-Identifier: MIT
#include "ump/planner.hpp"
 
#include <algorithm>
#include <cmath>
 
namespace ump {
 
bool World::stateValid(const Vec3& p) const {
  if (p.x < 0.0 || p.y < 0.0 || p.x > terrain_.width_m() || p.y > terrain_.height_m()) return false;
  if (p.z > aircraft_.max_alt_msl_m) return false;
  if (p.z - terrain_.elevation(p.x, p.y) < aircraft_.min_agl_m) return false;
  if (airspace_.blocked(p)) return false;
  return true;
}
 
std::string World::rejectReason(const Vec3& p) const {
  if (p.x < 0.0 || p.y < 0.0 || p.x > terrain_.width_m() || p.y > terrain_.height_m())
    return "outside DEM coverage";
  if (p.z > aircraft_.max_alt_msl_m) return "above ceiling";
  const double agl = p.z - terrain_.elevation(p.x, p.y);
  if (agl < aircraft_.min_agl_m) return "terrain clearance violated (AGL " +
                                        std::to_string(static_cast<int>(agl)) + " m)";
  const std::string z = airspace_.violatedBy(p);
  if (!z.empty()) return "inside no-fly zone " + z;
  return {};
}
 
bool World::edgeValid(const Vec3& a, const Vec3& b) const {
  if (!stateValid(a) || !stateValid(b)) return false;
  const double d = dist3(a, b);
  const int n = std::max(1, static_cast<int>(std::ceil(d / std::max(collision_step_m, 1.0))));
  for (int k = 1; k < n; ++k) {
    const double t = static_cast<double>(k) / n;
    if (!stateValid({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t}))
      return false;
  }
  return !airspace_.segmentBlocked(a, b, collision_step_m);
}
 
PlanResult summarize(const AircraftModel& ac, std::vector<Vec3> path, std::string planner) {
  PlanResult r;
  r.planner = std::move(planner);
  r.path = std::move(path);
  r.success = r.path.size() >= 2;
  for (std::size_t i = 1; i < r.path.size(); ++i) {
    const LegCost c = legCost(ac, r.path[i - 1], r.path[i]);
    r.time_s += c.time_s;
    r.energy_wh += c.energy_wh;
    r.length_m += dist3(r.path[i - 1], r.path[i]);
  }
  return r;
}
 
}  // namespace ump
 
