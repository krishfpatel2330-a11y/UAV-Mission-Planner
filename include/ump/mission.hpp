// Post-processing and flight-plan export.
#pragma once
 
#include <string>
#include <vector>
 
#include "ump/energy.hpp"
#include "ump/geo.hpp"
#include "ump/planner.hpp"
 
namespace ump {
 
/// Greedy line-of-sight shortcutting: removes intermediate lattice vertices
/// whose omission leaves the leg collision-free and no more expensive.
std::vector<Vec3> shortcut(const World& world, const std::vector<Vec3>& path);
 
struct Waypoint {
  int seq{0};
  LatLon ll;
  double alt_msl_m{0.0};
  double agl_m{0.0};
  double cum_time_s{0.0};
  double cum_energy_wh{0.0};
};
 
std::vector<Waypoint> toWaypoints(const GeoRef& geo, const World& world,
                                  const std::vector<Vec3>& path);
 
/// CSV, one row per waypoint, for analysis and the plotting tool.
void writeWaypointCsv(const std::string& path, const std::vector<Waypoint>& wps);
 
/// QGroundControl .plan (MAV_CMD_NAV_WAYPOINT items plus a geofence section).
void writeQgcPlan(const std::string& path, const std::vector<Waypoint>& wps,
                  const Airspace& airspace, const GeoRef& geo);
} 
