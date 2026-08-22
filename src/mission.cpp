#include "ump/mission.hpp"
 
#include <fstream>
#include <iomanip>
#include <stdexcept>
 
namespace ump {
 
std::vector<Vec3> shortcut(const World& world, const std::vector<Vec3>& path) {
  if (path.size() < 3) return path;
  const AircraftModel& ac = world.aircraft();
  std::vector<Vec3> out{path.front()};
  std::size_t i = 0;
  while (i + 1 < path.size()) {
    std::size_t best = i + 1;
    for (std::size_t j = path.size() - 1; j > i + 1; --j) {
      if (!world.edgeValid(path[i], path[j])) continue;
      // Only accept the shortcut if it does not cost more energy than the
      // legs it replaces (SRS-021).
      double replaced = 0.0;
      for (std::size_t k = i; k < j; ++k) replaced += legCost(ac, path[k], path[k + 1]).energy_wh;
      if (legCost(ac, path[i], path[j]).energy_wh <= replaced + 1e-9) {
        best = j;
        break;
      }
    }
    out.push_back(path[best]);
    i = best;
  }
  return out;
}
 
std::vector<Waypoint> toWaypoints(const GeoRef& geo, const World& world,
                                  const std::vector<Vec3>& path) {
  std::vector<Waypoint> wps;
  wps.reserve(path.size());
  double t = 0.0, e = 0.0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (i > 0) {
      const LegCost c = legCost(world.aircraft(), path[i - 1], path[i]);
      t += c.time_s;
      e += c.energy_wh;
    }
    Waypoint w;
    w.seq = static_cast<int>(i);
    w.ll = geo.toLatLon(path[i]);
    w.alt_msl_m = path[i].z;
    w.agl_m = path[i].z - world.terrain().elevation(path[i].x, path[i].y);
    w.cum_time_s = t;
    w.cum_energy_wh = e;
    wps.push_back(w);
  }
  return wps;
}
 
void writeWaypointCsv(const std::string& path, const std::vector<Waypoint>& wps) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot write " + path);
  out << std::fixed << std::setprecision(7);
  out << "seq,lat_deg,lon_deg,alt_msl_m,agl_m,cum_time_s,cum_energy_wh\n";
  for (const auto& w : wps) {
    out << w.seq << "," << w.ll.lat_deg << "," << w.ll.lon_deg << "," << std::setprecision(2)
        << w.alt_msl_m << "," << w.agl_m << "," << w.cum_time_s << "," << w.cum_energy_wh << "\n";
    out << std::setprecision(7);
  }
}
 
void writeQgcPlan(const std::string& path, const std::vector<Waypoint>& wps,
                  const Airspace& airspace, const GeoRef& geo) {
  if (wps.empty()) throw std::runtime_error("cannot export an empty mission");
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot write " + path);
  out << std::fixed << std::setprecision(7);
 
  out << "{\n  \"fileType\": \"Plan\",\n  \"version\": 1,\n";
  out << "  \"groundStation\": \"uav-mission-planner\",\n";
 
  // Geofence: each no-fly zone exported as a polygon inclusion=false.
  out << "  \"geoFence\": {\n    \"version\": 2,\n    \"circles\": [],\n    \"polygons\": [\n";
  for (std::size_t i = 0; i < airspace.zones().size(); ++i) {
    const auto& z = airspace.zones()[i];
    out << "      { \"inclusion\": false, \"version\": 1, \"polygon\": [";
    for (std::size_t v = 0; v < z.vertices.size(); ++v) {
      const LatLon ll = geo.toLatLon(Vec3{z.vertices[v][0], z.vertices[v][1], 0.0});
      out << (v ? ", " : "") << "[" << ll.lat_deg << ", " << ll.lon_deg << "]";
    }
    out << "] }" << (i + 1 < airspace.zones().size() ? "," : "") << "\n";
  }
  out << "    ]\n  },\n";
 
  out << "  \"mission\": {\n    \"version\": 2,\n    \"firmwareType\": 12,\n"
      << "    \"vehicleType\": 1,\n    \"cruiseSpeed\": 22,\n    \"hoverSpeed\": 5,\n";
  out << "    \"plannedHomePosition\": [" << wps.front().ll.lat_deg << ", "
      << wps.front().ll.lon_deg << ", " << std::setprecision(2) << wps.front().alt_msl_m
      << std::setprecision(7) << "],\n";
  out << "    \"items\": [\n";
  for (std::size_t i = 0; i < wps.size(); ++i) {
    const auto& w = wps[i];
    out << "      {\n        \"type\": \"SimpleItem\",\n        \"command\": 16,\n"
        << "        \"frame\": 0,\n        \"autoContinue\": true,\n"
        << "        \"doJumpId\": " << (i + 1) << ",\n"
        << "        \"params\": [0, 0, 0, null, " << w.ll.lat_deg << ", " << w.ll.lon_deg << ", "
        << std::setprecision(2) << w.alt_msl_m << std::setprecision(7) << "]\n      }"
        << (i + 1 < wps.size() ? "," : "") << "\n";
  }
  out << "    ]\n  },\n  \"rallyPoints\": { \"version\": 2, \"points\": [] }\n}\n";
}
 
}
