// Prohibited-airspace model: extruded polygons with floor/ceiling bands.
#pragma once
 
#include <array>
#include <string>
#include <vector>
 
#include "ump/geo.hpp"
 
namespace ump {
 
/// A no-fly zone: a simple polygon in the local XY plane, extruded between a
/// floor and a ceiling altitude (metres MSL).
struct NoFlyZone {
  std::string id;
  std::vector<std::array<double, 2>> vertices;
  double floor_m{0.0};
  double ceiling_m{1.0e6};
 
  double min_x{0.0}, min_y{0.0}, max_x{0.0}, max_y{0.0};  // cached AABB
 
  void finalize();
  bool containsXY(double x, double y) const;
  bool contains(const Vec3& p) const;
 
  /// True if the straight leg a->b penetrates the extruded volume.
  bool segmentIntersects(const Vec3& a, const Vec3& b, double step_m) const;
};
 
/// Collection of prohibited volumes.
class Airspace {
 public:
  void add(NoFlyZone zone);
  bool blocked(const Vec3& p) const;
  bool segmentBlocked(const Vec3& a, const Vec3& b, double step_m) const;
 
  /// Id of the first zone violated by p, or an empty string.
  std::string violatedBy(const Vec3& p) const;
 
  const std::vector<NoFlyZone>& zones() const { return zones_; }
 
 private:
  std::vector<NoFlyZone> zones_;
};
 
}  
