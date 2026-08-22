// SPDX-License-Identifier: MIT
#include "ump/airspace.hpp"
 
#include <algorithm>
#include <cmath>
#include <limits>
 
namespace ump {
namespace {
 
/// Do the 2-D segments p1p2 and p3p4 properly intersect or touch?
bool segmentsCrossXY(double x1, double y1, double x2, double y2, double x3, double y3, double x4,
                     double y4) {
  const double d1 = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
  const double d2 = (x2 - x1) * (y4 - y1) - (y2 - y1) * (x4 - x1);
  const double d3 = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
  const double d4 = (x4 - x3) * (y2 - y3) - (y4 - y3) * (x2 - x3);
  if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
      ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
    return true;
  // Collinear-touching cases are treated as intersections: a leg that grazes
  // a boundary is rejected rather than admitted (fail-safe, SRS-012).
  const double kEps = 1e-9;
  auto onSeg = [](double ax, double ay, double bx, double by, double px, double py) {
    return std::min(ax, bx) - 1e-9 <= px && px <= std::max(ax, bx) + 1e-9 &&
           std::min(ay, by) - 1e-9 <= py && py <= std::max(ay, by) + 1e-9;
  };
  if (std::fabs(d1) < kEps && onSeg(x1, y1, x2, y2, x3, y3)) return true;
  if (std::fabs(d2) < kEps && onSeg(x1, y1, x2, y2, x4, y4)) return true;
  if (std::fabs(d3) < kEps && onSeg(x3, y3, x4, y4, x1, y1)) return true;
  if (std::fabs(d4) < kEps && onSeg(x3, y3, x4, y4, x2, y2)) return true;
  return false;
}
 
}  // namespace
 
void NoFlyZone::finalize() {
  min_x = min_y = std::numeric_limits<double>::max();
  max_x = max_y = std::numeric_limits<double>::lowest();
  for (const auto& v : vertices) {
    min_x = std::min(min_x, v[0]);
    max_x = std::max(max_x, v[0]);
    min_y = std::min(min_y, v[1]);
    max_y = std::max(max_y, v[1]);
  }
}
 
bool NoFlyZone::containsXY(double x, double y) const {
  if (vertices.size() < 3) return false;
  if (x < min_x || x > max_x || y < min_y || y > max_y) return false;  // AABB reject
  bool inside = false;
  const std::size_t n = vertices.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = vertices[i][0], yi = vertices[i][1];
    const double xj = vertices[j][0], yj = vertices[j][1];
    if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) inside = !inside;
  }
  return inside;
}
 
bool NoFlyZone::contains(const Vec3& p) const {
  if (p.z < floor_m || p.z > ceiling_m) return false;
  return containsXY(p.x, p.y);
}
 
bool NoFlyZone::segmentIntersects(const Vec3& a, const Vec3& b, double step_m) const {
  // Altitude band reject: the leg is clear if it stays entirely above the
  // ceiling or entirely below the floor.
  const double lo = std::min(a.z, b.z);
  const double hi = std::max(a.z, b.z);
  if (lo > ceiling_m || hi < floor_m) return false;
 
  // Either endpoint inside settles it.
  if (contains(a) || contains(b)) return true;
 
  // Sampled interior test catches legs that pass through the volume.
  const double d = distXY(a, b);
  const int n = std::max(1, static_cast<int>(std::ceil(d / std::max(step_m, 1.0))));
  for (int k = 0; k <= n; ++k) {
    const double t = static_cast<double>(k) / n;
    const Vec3 p{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
    if (contains(p)) return true;
  }
 
  // Exact XY boundary crossing test, guarded by the altitude band above.
  // This catches thin slivers the sampler could step over.
  const std::size_t n_v = vertices.size();
  for (std::size_t i = 0, j = n_v - 1; i < n_v; j = i++) {
    if (segmentsCrossXY(a.x, a.y, b.x, b.y, vertices[j][0], vertices[j][1], vertices[i][0],
                        vertices[i][1]))
      return true;
  }
  return false;
}
 
void Airspace::add(NoFlyZone zone) {
  zone.finalize();
  zones_.push_back(std::move(zone));
}
 
bool Airspace::blocked(const Vec3& p) const {
  for (const auto& z : zones_)
    if (z.contains(p)) return true;
  return false;
}
 
std::string Airspace::violatedBy(const Vec3& p) const {
  for (const auto& z : zones_)
    if (z.contains(p)) return z.id;
  return {};
}
 
bool Airspace::segmentBlocked(const Vec3& a, const Vec3& b, double step_m) const {
  for (const auto& z : zones_)
    if (z.segmentIntersects(a, b, step_m)) return true;
  return false;
}
 
} 
