// Geodetic <-> local ENU conversion and basic 3-D vector math.
#pragma once
 
#include <cmath>
 
namespace ump {
 
/// Geodetic position (WGS-84).
struct LatLon {
  double lat_deg{0.0};
  double lon_deg{0.0};
};
 
/// Local East-North-Up position.
/// x = east [m], y = north [m], z = altitude above mean sea level [m].
struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};
 
inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(const Vec3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
 
inline double norm3(const Vec3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }
inline double normXY(const Vec3& v) { return std::sqrt(v.x * v.x + v.y * v.y); }
inline double distXY(const Vec3& a, const Vec3& b) { return normXY(b - a); }
inline double dist3(const Vec3& a, const Vec3& b) { return norm3(b - a); }
 
/// Equirectangular (flat-earth) projection about a fixed origin.
///
/// Valid for the mission radii this planner targets (< ~100 km). The along-
/// meridian scale uses the local meridional radius of curvature; the along-
/// parallel scale is fixed at the origin latitude, so east-west error grows
/// with displacement in latitude. SRS-001 bounds the resulting error.
class GeoRef {
 public:
  explicit GeoRef(LatLon origin) : origin_(origin) {
    constexpr double kA = 6378137.0;          // WGS-84 semi-major axis [m]
    constexpr double kE2 = 6.69437999014e-3;  // first eccentricity squared
    constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
    const double s = std::sin(origin.lat_deg * kDeg2Rad);
    const double w = std::sqrt(1.0 - kE2 * s * s);
    const double m = kA * (1.0 - kE2) / (w * w * w);  // meridional radius
    const double n = kA / w;                          // prime vertical radius
    m_per_deg_lat_ = m * kDeg2Rad;
    m_per_deg_lon_ = n * std::cos(origin.lat_deg * kDeg2Rad) * kDeg2Rad;
  }
 
  Vec3 toEnu(LatLon p, double alt_msl_m) const {
    return {(p.lon_deg - origin_.lon_deg) * m_per_deg_lon_,
            (p.lat_deg - origin_.lat_deg) * m_per_deg_lat_, alt_msl_m};
  }
 
  LatLon toLatLon(const Vec3& p) const {
    return {origin_.lat_deg + p.y / m_per_deg_lat_, origin_.lon_deg + p.x / m_per_deg_lon_};
  }
 
  LatLon origin() const { return origin_; }
 
 private:
  LatLon origin_;
  double m_per_deg_lat_{0.0};
  double m_per_deg_lon_{0.0};
};
 
} 
