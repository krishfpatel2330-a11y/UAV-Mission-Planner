// Digital elevation model with bilinear sampling.
#pragma once
 
#include <cstdint>
#include <string>
#include <vector>
 
#include "ump/geo.hpp"
 
namespace ump {
 
/// Regular-grid digital elevation model in the local ENU frame.
///
/// Sample (i,j) sits at ENU (i*cell, j*cell). Elevations are metres MSL.
class Terrain {
 public:
  Terrain() = default;
  Terrain(int nx, int ny, double cell_m, std::vector<float> z);
 
  /// Deterministic synthetic DEM: a ridge line plus Gaussian peaks.
  /// Reproducible for a given seed so plans are repeatable (SRS-002).
  static Terrain synthetic(int nx, int ny, double cell_m, std::uint32_t seed);
 
  /// ASCII DEM: "nx ny cell" header followed by nx*ny elevations, row-major.
  static Terrain loadAscii(const std::string& path);
  void saveAscii(const std::string& path) const;
 
  /// Bilinearly interpolated elevation [m MSL]. Queries outside the DEM are
  /// clamped to the border rather than extrapolated.
  double elevation(double x_m, double y_m) const;
 
  /// Highest terrain sampled along the straight segment a->b.
  double maxElevationAlong(const Vec3& a, const Vec3& b, double step_m) const;
 
  int nx() const { return nx_; }
  int ny() const { return ny_; }
  double cell_m() const { return cell_m_; }
  double width_m() const { return (nx_ - 1) * cell_m_; }
  double height_m() const { return (ny_ - 1) * cell_m_; }
  const std::vector<float>& data() const { return z_; }
 
 private:
  int nx_{0};
  int ny_{0};
  double cell_m_{1.0};
  std::vector<float> z_;
};
 
}
