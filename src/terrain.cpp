#include "ump/terrain.hpp"
 
#include <algorithm>
#include <cmath>
#include <fstream>
#include <random>
#include <stdexcept>
 
namespace ump {
 
Terrain::Terrain(int nx, int ny, double cell_m, std::vector<float> z)
    : nx_(nx), ny_(ny), cell_m_(cell_m), z_(std::move(z)) {
  if (nx_ < 2 || ny_ < 2) throw std::runtime_error("terrain: grid must be at least 2x2");
  if (cell_m_ <= 0.0) throw std::runtime_error("terrain: cell size must be positive");
  if (z_.size() != static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_))
    throw std::runtime_error("terrain: elevation count does not match grid size");
}
 
Terrain Terrain::synthetic(int nx, int ny, double cell_m, std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u01(0.0, 1.0);
 
  struct Peak {
    double x, y, h, s;
  };
  std::vector<Peak> peaks;
  const double w = (nx - 1) * cell_m;
  const double h = (ny - 1) * cell_m;
  const int n_peaks = 6;
  peaks.reserve(n_peaks);
  for (int i = 0; i < n_peaks; ++i) {
    peaks.push_back({u01(rng) * w, u01(rng) * h, 180.0 + 420.0 * u01(rng),
                     0.06 * w + 0.10 * w * u01(rng)});
  }
 
  std::vector<float> z(static_cast<std::size_t>(nx) * ny, 0.0f);
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      const double x = i * cell_m;
      const double y = j * cell_m;
      // Ridge running roughly south-west to north-east.
      const double t = (x / std::max(w, 1.0)) - (y / std::max(h, 1.0));
      double e = 220.0 * std::exp(-(t * t) / 0.02);
      for (const Peak& p : peaks) {
        const double dx = x - p.x;
        const double dy = y - p.y;
        e += p.h * std::exp(-(dx * dx + dy * dy) / (2.0 * p.s * p.s));
      }
      // Gentle regional slope so the DEM is never perfectly flat.
      e += 40.0 * (y / std::max(h, 1.0));
      z[static_cast<std::size_t>(j) * nx + i] = static_cast<float>(e);
    }
  }
  return Terrain(nx, ny, cell_m, std::move(z));
}
 
Terrain Terrain::loadAscii(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("terrain: cannot open " + path);
  int nx = 0, ny = 0;
  double cell = 0.0;
  in >> nx >> ny >> cell;
  if (!in) throw std::runtime_error("terrain: bad header in " + path);
  std::vector<float> z(static_cast<std::size_t>(nx) * ny);
  for (auto& v : z) {
    if (!(in >> v)) throw std::runtime_error("terrain: truncated elevation data in " + path);
  }
  return Terrain(nx, ny, cell, std::move(z));
}
 
void Terrain::saveAscii(const std::string& path) const {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("terrain: cannot write " + path);
  out << nx_ << " " << ny_ << " " << cell_m_ << "\n";
  for (int j = 0; j < ny_; ++j) {
    for (int i = 0; i < nx_; ++i) out << z_[static_cast<std::size_t>(j) * nx_ + i] << " ";
    out << "\n";
  }
}
 
double Terrain::elevation(double x_m, double y_m) const {
  const double fx = std::clamp(x_m / cell_m_, 0.0, static_cast<double>(nx_ - 1));
  const double fy = std::clamp(y_m / cell_m_, 0.0, static_cast<double>(ny_ - 1));
  const int i0 = static_cast<int>(fx);
  const int j0 = static_cast<int>(fy);
  const int i1 = std::min(i0 + 1, nx_ - 1);
  const int j1 = std::min(j0 + 1, ny_ - 1);
  const double tx = fx - i0;
  const double ty = fy - j0;
  const auto at = [&](int i, int j) { return static_cast<double>(z_[static_cast<std::size_t>(j) * nx_ + i]); };
  const double a = at(i0, j0) * (1.0 - tx) + at(i1, j0) * tx;
  const double b = at(i0, j1) * (1.0 - tx) + at(i1, j1) * tx;
  return a * (1.0 - ty) + b * ty;
}
 
double Terrain::maxElevationAlong(const Vec3& a, const Vec3& b, double step_m) const {
  const double d = distXY(a, b);
  const int n = std::max(1, static_cast<int>(std::ceil(d / std::max(step_m, 1.0))));
  double peak = -1.0e9;
  for (int k = 0; k <= n; ++k) {
    const double t = static_cast<double>(k) / n;
    peak = std::max(peak, elevation(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t));
  }
  return peak;
}
 
}  // namespace ump
