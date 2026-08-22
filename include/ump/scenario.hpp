// Line-oriented scenario file parser.
#pragma once
 
#include <string>
 
#include "ump/airspace.hpp"
#include "ump/energy.hpp"
#include "ump/geo.hpp"
#include "ump/terrain.hpp"
 
namespace ump {
 
/// Fully specified planning problem loaded from a scenario file.
struct Scenario {
  LatLon origin{29.6436, -82.3549};
  Terrain terrain;
  Airspace airspace;
  AircraftModel aircraft;
  Vec3 start;
  Vec3 goal;
  bool start_alt_is_agl{true};
  bool goal_alt_is_agl{true};
};
 
/// Parse a scenario file. Throws std::runtime_error with the offending line
/// number on malformed input (SRS-003).
Scenario loadScenario(const std::string& path);
 
} 
