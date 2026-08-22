// Fixed-wing endurance model used as the planner's edge cost.
#pragma once
 
#include "ump/geo.hpp"
 
namespace ump {
 
/// Performance envelope and energy budget of the air vehicle.
/// Defaults approximate a 25 kg-class electric fixed-wing ISR platform.
struct AircraftModel {
  double cruise_speed_mps{22.0};   ///< true airspeed in level flight
  double cruise_power_w{350.0};    ///< shaft power, level flight
  double climb_power_w{900.0};     ///< shaft power while climbing
  double descent_power_w{150.0};   ///< shaft power in a powered descent
  double max_climb_rate_mps{3.0};  ///< best rate of climb
  double max_sink_rate_mps{4.0};   ///< max commanded rate of descent
  double energy_wh{600.0};         ///< usable pack energy before reserve
  double reserve_frac{0.20};       ///< fraction withheld for diversion/loiter
  double min_agl_m{60.0};          ///< minimum terrain clearance
  double max_alt_msl_m{1200.0};    ///< ceiling (regulatory or performance)
 
  /// Energy available to the mission after the reserve is withheld.
  double usableWh() const { return energy_wh * (1.0 - reserve_frac); }
};
 
/// Time and energy to fly one straight leg.
struct LegCost {
  double time_s{0.0};
  double energy_wh{0.0};
};
 
/// Cost of the leg a->b.
///
/// Leg time is governed by whichever constraint binds: horizontal distance at
/// cruise speed, or the vertical displacement at the climb/sink rate limit.
/// Power is selected by the sign of the altitude change.
LegCost legCost(const AircraftModel& ac, const Vec3& a, const Vec3& b);
 
/// Greatest lower bound on energy consumed per metre of horizontal travel.
///
/// Because leg time is at least (horizontal distance / cruise speed) and leg
/// power is at least min(cruise, descent, climb), multiplying this constant by
/// the straight-line horizontal distance to the goal yields an admissible A*
/// heuristic (SRS-020).
double minEnergyPerMeterWh(const AircraftModel& ac);
 
} 
