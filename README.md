# UAV Mission Planner

Terrain- and threat-aware route planning for a fixed-wing UAV. Given a digital elevation model, a set of prohibited volumes, and an aircraft energy budget, it produces an energy-optimal route and exports it as a flight plan. Two planners solve the same problem against the same cost function, so their behaviour can be compared directly: a lattice A\* search and a sampling-based RRT\*.

![Planned route over terrain and prohibited airspace](docs/mission_ridgeline.png)

*A\* (blue) skirts the southern edge of the threat ring at low level. RRT\* (orange) climbs over the ridge to the north-west. Both routes are legal; the right-hand panels show the vertical profile and cumulative energy.*

## What it does

The planner treats the world as a digital elevation model sampled bilinearly, with a hard minimum-AGL floor beneath the aircraft and an altitude ceiling above it. Prohibited airspace is modelled as polygons extruded between a floor and a ceiling altitude, which means a route may legally overfly a zone whose ceiling is low enough — and the planner will choose to do exactly that whenever climbing costs less energy than flying around.

That last point is the heart of the design: the search minimises *energy*, not distance. Climbing draws more power than cruising and descending draws less, and the time a leg takes is governed by whichever constraint binds, either the horizontal distance at cruise speed or the vertical displacement at the aircraft's climb-rate limit. Branches whose accumulated cost would exceed the usable budget — pack energy minus the reserve fraction — are pruned during the search itself, so a mission the aircraft cannot fly comes back as a clean failure rather than as a route that quietly runs the battery flat.

Once a route is found and smoothed, every leg is re-checked against every constraint by code that took no part in the search, and the tool exits non-zero if that independent check disagrees. The result is exported as a QGroundControl `.plan` file, containing waypoint items alongside the no-fly zones as geofence exclusion polygons, and as a waypoint CSV for analysis and plotting.

## Build and run

The project needs CMake 3.16 or newer and a C++17 compiler. GoogleTest is fetched automatically during configuration, so there is nothing else to install.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure

./build/ump-plan --scenario config/scenario_ridgeline.txt \
                 --planner astar --out mission --dem dem.txt
```

Planning against the reference scenario prints a summary of the route and the verification result:

```
planner        : astar
waypoints      : 6
ground track   : 24.7 km
flight time    : 18.9 min
energy         : 95.1 Wh of 480.0 Wh usable (19.8 %)
min clearance  : 64.9 m AGL (limit 60.0 m)
search effort  : 84480 iterations in 1238.1 ms
verification   : PASS corridor, PASS energy
```

The figure at the top of this file is produced by the bundled visualiser, which overlays the route on the DEM and draws the vertical profile and energy curve beside it. Passing more than one waypoint CSV plots the routes together for comparison.

```sh
python3 tools/plot_mission.py --dem dem.txt --scenario config/scenario_ridgeline.txt \
        --mission mission.csv --out mission.png
```

## Scenario files

A scenario is plain text, one directive per line, with `#` introducing a comment. Coordinates are metres east and north of the geodetic origin, and altitudes are metres given as either `agl` or `msl`. Terrain is either generated synthetically from a seed, which keeps a plan reproducible from its scenario file alone, or loaded from an ASCII DEM. The aircraft directives override individual fields of the performance model, and each `nfz` line defines one prohibited polygon with its altitude band.

```
origin 29.6436 -82.3549
terrain synthetic 201 201 100 7        # nx ny cell_m seed  (or: terrain file dem.txt)

aircraft cruise_speed_mps  22
aircraft climb_power_w    900
aircraft energy_wh        600
aircraft reserve_frac    0.20
aircraft min_agl_m         60

start agl  1500  1500 120
goal  agl 18000 17500 120

nfz SAM_RING_ALPHA floor 0 ceiling 1200 poly 7000 6000 11000 6000 11000 10500 7000 10500
```

Malformed input is rejected with the offending line number rather than a stack trace.

## How the planners differ

A\* searches a 26-connected lattice anchored on the start state, with horizontal spacing set by `--cell` and vertical spacing by `--alt-step`. Its heuristic is the straight-line horizontal distance to the goal multiplied by the least energy the aircraft could possibly spend per metre travelled. Because leg time is always at least the horizontal distance divided by cruise speed, and leg power is always at least the minimum of the three power settings, that product is a true lower bound on the remaining cost — which makes the heuristic admissible and the returned path optimal on the lattice. The test suite checks that bound against twenty thousand randomised legs rather than asserting it in a comment.

RRT\* instead samples the continuous space, steers toward each sample, connects the new node through the cheapest parent in its neighbourhood, and rewires nearby nodes whose cost the new node improves. It keeps sampling after the first goal connection is found, which is what drives the solution cost downward toward the optimum. Both planners hand their output to the same greedy line-of-sight shortcutter, which removes an intermediate waypoint only when the replacement leg is both collision-free and no more expensive than the legs it replaces.

On the reference scenario, A\* at a 400 m lattice spacing flies 25.0 km for 96.4 Wh in 461 ms of search; tightening to 300 m gives 24.5 km and 94.7 Wh in 696 ms, 200 m gives 24.7 km and 95.1 Wh in 1238 ms, and 150 m gives 24.5 km and 94.7 Wh in 1823 ms. RRT\* with a 5,000-sample budget finds 24.2 km for 89.6 Wh in 47 ms, improves to 86.2 Wh at 15,000 samples, and reaches 85.6 Wh at 40,000 samples.

The interesting result is that RRT\* beats A\* by roughly ten percent on energy. A\* is genuinely optimal on its lattice, but the lattice restricts headings to 45-degree increments, so the true optimum is not representable in its search space at all — and refining the cell size cannot fix a discretisation of *heading*, which is why the A\* numbers flatten out instead of converging. RRT\* samples continuously and converges toward the real optimum, visibly so as its sample budget grows. What it gives up is determinism: A\* returns the same route on every run, and RRT\* does not.

## Requirements and verification

The file `docs/requirements.md` holds a numbered requirements list with each requirement traced to the test that verifies it, in the style of DO-178C traceability. Twenty-five tests cover the coordinate frames, the energy model, constraint enforcement, heuristic admissibility, the safety of path shortcutting, and export integrity. Continuous integration builds in both Debug and Release, runs the suite again under AddressSanitizer and UndefinedBehaviorSanitizer, and executes an end-to-end plan with each planner. The library compiles clean under `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror`.

## Layout

Public headers live under `include/ump` — `geo`, `terrain`, `airspace`, `energy`, `planner`, `mission`, and `scenario`. Implementations sit in `src`, with each planner in its own translation unit. The GoogleTest suites in `tests` are named for the behaviour they verify rather than the function they call. `tools` holds the matplotlib visualiser, `config` the scenario files, and `docs` the requirements and verification matrix.

## Scope

Several limitations are deliberate rather than accidental, and are recorded with their rationale at the end of `docs/requirements.md`. Legs are straight lines, so the routes are not yet flyable by an aircraft with a finite turn radius; a Dubins-airplane steering function is the natural next step. Routes may pass arbitrarily close to a zone boundary, where real planning would apply a lateral buffer sized by navigation uncertainty. Zones are static and there is no wind field, which is the single largest omission in the energy model. And the planner handles one vehicle against one objective, with no deconfliction between aircraft.
