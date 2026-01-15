#ifndef GEO_SCENARIO_H
#define GEO_SCENARIO_H

#include "Scenario.h"

/**
 * Geostationary (GEO) Constellation Scenario
 *
 * 3 satellites in geostationary orbit.
 *
 * Configuration:
 * - 3 satellites evenly spaced around equator
 * - Altitude: ~35,786 km (GEO)
 * - 0° inclination (equatorial)
 * - Circular orbits
 * - Duration: 24 hours (1 full orbit)
 * - Time warp: 100x
 *
 * Purpose:
 * - Demonstrate geostationary orbits
 * - Test Earth-synchronous motion
 * - Visualize fixed ground coverage
 */
class GEOScenario : public Scenario
{
public:
  GEOScenario() = default;
  ~GEOScenario() = default;

  // Scenario interface
  void setup(Universe &universe) override;
  void run(Simulation &sim) override;
  void teardown() override;

  // Scenario metadata
  std::string getName() const override { return "geo"; }

  std::string getDescription() const override
  {
    return "Geostationary Orbit - 3 satellites at 35,786km altitude";
  }

  double getDuration() const override { return 86400.0; } // 24 hours
  bool requiresGUI() const override { return true; }
  double getTimeWarp() const override { return 100.0; }
  double getTimeStep() const override { return 0.1; }
};

#endif // GEO_SCENARIO_H
