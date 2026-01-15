#ifndef GPS_SCENARIO_H
#define GPS_SCENARIO_H

#include "Scenario.h"

/**
 * GPS Constellation Scenario
 *
 * Full GPS constellation with 24 satellites.
 *
 * Configuration:
 * - 6 orbital planes at 55° inclination
 * - 4 satellites per plane
 * - Altitude: ~20,200 km (MEO)
 * - Circular orbits
 * - Duration: 12 hours (1 orbit)
 * - Time warp: 50x
 *
 * Purpose:
 * - Demonstrate MEO constellation
 * - Test multi-plane orbital mechanics
 * - Visualize global coverage
 */
class GPSScenario : public Scenario
{
public:
  GPSScenario() = default;
  ~GPSScenario() = default;

  // Scenario interface
  void setup(Universe &universe) override;
  void run(Simulation &sim) override;
  void teardown() override;

  // Scenario metadata
  std::string getName() const override { return "gps"; }

  std::string getDescription() const override
  {
    return "GPS Constellation - 24 satellites in 6 planes at 20,200km altitude";
  }

  double getDuration() const override { return 43200.0; } // 12 hours
  bool requiresGUI() const override { return true; }
  double getTimeWarp() const override { return 50.0; }
  double getTimeStep() const override { return 0.1; }
};

#endif // GPS_SCENARIO_H
