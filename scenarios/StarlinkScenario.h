#ifndef STARLINK_SCENARIO_H
#define STARLINK_SCENARIO_H

#include "Scenario.h"

/**
 * Starlink Constellation Scenario
 *
 * Starlink LEO constellation at 550km altitude.
 *
 * Configuration:
 * - 8 orbital planes at 53° inclination
 * - 4 satellites per plane (32 total)
 * - Altitude: 550 km (LEO)
 * - Circular orbits
 * - Duration: 2 hours (~2 orbits)
 * - Time warp: 30x
 *
 * Purpose:
 * - Demonstrate LEO mega-constellation
 * - Test high-density satellite tracking
 * - Visualize global broadband coverage
 */
class StarlinkScenario : public Scenario
{
public:
  StarlinkScenario() = default;
  ~StarlinkScenario() = default;

  // Scenario interface
  void setup(Universe &universe) override;
  void run(Simulation &sim) override;
  void teardown() override;

  // Scenario metadata
  std::string getName() const override { return "starlink"; }

  std::string getDescription() const override
  {
    return "Starlink Constellation - 32 satellites in 8 planes at 550km altitude";
  }

  double getDuration() const override { return 7200.0; } // 2 hours
  bool requiresGUI() const override { return true; }
  double getTimeWarp() const override { return 30.0; }
  double getTimeStep() const override { return 0.1; }
};

#endif // STARLINK_SCENARIO_H
