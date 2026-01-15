#ifndef MOLNIYA_SCENARIO_H
#define MOLNIYA_SCENARIO_H

#include "Scenario.h"

/**
 * Molniya Orbit Scenario
 *
 * 3 satellites in highly elliptical Molniya orbits.
 *
 * Configuration:
 * - 3 satellites with 120° phase separation
 * - Semi-major axis: 26,600 km
 * - Eccentricity: 0.72 (highly elliptical)
 * - Inclination: 63.4° (critical angle for frozen orbit)
 * - Apogee: ~39,800 km, Perigee: ~500 km
 * - Duration: 12 hours (1 orbit)
 * - Time warp: 50x
 *
 * Purpose:
 * - Demonstrate highly elliptical orbits
 * - Test frozen orbit mechanics (critical inclination)
 * - Visualize high-latitude coverage
 */
class MolniyaScenario : public Scenario
{
public:
  MolniyaScenario() = default;
  ~MolniyaScenario() = default;

  // Scenario interface
  void setup(Universe &universe) override;
  void run(Simulation &sim) override;
  void teardown() override;

  // Scenario metadata
  std::string getName() const override { return "molniya"; }

  std::string getDescription() const override
  {
    return "Molniya Orbit - 3 satellites in highly elliptical orbits for high-latitude coverage";
  }

  double getDuration() const override { return 43200.0; } // 12 hours
  bool requiresGUI() const override { return true; }
  double getTimeWarp() const override { return 50.0; }
  double getTimeStep() const override { return 0.1; }
};

#endif // MOLNIYA_SCENARIO_H
