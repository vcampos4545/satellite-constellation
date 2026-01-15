#ifndef SSO_TEST_SCENARIO_H
#define SSO_TEST_SCENARIO_H

#include "Scenario.h"

/**
 * Sun-Synchronous Orbit Test Scenario
 *
 * Simple test scenario with a single satellite in SSO.
 *
 * Configuration:
 * - Altitude: 700 km
 * - Inclination: 98° (sun-synchronous)
 * - Duration: 1.5 hours (1 orbit)
 * - Time warp: 10x
 *
 * Purpose:
 * - Test basic orbital mechanics
 * - Verify SSO precession
 * - Test spacecraft attitude control
 */
class SSOTestScenario : public Scenario
{
public:
  SSOTestScenario() = default;
  ~SSOTestScenario() = default;

  // Scenario interface
  void setup(Universe &universe) override;
  void run(Simulation &sim) override;
  void teardown() override;

  // Scenario metadata
  std::string getName() const override { return "sso-test"; }

  std::string getDescription() const override
  {
    return "Sun-Synchronous Orbit Test - Single satellite at 700km, 98° inclination";
  }

  double getDuration() const override { return 5400.0; } // 1.5 hours
  bool requiresGUI() const override { return true; }
  double getTimeWarp() const override { return 10.0; }
  double getTimeStep() const override { return 0.1; }
};

#endif // SSO_TEST_SCENARIO_H
