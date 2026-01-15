#ifndef SCENARIO_H
#define SCENARIO_H

#include <string>

// Forward declarations
class Universe;
class Simulation;

/**
 * Scenario Base Class
 *
 * Defines the interface for simulation scenarios.
 * Each scenario configures the universe with spacecraft, components, FSW, etc.
 * and controls how the simulation runs.
 *
 * Lifecycle:
 * 1. setup() - Configure universe (add spacecraft, set parameters)
 * 2. run() - Execute simulation with this scenario
 * 3. teardown() - Optional cleanup
 *
 * Usage:
 *   class MyScenario : public Scenario {
 *       void setup(Universe& universe) override { ... }
 *       void run(Simulation& sim) override { ... }
 *       std::string getName() const override { return "my-scenario"; }
 *   };
 */
class Scenario
{
public:
  virtual ~Scenario() = default;

  /**
   * Setup the scenario
   * Configure the universe with spacecraft, components, FSW, etc.
   * @param universe Universe to configure
   */
  virtual void setup(Universe &universe) = 0;

  /**
   * Run the simulation
   * Controls how the simulation executes (time warp, duration, etc.)
   * @param sim Simulation instance to run
   */
  virtual void run(Simulation &sim) = 0;

  /**
   * Optional cleanup after scenario completes
   */
  virtual void teardown() {}

  /**
   * Get scenario name (used for command-line selection)
   * @return Scenario name (e.g., "sso-test")
   */
  virtual std::string getName() const = 0;

  /**
   * Get human-readable description
   * @return Description of what this scenario tests/demonstrates
   */
  virtual std::string getDescription() const = 0;

  /**
   * Get simulation duration in seconds
   * @return Duration (default: 1 day)
   */
  virtual double getDuration() const { return 86400.0; }

  /**
   * Check if scenario requires GUI
   * @return True if GUI should be enabled
   */
  virtual bool requiresGUI() const { return true; }

  /**
   * Get default time warp multiplier
   * @return Time warp (1.0 = real-time, 10.0 = 10x speed)
   */
  virtual double getTimeWarp() const { return 1.0; }

  /**
   * Get physics time step
   * @return Maximum physics timestep in seconds
   */
  virtual double getTimeStep() const { return 0.1; }
};

#endif // SCENARIO_H
