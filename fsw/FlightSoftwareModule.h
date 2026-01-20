#ifndef FLIGHT_SOFTWARE_MODULE_H
#define FLIGHT_SOFTWARE_MODULE_H

#include <string>

// Forward declarations
class Spacecraft;
struct SpacecraftEnvironment;

/**
 * FlightSoftwareModule
 *
 * Base class for all flight software modules.
 *
 * Flight software is completely decoupled from spacecraft physics.
 * Each module is responsible for a specific task:
 * - ADCSModule: Attitude determination and control
 * - PowerModule: Power management
 * - StationKeepingModule: Orbit maintenance
 * - CollisionAvoidanceModule: Collision detection and avoidance
 * - TelemetryModule: Data logging and downlink
 * - etc.
 *
 * Design Philosophy:
 * - Spacecraft = Pure physics state and dynamics
 * - FSW Modules = Read sensors, make decisions, command actuators
 * - Scenarios = Configure which FSW modules to use
 *
 * This allows:
 * - Testing spacecraft physics without FSW
 * - Testing FSW with mock spacecraft
 * - Easy swapping of FSW implementations
 * - Spacecraft without FSW (debris, passive satellites)
 */
class FlightSoftwareModule
{
public:
  FlightSoftwareModule(const std::string &name) : name(name), enabled(true) {}
  virtual ~FlightSoftwareModule() = default;

  /**
   * Update the flight software module
   * @param deltaTime Time step (seconds)
   * @param spacecraft Spacecraft being controlled
   * @param environment Universe environment data
   */
  virtual void update(double deltaTime, Spacecraft &spacecraft, const SpacecraftEnvironment &environment) = 0;

  /**
   * Get module name
   */
  virtual std::string getName() const { return name; }

  /**
   * Enable/disable module
   */
  void setEnabled(bool enable) { enabled = enable; }
  bool isEnabled() const { return enabled; }

protected:
  std::string name;
  bool enabled;
};

#endif // FLIGHT_SOFTWARE_MODULE_H
