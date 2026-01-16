#ifndef FLIGHT_SOFTWARE_TASK_H
#define FLIGHT_SOFTWARE_TASK_H

#include <string>

// Forward declaration
class Spacecraft;

/**
 * Base class for flight software tasks
 *
 * Flight software defines the "brain" of the satellite - how it makes decisions,
 * controls its attitude, manages power, performs station keeping, etc.
 *
 * The Spacecraft class provides hardware capabilities (sensors, actuators, power system),
 * while FlightSoftwareTask subclasses implement the logic for using that hardware.
 *
 * This separation allows:
 * - Different FSW for different satellite types (Starlink, Cubesat, GPS, etc.)
 * - Easy scenario creation without modifying core Spacecraft class
 * - Testing FSW independently of physics simulation
 * - Realistic modeling of spacecraft autonomy
 */
class FlightSoftwareTask
{
public:
  virtual ~FlightSoftwareTask() = default;

  /**
   * Execute flight software for this frame
   *
   * Called every simulation tick. The FSW can:
   * - Read sensor data from satellite (position, velocity, attitude, power, etc.)
   * - Make decisions based on current state
   * - Command actuators (thrusters, reaction wheels, solar panels)
   * - Update internal FSW state
   *
   * @param satellite Pointer to the satellite hardware
   * @param deltaTime Time step in seconds
   */
  virtual void execute(Spacecraft *satellite, double deltaTime) = 0;

  /**
   * Get the name of this FSW task (for debugging/logging)
   */
  virtual std::string getName() const = 0;

  /**
   * Reset FSW state (called when simulation restarts or satellite resets)
   */
  virtual void reset() {}
};

#endif // FLIGHT_SOFTWARE_TASK_H
