#ifndef THRUSTER_H
#define THRUSTER_H

#include "Component.h"
#include <glm/glm.hpp>
#include <string>

/**
 * Thruster Actuator Component
 *
 * Simulates a rocket thruster that produces force and torque through
 * propellant expulsion. Thrusters are used for:
 * - Orbit maneuvers (delta-V)
 * - Attitude control (RCS thrusters)
 * - Station keeping
 * - Deorbit burns
 *
 * Thruster Types Modeled:
 * - Chemical (monoprop, biprop): High thrust, low Isp
 * - Electric (ion, Hall): Low thrust, high Isp
 * - Cold gas: Simple, low performance
 *
 * Physics:
 * - Thrust = mass_flow_rate × exhaust_velocity
 * - Isp = thrust / (mass_flow_rate × g0)
 * - Torque = r × F (moment arm cross thrust vector)
 *
 * Features:
 * - Configurable thrust magnitude and direction
 * - Propellant consumption tracking
 * - Thrust build-up dynamics
 * - Minimum impulse bit (MIB)
 * - Pulse mode operation
 *
 * Usage:
 *   auto thruster = spacecraft->addComponent<Thruster>("RCS-1");
 *   thruster->setMaxThrust(10.0);        // 10 N max thrust
 *   thruster->setIsp(220.0);             // 220 s specific impulse
 *   thruster->setPosition(glm::dvec3(0.5, 0, 0));  // Position from CoM
 *   thruster->fire(0.8);                 // Fire at 80% throttle
 */
class Thruster : public Actuator
{
public:
  /**
   * Thruster type presets
   */
  enum class Type
  {
    MONOPROP,   // Hydrazine monopropellant
    BIPROP,     // Bipropellant (MMH/NTO typical)
    COLD_GAS,   // Cold gas (N2, He)
    ION,        // Ion thruster
    HALL        // Hall effect thruster
  };

  /**
   * Default constructor - monoprop RCS thruster
   */
  Thruster();

  /**
   * Named constructor
   * @param name Component name
   */
  explicit Thruster(const std::string &name);

  /**
   * Constructor with type preset
   * @param type Thruster type
   * @param name Component name
   */
  Thruster(Type type, const std::string &name = "Thruster");

  // ========== ACTUATOR INTERFACE ==========

  /**
   * Get component type name
   */
  std::string getTypeName() const override { return "Thruster"; }

  /**
   * Get force produced by thruster
   * @return Force vector in body frame (N)
   */
  glm::dvec3 getForce() const override;

  /**
   * Get torque produced by thruster (from offset mounting)
   * @return Torque vector in body frame (Nm)
   */
  glm::dvec3 getTorque() const override;

  /**
   * Update thruster state (thrust buildup, propellant consumption)
   * @param deltaTime Time step (seconds)
   */
  void update(double deltaTime) override;

  // ========== COMMAND INTERFACE ==========

  /**
   * Fire thruster at specified throttle level
   * @param throttle Throttle setting (0.0 to 1.0)
   */
  void fire(double throttle);

  /**
   * Stop firing thruster
   */
  void stop() { commandedThrottle = 0.0; firing = false; }

  /**
   * Fire a pulse of specified duration
   * @param duration Pulse duration (seconds)
   * @param throttle Throttle setting (0.0 to 1.0)
   */
  void pulse(double duration, double throttle = 1.0);

  /**
   * Check if thruster is currently firing
   * @return True if actively firing
   */
  bool isFiring() const { return firing && actualThrottle > 0.01; }

  /**
   * Get commanded throttle
   * @return Throttle (0-1)
   */
  double getCommandedThrottle() const { return commandedThrottle; }

  /**
   * Get actual throttle (after dynamics)
   * @return Actual throttle (0-1)
   */
  double getActualThrottle() const { return actualThrottle; }

  // ========== PROPELLANT INTERFACE ==========

  /**
   * Get current propellant mass
   * @return Propellant mass (kg)
   */
  double getPropellantMass() const { return propellantMass; }

  /**
   * Set propellant mass
   * @param mass Propellant mass (kg)
   */
  void setPropellantMass(double mass) { propellantMass = mass; }

  /**
   * Get mass flow rate at current throttle
   * @return Mass flow rate (kg/s)
   */
  double getMassFlowRate() const;

  /**
   * Get total propellant consumed
   * @return Total propellant used (kg)
   */
  double getPropellantConsumed() const { return propellantConsumed; }

  /**
   * Check if thruster has propellant remaining
   * @return True if propellant available
   */
  bool hasPropellant() const { return propellantMass > 0.0; }

  // ========== CONFIGURATION ==========

  /**
   * Set maximum thrust
   * @param thrust Max thrust (N)
   */
  void setMaxThrust(double thrust) { maxThrust = thrust; }

  /**
   * Get maximum thrust
   * @return Max thrust (N)
   */
  double getMaxThrust() const { return maxThrust; }

  /**
   * Set specific impulse
   * @param isp Specific impulse (seconds)
   */
  void setIsp(double isp) { specificImpulse = isp; }

  /**
   * Get specific impulse
   * @return Isp (seconds)
   */
  double getIsp() const { return specificImpulse; }

  /**
   * Set thrust direction in body frame
   * @param direction Unit vector of thrust direction
   */
  void setDirection(const glm::dvec3 &direction) { thrustDirection = glm::normalize(direction); }

  /**
   * Get thrust direction
   * @return Thrust direction unit vector
   */
  glm::dvec3 getDirection() const { return thrustDirection; }

  /**
   * Set thruster position relative to spacecraft center of mass
   * @param position Position vector in body frame (m)
   */
  void setPosition(const glm::dvec3 &position) { mountPosition = position; }

  /**
   * Get thruster position
   * @return Position in body frame (m)
   */
  glm::dvec3 getPosition() const { return mountPosition; }

  /**
   * Set thrust buildup time constant
   * @param tau Time constant (seconds)
   */
  void setTimeConstant(double tau) { timeConstant = tau; }

  /**
   * Set minimum impulse bit
   * @param mib Minimum impulse (Ns)
   */
  void setMinimumImpulseBit(double mib) { minimumImpulseBit = mib; }

  /**
   * Get current power consumption
   * @return Power (W) - significant for electric thrusters
   */
  double getPowerConsumption() const;

private:
  // ========== THRUSTER CHARACTERISTICS ==========
  double maxThrust;           // Maximum thrust (N)
  double specificImpulse;     // Specific impulse (s)
  glm::dvec3 thrustDirection; // Thrust direction in body frame (unit vector)
  glm::dvec3 mountPosition;   // Position relative to CoM (m)
  double timeConstant;        // Thrust buildup time constant (s)
  double minimumImpulseBit;   // Minimum impulse bit (Ns)
  double powerPerThrust;      // Power consumption per Newton (W/N)
  Type thrusterType;          // Thruster type

  // ========== STATE ==========
  double commandedThrottle;   // Commanded throttle (0-1)
  double actualThrottle;      // Actual throttle after dynamics (0-1)
  bool firing;                // Thruster firing state
  double propellantMass;      // Remaining propellant (kg)
  double propellantConsumed;  // Total propellant consumed (kg)
  double pulseTimeRemaining;  // Time remaining in pulse (s)

  // ========== CONSTANTS ==========
  static constexpr double g0 = 9.80665;  // Standard gravity (m/s²)
};

#endif // THRUSTER_H
