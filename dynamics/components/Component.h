#ifndef COMPONENT_H
#define COMPONENT_H

#include <string>
#include <glm/glm.hpp>

/**
 * Component Base Class
 *
 * Base class for all spacecraft components (actuators, sensors, power systems, etc.)
 * Provides common interface for component management and updates.
 *
 * Component Categories:
 * - Sensors: IMU, GPS, Star Tracker, etc.
 * - Actuators: Reaction Wheels, Thrusters, Magnetorquers, etc.
 * - Power: Solar Panels, Batteries
 * - Communication: Antennas, Transponders
 *
 * Design Pattern: Component-based architecture
 * Each component is independent with its own state and behavior.
 */
class Component
{
public:
  virtual ~Component() = default;

  /**
   * Update component state
   * Called each simulation frame
   * @param deltaTime Time step in seconds
   */
  virtual void update(double deltaTime) {}

  /**
   * Get component type name for identification
   * @return Type name (e.g., "SolarPanel", "ReactionWheel")
   */
  virtual std::string getTypeName() const = 0;

  /**
   * Reset component to initial state
   */
  virtual void reset() {}

  // Component identification
  std::string name;     // Human-readable name (e.g., "RW+X", "Panel1")
  bool enabled = true;  // Whether component is active

protected:
  Component() = default;
  Component(const std::string &name) : name(name) {}
};

/**
 * Actuator Component
 *
 * Base class for components that produce forces and/or torques.
 * Forces and torques are in body frame coordinates.
 *
 * Examples: Reaction Wheels, Thrusters, Magnetorquers
 */
class Actuator : public Component
{
public:
  /**
   * Get force produced by actuator in body frame
   * @return Force vector in Newtons
   */
  virtual glm::dvec3 getForce() const { return glm::dvec3(0.0); }

  /**
   * Get torque produced by actuator in body frame
   * @return Torque vector in Newton-meters
   */
  virtual glm::dvec3 getTorque() const { return glm::dvec3(0.0); }

protected:
  Actuator() = default;
  Actuator(const std::string &name) : Component(name) {}
};

/**
 * Sensor Component
 *
 * Base class for measurement devices.
 * Sensors observe spacecraft state and environment.
 *
 * Examples: IMU, GPS, Sun Sensor, Star Tracker
 */
class Sensor : public Component
{
public:
  /**
   * Take measurement from spacecraft state
   * Subclasses implement specific sensing physics and noise models
   */
  virtual void measure() = 0;

protected:
  Sensor() = default;
  Sensor(const std::string &name) : Component(name) {}
};

/**
 * Power Component
 *
 * Base class for power generation and storage systems.
 *
 * Examples: Solar Panels, Batteries, Fuel Cells
 */
class PowerComponent : public Component
{
public:
  /**
   * Get current power generation rate
   * @return Power in Watts (positive = generating)
   */
  virtual double getPowerGeneration() const { return 0.0; }

  /**
   * Get current power consumption rate
   * @return Power in Watts (positive = consuming)
   */
  virtual double getPowerConsumption() const { return 0.0; }

protected:
  PowerComponent() = default;
  PowerComponent(const std::string &name) : Component(name) {}
};

#endif // COMPONENT_H
