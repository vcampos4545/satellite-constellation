#ifndef SPACECRAFT_H
#define SPACECRAFT_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>
#include <vector>
#include <unordered_map>
#include <string>
#include "Component.h"
#include "Orbit.h"

// Forward declarations
class FlightSoftwareTask;

// Spacecraft types for rendering and identification
enum class SpacecraftType
{
  DEFAULT,    // Generic spacecraft with simple geometry
  CUBESAT_1U, // 1U CubeSat (10cm cube)
  CUBESAT_2U, // 2U CubeSat (10x10x20cm)
  STARLINK    // SpaceX Starlink satellite
};

// Attitude control modes
enum class AttitudeControlMode
{
  NONE,              // No active control (tumbling)
  DETUMBLE,          // Reduce angular velocity to zero using magnetorquers
  NADIR_POINTING,    // Point one axis toward Earth center
  SUN_POINTING,      // Point one axis toward Sun
  VELOCITY_POINTING, // Point one axis along velocity vector
  INERTIAL_HOLD,     // Maintain fixed orientation in inertial space
  TARGET_TRACKING    // Point toward a specific target
};

// Control algorithm types for reaction wheels
enum class ControlAlgorithm
{
  PID, // PID controller (default)
  LQR, // Linear Quadratic Regulator
  MPC  // Model Predictive Control
};

/**
 * Spacecraft Class
 *
 * Generic spacecraft with component-based architecture.
 * Manages core physics state (position, velocity, attitude) and components
 * (actuators, sensors, power systems, etc.)
 *
 * Design Philosophy:
 * - Spacecraft owns physics state (position, velocity, attitude, mass, inertia)
 * - Components provide specific functionality (sensors, actuators, power)
 * - Components affect spacecraft through force/torque accumulation
 * - Clean separation: Spacecraft = physics, Components = hardware
 */
class Spacecraft
{
public:
  Spacecraft();
  Spacecraft(const glm::dvec3 &position, const glm::dvec3 &velocity);
  Spacecraft(const Orbit &orbit,
             const glm::dvec3 &initPos,
             const glm::dvec3 &initVel,
             const std::string &name = "");
  ~Spacecraft();

  // ========== PHYSICS STATE GETTERS ==========
  glm::dvec3 getPosition() const { return position; }
  glm::dvec3 getVelocity() const { return velocity; }
  glm::dquat getAttitude() const { return quaternion; }
  glm::dvec3 getAngularVelocity() const { return angularVelocity; }
  glm::dmat3 getInertiaMatrix() const { return inertiaMatrix; }
  double getMass() const { return mass; }

  // Physical properties getters
  double getDragCoefficient() const { return dragCoefficient; }
  double getCrossSectionalArea() const { return crossSectionalArea; }
  double getReflectivity() const { return reflectivity; }

  // Identification getters
  std::string getName() const { return name; }
  std::string getModelName() const { return modelName; }

  // ========== PHYSICS STATE SETTERS ==========
  void setPosition(const glm::dvec3 &pos) { position = pos; }
  void setVelocity(const glm::dvec3 &vel) { velocity = vel; }
  void setAttitude(const glm::dquat &quat) { quaternion = glm::normalize(quat); }
  void setAngularVelocity(const glm::dvec3 &omega) { angularVelocity = omega; }
  void setInertiaMatrix(const glm::dmat3 &inertia) { inertiaMatrix = inertia; }
  void setMass(double m) { mass = m; }

  // Physical properties setters
  void setDragCoefficient(double cd) { dragCoefficient = cd; }
  void setCrossSectionalArea(double area) { crossSectionalArea = area; }
  void setReflectivity(double cr) { reflectivity = cr; }

  // ========== COMPONENT MANAGEMENT ==========

  /**
   * Add component to spacecraft
   * @param name Unique name for component (e.g., "RW+X", "Panel1")
   * @param args Constructor arguments for component type T
   * @return Pointer to created component
   */
  template <typename T, typename... Args>
  T *addComponent(const std::string &name, Args &&...args)
  {
    auto comp = std::make_unique<T>(std::forward<Args>(args)...);
    comp->name = name;
    T *ptr = comp.get();

    components.push_back(std::move(comp));
    componentLookup[name] = ptr;

    return ptr;
  }

  /**
   * Get component by name
   * @param name Component name
   * @return Pointer to component or nullptr if not found
   */
  Component *getComponent(const std::string &name);

  /**
   * Get component by name with type checking
   * @param name Component name
   * @return Typed pointer to component or nullptr if not found or wrong type
   */
  template <typename T>
  T *getComponent(const std::string &name)
  {
    auto it = componentLookup.find(name);
    if (it != componentLookup.end())
    {
      return dynamic_cast<T *>(it->second);
    }
    return nullptr;
  }

  /**
   * Get all components of a specific type
   * @return Vector of pointers to components of type T
   */
  template <typename T>
  std::vector<T *> getComponents()
  {
    std::vector<T *> result;
    for (auto &comp : components)
    {
      T *typed = dynamic_cast<T *>(comp.get());
      if (typed)
      {
        result.push_back(typed);
      }
    }
    return result;
  }

  /**
   * Remove component by name
   * @param name Component name
   * @return True if component was found and removed
   */
  bool removeComponent(const std::string &name);

  /**
   * Get number of components
   */
  size_t getComponentCount() const { return components.size(); }

  // ========== PHYSICS UPDATE ==========

  /**
   * Update spacecraft physics and all components
   * @param deltaTime Time step in seconds
   * @param earthCenter Position of Earth center (meters)
   * @param earthMass Mass of Earth (kg)
   * @param sunPosition Position of Sun (meters)
   * @param moonPosition Position of Moon (meters)
   */
  void update(double deltaTime,
              const glm::dvec3 &earthCenter,
              double earthMass,
              const glm::dvec3 &sunPosition,
              const glm::dvec3 &moonPosition);

  /**
   * Calculate total acceleration from all forces
   * Includes gravity (point mass + zonal harmonics), third-body perturbations,
   * atmospheric drag, and solar radiation pressure
   *
   * @param pos Position (meters)
   * @param vel Velocity (m/s)
   * @param earthCenter Position of Earth center (meters)
   * @param earthMass Mass of Earth (kg)
   * @param sunPos Position of Sun (meters)
   * @param moonPos Position of Moon (meters)
   * @return Total acceleration (m/s²)
   */
  glm::dvec3 calculateAcceleration(const glm::dvec3 &pos,
                                   const glm::dvec3 &vel,
                                   const glm::dvec3 &earthCenter,
                                   double earthMass,
                                   const glm::dvec3 &sunPos,
                                   const glm::dvec3 &moonPos) const;

  /**
   * Aggregate forces from all actuator components
   * @return Total force in body frame (Newtons)
   */
  glm::dvec3 aggregateForces() const;

  /**
   * Aggregate torques from all actuator components
   * @return Total torque in body frame (Newton-meters)
   */
  glm::dvec3 aggregateTorques() const;

  // ========== UTILITY ==========

  /**
   * Get body axes in inertial frame
   */
  glm::dvec3 getBodyXAxis() const { return quaternion * glm::dvec3(1.0, 0.0, 0.0); }
  glm::dvec3 getBodyYAxis() const { return quaternion * glm::dvec3(0.0, 1.0, 0.0); }
  glm::dvec3 getBodyZAxis() const { return quaternion * glm::dvec3(0.0, 0.0, 1.0); }

private:
  // ========== PHYSICS STATE ==========
  glm::dvec3 position;        // Position in inertial frame (meters)
  glm::dvec3 velocity;        // Velocity in inertial frame (m/s)
  glm::dquat quaternion;      // Attitude quaternion (body to inertial)
  glm::dvec3 angularVelocity; // Angular velocity in body frame (rad/s)

  // ========== PHYSICAL PROPERTIES ==========
  double mass;               // Total mass (kg)
  glm::dmat3 inertiaMatrix;  // Moment of inertia matrix (kg·m²)
  double dragCoefficient;    // Drag coefficient (dimensionless, ~2.2 for satellites)
  double crossSectionalArea; // Cross-sectional area (m²)
  double reflectivity;       // Reflectivity coefficient (1.0 = absorbing, 2.0 = perfect mirror)

  // ========== IDENTIFICATION ==========
  std::string name;      // Spacecraft name/identifier
  std::string modelName; // Model name for rendering

  // ========== COMPONENT SYSTEM ==========
  std::vector<std::unique_ptr<Component>> components;           // Owned components
  std::unordered_map<std::string, Component *> componentLookup; // Fast name lookup
};

#endif // SPACECRAFT_H
