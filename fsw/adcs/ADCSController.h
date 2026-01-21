#ifndef ADCS_CONTROLLER_H
#define ADCS_CONTROLLER_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "FlightSoftwareModule.h"
#include "PIDController.h"
#include "AttitudeEstimator.h"
#include "SpacecraftEnvironment.h"
#include <memory>

// Forward declarations
class Spacecraft;
class ReactionWheel;
class GroundStation;

/**
 * ADCS Controller
 *
 * Attitude Determination and Control System (as a Flight Software Module)
 *
 * Responsibilities:
 * 1. Attitude Estimation: Reads sensors (IMU, star tracker, sun sensor, etc.)
 * 2. Target Selection: Chooses pointing target (ground station, nadir, sun, inertial, etc.)
 * 3. Control: Commands actuators (reaction wheels, magnetorquers) to achieve target attitude
 *
 * Control Modes:
 * - DETUMBLE: Reduce angular rates to near zero
 * - NADIR_POINTING: Point -Z axis toward Earth center
 * - GROUND_STATION_POINTING: Point -Z axis toward nearest ground station
 * - SUN_POINTING: Point +X axis toward sun
 * - INERTIAL_HOLD: Maintain fixed inertial attitude
 *
 * This is a simplified ADCS for demonstration. Real spacecraft ADCS include:
 * - Kalman filtering for attitude estimation
 * - Advanced control algorithms (LQR, MPC, sliding mode)
 * - Momentum management and desaturation logic
 * - Fault detection and recovery
 */
class ADCSController : public FlightSoftwareModule
{
public:
  enum class Mode
  {
    OFF,                       // ADCS disabled
    DETUMBLE,                  // Reduce angular rates
    NADIR_POINTING,            // Point -Z to Earth center
    GROUND_STATION_POINTING,   // Point -Z to nearest ground station
    SUN_POINTING,              // Point +X to Sun
    INERTIAL_HOLD              // Maintain fixed attitude
  };

  ADCSController();

  /**
   * Update ADCS controller (FlightSoftwareModule interface)
   * @param deltaTime Time step (seconds)
   * @param spacecraft Spacecraft being controlled
   * @param environment Universe environment data
   */
  void update(double deltaTime, Spacecraft &spacecraft, const SpacecraftEnvironment &environment) override;

  // Mode control
  void setMode(Mode mode) { this->mode = mode; }
  Mode getMode() const { return mode; }

  // Target attitude (for INERTIAL_HOLD mode)
  void setTargetAttitude(const glm::dquat &quat) { targetAttitude = quat; }
  glm::dquat getTargetAttitude() const { return targetAttitude; }

  // Status
  glm::dvec3 getAttitudeError() const { return attitudeError; }
  glm::dvec3 getRateError() const { return rateError; }
  std::string getCurrentTargetName() const { return currentTargetName; }

private:
  Mode mode;
  PIDController pidController;
  AttitudeEstimator attitudeEstimator;

  // Attitude estimates from MEKF
  glm::dquat estimatedAttitude;
  glm::dvec3 estimatedAngularVelocity;

  // Target attitude
  glm::dquat targetAttitude;
  glm::dvec3 targetAngularVelocity; // Usually zero

  // Error tracking
  glm::dvec3 attitudeError;  // Quaternion error converted to axis-angle
  glm::dvec3 rateError;      // Angular velocity error

  // Target info
  std::string currentTargetName;

  // PID tuning state
  bool pidTuned;  // True after PID has been tuned for this spacecraft

  /**
   * Estimate attitude from sensors
   * For now, just uses true attitude from spacecraft
   * In the future, this would use IMU, star tracker, etc. with Kalman filtering
   */
  void estimateAttitude(const Spacecraft &spacecraft);

  /**
   * Compute target quaternion based on current mode
   * @param spacecraft Spacecraft being controlled
   * @param environment Universe environment data
   * @return Target quaternion (body frame to inertial frame)
   */
  glm::dquat computeTargetQuaternion(const Spacecraft &spacecraft, const SpacecraftEnvironment &environment);

  /**
   * Find closest ground station to spacecraft
   * @param spacecraft Spacecraft position
   * @param environment Universe environment data
   * @return Pointer to closest ground station, or nullptr if none available
   */
  const GroundStation *findClosestGroundStation(const Spacecraft &spacecraft, const SpacecraftEnvironment &environment);

  /**
   * Compute quaternion to point -Z axis toward target position
   * @param spacecraftPos Spacecraft position (inertial)
   * @param targetPos Target position (inertial)
   * @return Quaternion for pointing -Z toward target
   */
  glm::dquat computePointingQuaternion(const glm::dvec3 &spacecraftPos, const glm::dvec3 &targetPos);

  /**
   * Compute attitude error from current to target quaternion
   * Converts quaternion error to axis-angle representation
   * @param current Current attitude quaternion
   * @param target Target attitude quaternion
   * @return Error as axis-angle (3D vector)
   */
  glm::dvec3 computeAttitudeError(const glm::dquat &current, const glm::dquat &target);

  /**
   * Command reaction wheels based on PID control
   * @param spacecraft Spacecraft with reaction wheels
   * @param deltaTime Time step (seconds)
   */
  void commandReactionWheels(Spacecraft &spacecraft, double deltaTime);
};

#endif // ADCS_CONTROLLER_H
