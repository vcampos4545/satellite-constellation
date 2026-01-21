#ifndef CMG_H
#define CMG_H

#include "Component.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>

/**
 * Control Moment Gyroscope (CMG) Actuator Component
 *
 * Simulates a single-gimbal control moment gyroscope that produces torque
 * by changing the direction of a spinning flywheel. CMGs are used for:
 * - High-agility attitude control (ISS, large satellites)
 * - Rapid slewing maneuvers
 * - Precision pointing
 *
 * Physics:
 * - Angular momentum: h = I_wheel * ω_wheel (along spin axis)
 * - Output torque: τ = dh/dt = h × gimbal_rate
 * - Torque perpendicular to both spin axis and gimbal axis
 *
 * Key Characteristics:
 * - Much higher torque than reaction wheels (100x or more)
 * - Constant speed flywheel (no acceleration limits)
 * - Singularity issues (gimbal lock configurations)
 * - More complex than reaction wheels
 *
 * Single-Gimbal CMG:
 * - Spin axis: direction the flywheel spins
 * - Gimbal axis: axis the CMG rotates around
 * - Output torque axis: perpendicular to both
 *
 * Typical Values:
 * - Angular momentum: 10-1000 Nms
 * - Gimbal rate: 0.1-10 rad/s
 * - Output torque: 1-10000 Nm
 *
 * Usage:
 *   auto cmg = spacecraft->addComponent<CMG>("CMG-1");
 *   cmg->setAngularMomentum(100.0);    // 100 Nms
 *   cmg->setMaxGimbalRate(1.0);        // 1 rad/s max
 *   cmg->commandGimbalRate(0.5);       // Command 0.5 rad/s
 *   glm::dvec3 torque = cmg->getTorque();
 */
class CMG : public Actuator
{
public:
  /**
   * Default constructor
   */
  CMG();

  /**
   * Named constructor
   * @param name Component name (e.g., "CMG-1", "CMG-2")
   */
  explicit CMG(const std::string &name);

  /**
   * Full constructor
   * @param spinAxis Initial spin axis direction (body frame)
   * @param gimbalAxis Gimbal rotation axis (body frame)
   * @param angularMomentum Flywheel angular momentum magnitude (Nms)
   * @param name Component name
   */
  CMG(const glm::dvec3 &spinAxis, const glm::dvec3 &gimbalAxis,
      double angularMomentum, const std::string &name = "CMG");

  // ========== ACTUATOR INTERFACE ==========

  /**
   * Get component type name
   */
  std::string getTypeName() const override { return "CMG"; }

  /**
   * Get torque produced by CMG
   * Torque = h × gimbal_rate_vector
   * @return Torque vector in body frame (Nm)
   */
  glm::dvec3 getTorque() const override;

  /**
   * Update CMG state (gimbal dynamics)
   * @param deltaTime Time step (seconds)
   */
  void update(double deltaTime) override;

  // ========== COMMAND INTERFACE ==========

  /**
   * Command gimbal rate
   * @param rate Gimbal rate (rad/s), positive = rotation around gimbal axis
   */
  void commandGimbalRate(double rate);

  /**
   * Command gimbal angle directly
   * @param angle Gimbal angle (rad)
   */
  void commandGimbalAngle(double angle);

  /**
   * Get commanded gimbal rate
   * @return Commanded rate (rad/s)
   */
  double getCommandedGimbalRate() const { return commandedGimbalRate; }

  /**
   * Get actual gimbal rate (after dynamics)
   * @return Actual rate (rad/s)
   */
  double getActualGimbalRate() const { return actualGimbalRate; }

  /**
   * Get current gimbal angle
   * @return Gimbal angle (rad)
   */
  double getGimbalAngle() const { return gimbalAngle; }

  // ========== MOMENTUM INTERFACE ==========

  /**
   * Get angular momentum vector
   * @return Angular momentum in body frame (Nms)
   */
  glm::dvec3 getAngularMomentum() const;

  /**
   * Get angular momentum magnitude
   * @return |h| in Nms
   */
  double getAngularMomentumMagnitude() const { return angularMomentum; }

  /**
   * Get current spin axis (rotates with gimbal)
   * @return Current spin axis unit vector in body frame
   */
  glm::dvec3 getCurrentSpinAxis() const;

  /**
   * Get output torque axis (perpendicular to spin and gimbal axes)
   * @return Output torque axis unit vector
   */
  glm::dvec3 getOutputAxis() const;

  // ========== CONFIGURATION ==========

  /**
   * Set gimbal axis (rotation axis of the CMG)
   * @param axis Unit vector in body frame
   */
  void setGimbalAxis(const glm::dvec3 &axis) { gimbalAxis = glm::normalize(axis); }

  /**
   * Get gimbal axis
   * @return Gimbal axis unit vector
   */
  glm::dvec3 getGimbalAxis() const { return gimbalAxis; }

  /**
   * Set initial spin axis (at gimbal angle = 0)
   * @param axis Unit vector in body frame
   */
  void setInitialSpinAxis(const glm::dvec3 &axis) { initialSpinAxis = glm::normalize(axis); }

  /**
   * Set angular momentum magnitude
   * @param h Angular momentum (Nms)
   */
  void setAngularMomentum(double h) { angularMomentum = h; }

  /**
   * Set maximum gimbal rate
   * @param maxRate Maximum gimbal rate (rad/s)
   */
  void setMaxGimbalRate(double maxRate) { maxGimbalRate = maxRate; }

  /**
   * Get maximum gimbal rate
   * @return Max rate (rad/s)
   */
  double getMaxGimbalRate() const { return maxGimbalRate; }

  /**
   * Set gimbal acceleration limit
   * @param maxAccel Maximum gimbal acceleration (rad/s²)
   */
  void setMaxGimbalAccel(double maxAccel) { maxGimbalAccel = maxAccel; }

  /**
   * Set gimbal angle limits
   * @param minAngle Minimum angle (rad)
   * @param maxAngle Maximum angle (rad)
   */
  void setGimbalLimits(double minAngle, double maxAngle);

  /**
   * Check if CMG is near singularity
   * Singularity occurs when output axes of multiple CMGs become coplanar
   * @return True if approaching singularity
   */
  bool isNearSingularity() const;

  /**
   * Get power consumption
   * @return Power (W)
   */
  double getPowerConsumption() const;

private:
  // ========== CONFIGURATION ==========
  glm::dvec3 initialSpinAxis;   // Spin axis at gimbal angle = 0 (body frame)
  glm::dvec3 gimbalAxis;        // Gimbal rotation axis (body frame)
  double angularMomentum;       // Flywheel angular momentum magnitude (Nms)
  double maxGimbalRate;         // Maximum gimbal rate (rad/s)
  double maxGimbalAccel;        // Maximum gimbal acceleration (rad/s²)
  double minGimbalAngle;        // Minimum gimbal angle (rad)
  double maxGimbalAngle;        // Maximum gimbal angle (rad)
  double basePower;             // Base power to spin flywheel (W)
  double gimbalPowerCoeff;      // Power per gimbal rate (W/(rad/s))

  // ========== STATE ==========
  double gimbalAngle;           // Current gimbal angle (rad)
  double commandedGimbalRate;   // Commanded gimbal rate (rad/s)
  double actualGimbalRate;      // Actual gimbal rate after dynamics (rad/s)
  double commandedGimbalAngle;  // For angle command mode (rad)
  bool angleCommandMode;        // True if commanding angle instead of rate
};

#endif // CMG_H
