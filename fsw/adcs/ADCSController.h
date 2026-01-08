#ifndef ADCS_CONTROLLER_H
#define ADCS_CONTROLLER_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// Forward declaration
class Satellite;
enum class AttitudeControlMode;
enum class ControlAlgorithm;

/**
 * Attitude Determination and Control System (ADCS) Controller
 *
 * Encapsulates all attitude control logic for satellites:
 * - Attitude determination from sensors
 * - Target attitude computation based on control mode
 * - Control torque computation (PID, LQR, MPC)
 * - Actuator commanding (reaction wheels, magnetorquers, CMGs)
 *
 * This separates flight software logic from satellite hardware.
 */
class ADCSController
{
public:
  ADCSController();

  /**
   * Execute ADCS control loop
   *
   * @param satellite Pointer to satellite hardware
   * @param deltaTime Time step (seconds)
   * @param earthCenter Earth position for nadir pointing
   * @param sunPosition Sun position for sun pointing
   */
  void executeControlLoop(Satellite *satellite, double deltaTime,
                          const glm::dvec3 &earthCenter, const glm::dvec3 &sunPosition);

  /**
   * Determine current attitude from sensors
   *
   * Currently uses IMU + truth. Future: star tracker, magnetometer, sun sensors.
   *
   * @param satellite Satellite providing sensor access
   * @param earthCenter Earth position for reference
   * @param sunPosition Sun position for reference
   * @return Estimated attitude quaternion
   */
  glm::dquat determineAttitude(Satellite *satellite,
                               const glm::dvec3 &earthCenter,
                               const glm::dvec3 &sunPosition);

  /**
   * Compute target attitude based on control mode
   *
   * @param satellite Satellite for position/velocity access
   * @param earthCenter Earth position
   * @param sunPosition Sun position
   * @return Target attitude quaternion
   */
  glm::dquat computeTargetAttitude(Satellite *satellite,
                                   const glm::dvec3 &earthCenter,
                                   const glm::dvec3 &sunPosition);

  /**
   * Compute attitude error vector
   *
   * Converts quaternion difference to 3D error vector for control.
   *
   * @param currentAttitude Current attitude quaternion
   * @param targetAttitude Desired attitude quaternion
   * @return Error vector (radians in each axis)
   */
  glm::dvec3 computeAttitudeError(const glm::dquat &currentAttitude,
                                  const glm::dquat &targetAttitude);

  /**
   * Compute control torque to null attitude error
   *
   * Uses selected control algorithm (PID/LQR/MPC).
   *
   * @param satellite Satellite for state access
   * @param attitudeError Error vector from computeAttitudeError
   * @param deltaTime Time step for integral/derivative terms
   * @return Desired control torque (N·m)
   */
  glm::dvec3 computeControlTorque(Satellite *satellite,
                                  const glm::dvec3 &attitudeError,
                                  double deltaTime);

  /**
   * PID control law
   */
  glm::dvec3 computeControlTorquePID(Satellite *satellite,
                                     const glm::dvec3 &attitudeError,
                                     double deltaTime);

  /**
   * Linear Quadratic Regulator (LQR) control law
   */
  glm::dvec3 computeControlTorqueLQR(Satellite *satellite,
                                     const glm::dvec3 &attitudeError,
                                     double deltaTime);

  /**
   * Model Predictive Control (MPC) control law
   */
  glm::dvec3 computeControlTorqueMPC(Satellite *satellite,
                                     const glm::dvec3 &attitudeError,
                                     double deltaTime);

  /**
   * Command actuators to apply desired torque
   *
   * @param satellite Satellite providing actuator access
   * @param desiredTorque Desired torque from control law
   * @param deltaTime Time step
   */
  void commandActuators(Satellite *satellite,
                        const glm::dvec3 &desiredTorque,
                        double deltaTime);

  // PID state
  glm::dvec3 integralError;
  glm::dvec3 previousError;
};

#endif // ADCS_CONTROLLER_H
