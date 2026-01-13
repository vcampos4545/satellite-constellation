#ifndef GROUND_TRACKING_ADCS_H
#define GROUND_TRACKING_ADCS_H

#include "FlightSoftwareTask.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <string>

/**
 * Ground Tracking ADCS Flight Software
 *
 * Implements a simple ADCS algorithm that:
 * 1. Reads IMU data to estimate attitude
 * 2. Finds the nearest ground station
 * 3. Commands reaction wheels using PID control to point at that station
 *
 * This demonstrates basic Earth-observation satellite pointing capabilities.
 * Useful for communication satellites, remote sensing, etc.
 */
class GroundTrackingADCS : public FlightSoftwareTask
{
public:
  GroundTrackingADCS();
  virtual ~GroundTrackingADCS() = default;

  void execute(Satellite *satellite, double deltaTime) override;
  std::string getName() const override { return "GroundTrackingADCS"; }
  void reset() override;

  /**
   * Get current target ground station position (for visualization)
   * @return Target position in ECEF, or null vector if no target
   */
  glm::dvec3 getTargetPosition() const { return currentTargetPosition; }

  /**
   * Check if currently tracking a target
   */
  bool hasTarget() const { return currentTargetStation >= 0; }

private:
  /**
   * Ground station database entry
   */
  struct GroundStationInfo
  {
    std::string name;
    glm::dvec3 position; // ECEF position (meters)
    double latitude;     // degrees
    double longitude;    // degrees
  };

  /**
   * Estimate satellite attitude from IMU
   *
   * For now uses shortcuts (truth-based + IMU noise) until sun sensor
   * and star tracker are implemented.
   *
   * @param satellite Satellite providing IMU access
   * @param deltaTime Time step
   * @return Estimated attitude quaternion
   */
  glm::dquat estimateAttitude(Satellite *satellite, double deltaTime);

  /**
   * Find the nearest ground station in visibility
   *
   * @param satellite Satellite providing position
   * @return Index of nearest ground station, or -1 if none visible
   */
  int findNearestGroundStation(Satellite *satellite);

  /**
   * Compute target quaternion to point at ground station
   *
   * Computes attitude that aligns satellite body +Z axis with
   * the direction to the ground station.
   *
   * @param satellite Satellite providing position/velocity
   * @param groundStationPos Position of target ground station
   * @return Target attitude quaternion
   */
  glm::dquat computePointingQuaternion(Satellite *satellite, const glm::dvec3 &groundStationPos);

  /**
   * Compute PID control torque to track target attitude
   *
   * @param satellite Satellite providing state
   * @param currentAttitude Current estimated attitude
   * @param targetAttitude Desired pointing attitude
   * @param deltaTime Time step
   * @return Control torque (N·m)
   */
  glm::dvec3 computePIDTorque(Satellite *satellite,
                              const glm::dquat &currentAttitude,
                              const glm::dquat &targetAttitude,
                              double deltaTime);

  /**
   * Compute attitude error from quaternion difference
   *
   * @param current Current attitude
   * @param target Target attitude
   * @return Error vector (radians)
   */
  glm::dvec3 computeAttitudeError(const glm::dquat &current, const glm::dquat &target);

  /**
   * Convert lat/lon to ECEF position
   *
   * @param latitude Latitude in degrees
   * @param longitude Longitude in degrees
   * @param earthRotation Current Earth rotation angle (radians)
   * @return ECEF position (meters)
   */
  glm::dvec3 latLonToECEF(double latitude, double longitude, double earthRotation);

  /**
   * Auto-tune PID gains based on satellite properties
   *
   * Uses Ziegler-Nichols-like method adapted for spacecraft attitude control:
   * - Computes natural frequency from inertia
   * - Sets gains for critically damped or slightly underdamped response
   * - Ensures stability margins for robust performance
   *
   * @param satellite Satellite to tune gains for
   */
  void autoTunePIDGains(Satellite *satellite);

  // Ground station database (populated in constructor)
  std::vector<GroundStationInfo> groundStations;

  // PID controller state
  glm::dvec3 integralError;
  glm::dvec3 previousError;

  // PID gains (tuned for typical cubesat)
  double Kp = 0.1;  // Proportional gain
  double Ki = 0.01; // Integral gain
  double Kd = 0.5;  // Derivative gain

  // Current tracking target
  int currentTargetStation = -1;
  glm::dvec3 currentTargetPosition = glm::dvec3(0.0); // Position of current target (for visualization)

  // Flag to track if gains have been auto-tuned
  bool gainsAutoTuned = false;
};

#endif // GROUND_TRACKING_ADCS_H
