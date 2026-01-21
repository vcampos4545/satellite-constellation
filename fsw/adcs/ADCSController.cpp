#include "ADCSController.h"
#include "AttitudeEstimator.h"
#include "Spacecraft.h"
#include "ReactionWheel.h"
#include "GroundStation.h"
#include "Constants.h"
#include <glm/gtc/type_ptr.hpp>
#include <limits>
#include <cmath>

ADCSController::ADCSController()
    : FlightSoftwareModule("ADCS"),
      mode(Mode::OFF),
      estimatedAttitude(1.0, 0.0, 0.0, 0.0),
      estimatedAngularVelocity(0.0),
      targetAttitude(1.0, 0.0, 0.0, 0.0),
      targetAngularVelocity(0.0),
      attitudeError(0.0),
      rateError(0.0),
      currentTargetName("None"),
      pidTuned(false)
{
  // PID will be tuned on first update with actual spacecraft inertia
}

void ADCSController::update(double deltaTime, Spacecraft &spacecraft, const SpacecraftEnvironment &environment)
{
  if (mode == Mode::OFF)
  {
    return; // ADCS disabled
  }

  // Auto-tune PID on first update with actual spacecraft inertia
  if (!pidTuned)
  {
    glm::dvec3 inertia = spacecraft.getInertiaDiagonal();
    // Use longer settling time for larger inertia spacecraft
    // Settling time scales with sqrt(I) for fixed torque capability
    double settlingTime = 60.0; // 60 seconds for large satellites
    pidController.autoTune(inertia, settlingTime, 0.8); // 0.8 damping ratio
    pidTuned = true;
  }

  // Step 1: Estimate attitude from sensors
  estimateAttitude(spacecraft);

  // Step 2: Compute target quaternion based on mode
  targetAttitude = computeTargetQuaternion(spacecraft, environment);

  // Step 3: Compute attitude and rate errors
  attitudeError = computeAttitudeError(estimatedAttitude, targetAttitude);
  rateError = estimatedAngularVelocity - targetAngularVelocity;

  // Step 4: Command reaction wheels using PID control
  commandReactionWheels(spacecraft, deltaTime);
}

void ADCSController::estimateAttitude(const Spacecraft &spacecraft)
{
  /**
   * Attitude Estimation using TRIAD + Multiplicative EKF
   *
   * This simulates a realistic ADCS attitude determination system:
   * 1. Gyroscope provides high-rate angular velocity measurements
   * 2. Sun sensor + Earth sensor provide vector measurements for TRIAD
   * 3. MEKF fuses gyro propagation with TRIAD updates
   *
   * In this simulation, we use "true" spacecraft state with simulated noise
   * to demonstrate the filtering algorithm.
   */

  // Get true state (in real system, these would come from sensors)
  glm::dvec3 trueOmega = spacecraft.getAngularVelocity();
  glm::dquat trueAttitude = spacecraft.getAttitude();

  // Simulate gyroscope measurement with noise
  // Typical MEMS gyro: ~0.01 deg/s noise, ~0.001 deg/s bias drift
  glm::dvec3 gyroNoise(
      (rand() / (double)RAND_MAX - 0.5) * 0.0002, // ~0.01 deg/s
      (rand() / (double)RAND_MAX - 0.5) * 0.0002,
      (rand() / (double)RAND_MAX - 0.5) * 0.0002);
  glm::dvec3 gyroMeasurement = trueOmega + gyroNoise;

  // Initialize estimator on first call
  if (!attitudeEstimator.isInitialized())
  {
    attitudeEstimator.initialize(trueAttitude);
  }

  // Propagate attitude estimate using gyro (high rate - every update)
  // In real system, this would be called at gyro rate (e.g., 100 Hz)
  attitudeEstimator.propagate(gyroMeasurement, 0.1); // Assume 10 Hz ADCS rate

  // Simulate sun and nadir vector measurements for TRIAD
  // These would come from sun sensor and Earth horizon sensor
  static int updateCounter = 0;
  updateCounter++;

  // Update TRIAD every 10 cycles (~1 Hz measurement rate)
  if (updateCounter >= 10)
  {
    updateCounter = 0;

    // Sun vector in inertial frame (simplified - should come from ephemeris)
    glm::dvec3 sunDir_inertial = glm::normalize(glm::dvec3(1.0, 0.0, 0.0)); // Assume sun in +X

    // Nadir vector in inertial frame
    glm::dvec3 nadirDir_inertial = glm::normalize(-spacecraft.getPosition());

    // Transform to body frame using true attitude (simulates sensor measurements)
    glm::dmat3 R_inertial_to_body = glm::mat3_cast(glm::inverse(trueAttitude));

    // Add measurement noise (~1 degree for sun sensor, ~2 degrees for Earth sensor)
    glm::dvec3 sunNoise(
        (rand() / (double)RAND_MAX - 0.5) * 0.02,
        (rand() / (double)RAND_MAX - 0.5) * 0.02,
        (rand() / (double)RAND_MAX - 0.5) * 0.02);
    glm::dvec3 nadirNoise(
        (rand() / (double)RAND_MAX - 0.5) * 0.04,
        (rand() / (double)RAND_MAX - 0.5) * 0.04,
        (rand() / (double)RAND_MAX - 0.5) * 0.04);

    glm::dvec3 sunDir_body = glm::normalize(R_inertial_to_body * sunDir_inertial + sunNoise);
    glm::dvec3 nadirDir_body = glm::normalize(R_inertial_to_body * nadirDir_inertial + nadirNoise);

    // Update estimator with TRIAD measurement
    attitudeEstimator.update(sunDir_inertial, nadirDir_inertial,
                              sunDir_body, nadirDir_body);
  }

  // Get filtered estimates
  estimatedAttitude = attitudeEstimator.getAttitude();
  estimatedAngularVelocity = attitudeEstimator.getAngularVelocity();
}

glm::dquat ADCSController::computeTargetQuaternion(const Spacecraft &spacecraft, const SpacecraftEnvironment &environment)
{
  glm::dvec3 spacecraftPos = spacecraft.getPosition();

  switch (mode)
  {
  case Mode::DETUMBLE:
    // Keep current attitude, just reduce rates to zero
    currentTargetName = "Detumble (current attitude)";
    return estimatedAttitude;

  case Mode::NADIR_POINTING:
    // Point -Z axis toward Earth center
    currentTargetName = "Earth Nadir";
    return computePointingQuaternion(spacecraftPos, environment.earthPosition);

  case Mode::GROUND_STATION_POINTING:
  {
    // Point -Z axis toward nearest ground station
    const GroundStation *closestStation = findClosestGroundStation(spacecraft, environment);
    if (closestStation)
    {
      currentTargetName = closestStation->getName();
      return computePointingQuaternion(spacecraftPos, closestStation->getPosition());
    }
    else
    {
      // No ground stations available, fall back to nadir pointing
      currentTargetName = "Earth Nadir (no ground stations)";
      return computePointingQuaternion(spacecraftPos, environment.earthPosition);
    }
  }

  case Mode::SUN_POINTING:
    // Point +X axis toward Sun
    // For now, use simple pointing (later: rotate to point +X instead of -Z)
    currentTargetName = "Sun";
    return computePointingQuaternion(spacecraftPos, environment.sunPosition);

  case Mode::INERTIAL_HOLD:
    // Maintain fixed inertial attitude
    currentTargetName = "Inertial Hold";
    return targetAttitude;

  case Mode::OFF:
  default:
    return glm::dquat(1.0, 0.0, 0.0, 0.0); // Identity
  }
}

const GroundStation *ADCSController::findClosestGroundStation(const Spacecraft &spacecraft, const SpacecraftEnvironment &environment)
{
  if (!environment.groundStations || environment.groundStations->empty())
  {
    return nullptr;
  }

  const GroundStation *closest = nullptr;
  double minDistance = std::numeric_limits<double>::max();

  glm::dvec3 spacecraftPos = spacecraft.getPosition();

  for (const auto &station : *environment.groundStations)
  {
    glm::dvec3 stationPos = station->getPosition();
    double distance = glm::length(stationPos - spacecraftPos);

    if (distance < minDistance)
    {
      minDistance = distance;
      closest = station.get();
    }
  }

  return closest;
}

glm::dquat ADCSController::computePointingQuaternion(const glm::dvec3 &spacecraftPos, const glm::dvec3 &targetPos)
{
  /**
   * Compute quaternion to point spacecraft -Z axis toward target
   *
   * Approach:
   * 1. Compute pointing direction: from spacecraft to target
   * 2. Desired body -Z axis should align with this direction
   * 3. Use quaternion rotation to align -Z with pointing direction
   * 4. Choose X and Y axes arbitrarily (but consistently)
   */

  // Pointing direction (inertial frame)
  glm::dvec3 pointingDir = glm::normalize(targetPos - spacecraftPos);

  // Desired body -Z axis in inertial frame
  glm::dvec3 desiredZ = -pointingDir;

  // Choose arbitrary X axis perpendicular to Z
  // Use cross product with a reference vector
  glm::dvec3 referenceUp(0.0, 0.0, 1.0); // Inertial Z (Earth's rotation axis)

  // If desiredZ is nearly parallel to referenceUp, use different reference
  if (std::abs(glm::dot(desiredZ, referenceUp)) > 0.99)
  {
    referenceUp = glm::dvec3(1.0, 0.0, 0.0); // Use inertial X instead
  }

  // Compute body X axis (perpendicular to Z)
  glm::dvec3 desiredX = glm::normalize(glm::cross(referenceUp, desiredZ));

  // Compute body Y axis (completes right-handed frame)
  glm::dvec3 desiredY = glm::cross(desiredZ, desiredX);

  // Construct rotation matrix from body axes
  glm::dmat3 rotationMatrix(desiredX, desiredY, desiredZ);

  // Convert rotation matrix to quaternion
  return glm::quat_cast(rotationMatrix);
}

glm::dvec3 ADCSController::computeAttitudeError(const glm::dquat &current, const glm::dquat &target)
{
  /**
   * Compute attitude error as axis-angle representation
   *
   * Error quaternion: q_error = q_target * q_current^-1
   * Axis-angle: θ * axis = 2 * vector_part(q_error) / |vector_part(q_error)|
   *
   * For small errors, can approximate as: error ≈ 2 * vector_part(q_error)
   */

  // Compute error quaternion
  glm::dquat errorQuat = target * glm::inverse(current);

  // Normalize to ensure unit quaternion
  errorQuat = glm::normalize(errorQuat);

  // Extract axis-angle error
  // For small angles: error ≈ 2 * [x, y, z] component of quaternion
  // For large angles: need full conversion

  double w = errorQuat.w;
  glm::dvec3 vec(errorQuat.x, errorQuat.y, errorQuat.z);

  // Angle of rotation
  double angle = 2.0 * std::atan2(glm::length(vec), w);

  // Axis of rotation
  glm::dvec3 axis = glm::length(vec) > 1e-10 ? glm::normalize(vec) : glm::dvec3(0.0, 0.0, 1.0);

  // Axis-angle error vector
  return angle * axis;
}

void ADCSController::commandReactionWheels(Spacecraft &spacecraft, double deltaTime)
{
  // Compute control torque using PID controller with actual timestep
  glm::dvec3 controlTorque = pidController.computeControl(attitudeError, rateError, deltaTime);

  // Get all reaction wheels
  auto wheels = spacecraft.getComponents<ReactionWheel>();

  if (wheels.empty())
  {
    // No reaction wheels available
    return;
  }

  // Distribute torque among reaction wheels
  // For simplicity, assume we have 3 wheels aligned with body axes
  // Each wheel commands the component of torque along its axis

  for (auto *wheel : wheels)
  {
    if (!wheel->enabled)
      continue;

    // Get wheel spin axis
    glm::dvec3 axis = wheel->getSpinAxis();

    // Project control torque onto wheel axis
    double torqueComponent = glm::dot(controlTorque, axis);

    // Command the wheel
    wheel->commandTorque(torqueComponent);
  }
}
