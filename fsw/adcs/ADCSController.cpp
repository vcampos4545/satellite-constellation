#include "ADCSController.h"
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
      currentTargetName("None")
{
  // Auto-tune PID controller for typical small satellite
  // Assume moment of inertia ~1 kg·m² per axis
  glm::dvec3 inertia(1.0, 1.0, 1.0);
  pidController.autoTune(inertia, 20.0, 0.9); // 20s settling time, 0.9 damping
}

void ADCSController::update(double deltaTime, Spacecraft &spacecraft, const SpacecraftEnvironment &environment)
{
  if (mode == Mode::OFF)
  {
    return; // ADCS disabled
  }

  // Step 1: Estimate attitude from sensors
  estimateAttitude(spacecraft);

  // Step 2: Compute target quaternion based on mode
  targetAttitude = computeTargetQuaternion(spacecraft, environment);

  // Step 3: Compute attitude and rate errors
  attitudeError = computeAttitudeError(estimatedAttitude, targetAttitude);
  rateError = estimatedAngularVelocity - targetAngularVelocity;

  // Step 4: Command reaction wheels using PID control
  commandReactionWheels(spacecraft);
}

void ADCSController::estimateAttitude(const Spacecraft &spacecraft)
{
  // For now, just use true attitude (perfect knowledge)
  // In a real system, this would fuse data from:
  // - IMU (gyroscopes for angular velocity, accelerometers for gravity vector)
  // - Star tracker (high-precision attitude)
  // - Sun sensor (coarse attitude)
  // - Magnetometer (magnetic field vector)
  // Using an Extended Kalman Filter or similar estimator

  estimatedAttitude = spacecraft.getAttitude();
  estimatedAngularVelocity = spacecraft.getAngularVelocity();
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

void ADCSController::commandReactionWheels(Spacecraft &spacecraft)
{
  // Compute control torque using PID controller
  glm::dvec3 controlTorque = pidController.computeControl(attitudeError, rateError, 0.1); // Assume 0.1s update rate

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
