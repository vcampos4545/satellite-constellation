#include "CMG.h"
#include <cmath>
#include <algorithm>
#include <glm/gtc/quaternion.hpp>

CMG::CMG()
    : Actuator(),
      initialSpinAxis(1.0, 0.0, 0.0),  // Spin axis along X at gimbal=0
      gimbalAxis(0.0, 0.0, 1.0),       // Gimbal axis along Z
      angularMomentum(100.0),          // 100 Nms typical
      maxGimbalRate(1.0),              // 1 rad/s max
      maxGimbalAccel(5.0),             // 5 rad/s² acceleration limit
      minGimbalAngle(-M_PI),           // ±180 degrees
      maxGimbalAngle(M_PI),
      basePower(50.0),                 // 50 W base power
      gimbalPowerCoeff(10.0),          // 10 W per rad/s gimbal rate
      gimbalAngle(0.0),
      commandedGimbalRate(0.0),
      actualGimbalRate(0.0),
      commandedGimbalAngle(0.0),
      angleCommandMode(false)
{
}

CMG::CMG(const std::string &name)
    : Actuator(name),
      initialSpinAxis(1.0, 0.0, 0.0),
      gimbalAxis(0.0, 0.0, 1.0),
      angularMomentum(100.0),
      maxGimbalRate(1.0),
      maxGimbalAccel(5.0),
      minGimbalAngle(-M_PI),
      maxGimbalAngle(M_PI),
      basePower(50.0),
      gimbalPowerCoeff(10.0),
      gimbalAngle(0.0),
      commandedGimbalRate(0.0),
      actualGimbalRate(0.0),
      commandedGimbalAngle(0.0),
      angleCommandMode(false)
{
}

CMG::CMG(const glm::dvec3 &spinAxis, const glm::dvec3 &gimbalAxisIn,
         double h, const std::string &name)
    : Actuator(name),
      initialSpinAxis(glm::normalize(spinAxis)),
      gimbalAxis(glm::normalize(gimbalAxisIn)),
      angularMomentum(h),
      maxGimbalRate(1.0),
      maxGimbalAccel(5.0),
      minGimbalAngle(-M_PI),
      maxGimbalAngle(M_PI),
      basePower(50.0),
      gimbalPowerCoeff(10.0),
      gimbalAngle(0.0),
      commandedGimbalRate(0.0),
      actualGimbalRate(0.0),
      commandedGimbalAngle(0.0),
      angleCommandMode(false)
{
}

void CMG::update(double deltaTime)
{
  if (!enabled)
  {
    actualGimbalRate = 0.0;
    return;
  }

  double targetRate;

  if (angleCommandMode)
  {
    // Position control mode - compute rate to reach commanded angle
    double angleError = commandedGimbalAngle - gimbalAngle;

    // Simple proportional control with rate limiting
    double kp = 2.0;  // Proportional gain
    targetRate = kp * angleError;
  }
  else
  {
    // Rate command mode
    targetRate = commandedGimbalRate;
  }

  // Clamp target rate to maximum
  targetRate = std::clamp(targetRate, -maxGimbalRate, maxGimbalRate);

  // Apply acceleration limit
  double rateError = targetRate - actualGimbalRate;
  double maxRateChange = maxGimbalAccel * deltaTime;

  if (std::abs(rateError) > maxRateChange)
  {
    actualGimbalRate += std::copysign(maxRateChange, rateError);
  }
  else
  {
    actualGimbalRate = targetRate;
  }

  // Integrate gimbal angle
  gimbalAngle += actualGimbalRate * deltaTime;

  // Apply gimbal angle limits
  if (gimbalAngle < minGimbalAngle)
  {
    gimbalAngle = minGimbalAngle;
    actualGimbalRate = 0.0;  // Hit limit
  }
  else if (gimbalAngle > maxGimbalAngle)
  {
    gimbalAngle = maxGimbalAngle;
    actualGimbalRate = 0.0;  // Hit limit
  }
}

glm::dvec3 CMG::getTorque() const
{
  if (!enabled)
  {
    return glm::dvec3(0.0);
  }

  /**
   * CMG torque physics:
   *
   * The angular momentum vector h rotates with the gimbal.
   * Output torque is the rate of change of h:
   *
   * τ = dh/dt = ω_gimbal × h
   *
   * where ω_gimbal = gimbalRate * gimbalAxis
   *
   * This simplifies to:
   * τ = h × (gimbalRate * gimbalAxis)
   * τ = angularMomentum * gimbalRate * (spinAxis × gimbalAxis)
   */

  glm::dvec3 currentSpin = getCurrentSpinAxis();
  glm::dvec3 gimbalRateVector = actualGimbalRate * gimbalAxis;

  // τ = h × ω_gimbal = (h * spinAxis) × (gimbalRate * gimbalAxis)
  glm::dvec3 hVector = angularMomentum * currentSpin;
  glm::dvec3 torque = glm::cross(gimbalRateVector, hVector);

  return torque;
}

void CMG::commandGimbalRate(double rate)
{
  commandedGimbalRate = std::clamp(rate, -maxGimbalRate, maxGimbalRate);
  angleCommandMode = false;
}

void CMG::commandGimbalAngle(double angle)
{
  commandedGimbalAngle = std::clamp(angle, minGimbalAngle, maxGimbalAngle);
  angleCommandMode = true;
}

glm::dvec3 CMG::getAngularMomentum() const
{
  return angularMomentum * getCurrentSpinAxis();
}

glm::dvec3 CMG::getCurrentSpinAxis() const
{
  /**
   * The spin axis rotates around the gimbal axis by the gimbal angle.
   * Use Rodrigues' rotation formula or quaternion rotation.
   */

  // Rotate initial spin axis around gimbal axis by gimbal angle
  glm::dquat rotation = glm::angleAxis(gimbalAngle, gimbalAxis);
  return rotation * initialSpinAxis;
}

glm::dvec3 CMG::getOutputAxis() const
{
  /**
   * Output torque axis is perpendicular to both spin axis and gimbal axis.
   * This is the direction torque will be applied when gimbaling.
   */
  glm::dvec3 currentSpin = getCurrentSpinAxis();
  return glm::normalize(glm::cross(currentSpin, gimbalAxis));
}

void CMG::setGimbalLimits(double minAngle, double maxAngle)
{
  minGimbalAngle = minAngle;
  maxGimbalAngle = maxAngle;

  // Clamp current angle to new limits
  gimbalAngle = std::clamp(gimbalAngle, minGimbalAngle, maxGimbalAngle);
}

bool CMG::isNearSingularity() const
{
  /**
   * For a single CMG, singularity is when the output axis aligns
   * with (or becomes perpendicular to) the desired torque direction.
   *
   * For CMG arrays, singularity occurs when the output axes of all
   * CMGs become coplanar (they can't produce torque in some direction).
   *
   * This simple check flags when the CMG is near its gimbal limits,
   * which reduces its effectiveness.
   */

  double margin = 0.1;  // ~6 degrees from limit
  return (gimbalAngle < minGimbalAngle + margin) ||
         (gimbalAngle > maxGimbalAngle - margin);
}

double CMG::getPowerConsumption() const
{
  if (!enabled)
  {
    return 0.0;
  }

  // Base power to keep flywheel spinning + gimbal motor power
  return basePower + gimbalPowerCoeff * std::abs(actualGimbalRate);
}
