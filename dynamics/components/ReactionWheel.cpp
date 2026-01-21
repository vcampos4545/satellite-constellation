#include "ReactionWheel.h"
#include <cmath>

ReactionWheel::ReactionWheel(double maxTorque,
                             double maxMomentum,
                             const glm::dvec3 &spinAxis)
    : Actuator(),
      maxTorque(maxTorque),
      maxMomentum(maxMomentum),
      spinAxis(glm::normalize(spinAxis)),
      currentMomentum(0.0),
      commandedTorque(0.0),
      actualTorque(0.0),
      wheelSpeed(0.0),
      powerConsumption(0.0),
      idlePower(2.0),           // 2W idle power (typical)
      torquePowerCoeff(10.0)    // 10W per N·m (typical efficiency)
{
}

ReactionWheel::ReactionWheel(const std::string &name,
                             double maxTorque,
                             double maxMomentum,
                             const glm::dvec3 &spinAxis)
    : Actuator(name),
      maxTorque(maxTorque),
      maxMomentum(maxMomentum),
      spinAxis(glm::normalize(spinAxis)),
      currentMomentum(0.0),
      commandedTorque(0.0),
      actualTorque(0.0),
      wheelSpeed(0.0),
      powerConsumption(0.0),
      idlePower(2.0),           // 2W idle power (typical)
      torquePowerCoeff(10.0)    // 10W per N·m (typical efficiency)
{
}

void ReactionWheel::update(double deltaTime)
{
  // Apply limits based on current state
  applyLimits(deltaTime);

  // Integrate momentum: dH/dt = τ
  currentMomentum += actualTorque * deltaTime;

  // Clamp momentum to physical limits
  currentMomentum = glm::clamp(currentMomentum, -maxMomentum, maxMomentum);

  // Estimate wheel speed (assuming moment of inertia ~0.01 kg·m²)
  // H = I_wheel * ω  =>  ω = H / I_wheel
  const double wheelInertia = 0.01; // kg·m² (typical small wheel)
  wheelSpeed = currentMomentum / wheelInertia;

  // Update power consumption
  updatePower();
}

void ReactionWheel::reset()
{
  currentMomentum = 0.0;
  commandedTorque = 0.0;
  actualTorque = 0.0;
  wheelSpeed = 0.0;
  powerConsumption = idlePower;
}

glm::dvec3 ReactionWheel::getTorque() const
{
  // Return torque ON THE SPACECRAFT (not on the wheel)
  // By Newton's 3rd law: when wheel accelerates, spacecraft experiences opposite torque
  // If actualTorque > 0, wheel speeds up, spacecraft rotates in opposite direction
  return enabled ? (spinAxis * (-actualTorque)) : glm::dvec3(0.0);
}

void ReactionWheel::commandTorque(const glm::dvec3 &torque)
{
  // Project torque onto spin axis
  // Only the component along spin axis can be produced
  commandedTorque = glm::dot(torque, spinAxis);
}

void ReactionWheel::commandTorque(double torque)
{
  commandedTorque = torque;
}

double ReactionWheel::getSaturation() const
{
  return std::abs(currentMomentum) / maxMomentum;
}

bool ReactionWheel::isSaturated() const
{
  // Consider saturated if >95% of max momentum
  return getSaturation() > 0.95;
}

void ReactionWheel::desaturate(double externalTorque, double deltaTime)
{
  // External torque (e.g., from magnetorquers) removes momentum
  // Positive external torque reduces positive momentum
  currentMomentum -= externalTorque * deltaTime;

  // Clamp to valid range
  currentMomentum = glm::clamp(currentMomentum, -maxMomentum, maxMomentum);
}

void ReactionWheel::applyLimits(double deltaTime)
{
  /**
   * Apply physical constraints:
   * 1. Torque limit: |τ| ≤ τ_max
   * 2. Momentum limit: Can't command torque that would exceed H_max
   * 3. Back-off near saturation: Reduce available torque
   */

  // Start with commanded torque
  actualTorque = commandedTorque;

  // Limit 1: Maximum torque capability
  actualTorque = glm::clamp(actualTorque, -maxTorque, maxTorque);

  // Limit 2: Momentum constraint
  // Predict momentum after this timestep
  double predictedMomentum = currentMomentum + actualTorque * deltaTime;

  // If would exceed momentum limit, reduce torque
  if (predictedMomentum > maxMomentum)
  {
    // Calculate max torque that keeps us at limit
    actualTorque = (maxMomentum - currentMomentum) / deltaTime;
    actualTorque = glm::clamp(actualTorque, 0.0, maxTorque);
  }
  else if (predictedMomentum < -maxMomentum)
  {
    // Calculate max reverse torque
    actualTorque = (-maxMomentum - currentMomentum) / deltaTime;
    actualTorque = glm::clamp(actualTorque, -maxTorque, 0.0);
  }

  // Limit 3: Back-off factor near saturation
  // As momentum approaches limit, reduce available torque for safety margin
  double saturation = getSaturation();
  if (saturation > 0.85)
  {
    // Linear back-off: 85% saturation = full torque, 95% = 50% torque
    double backoffFactor = 1.0 - 5.0 * (saturation - 0.85); // 1.0 at 85%, 0.5 at 95%
    backoffFactor = glm::clamp(backoffFactor, 0.5, 1.0);
    actualTorque *= backoffFactor;
  }
}

void ReactionWheel::updatePower()
{
  /**
   * Simple power model:
   * P = P_idle + k * |τ|
   *
   * Where:
   * - P_idle = constant power for electronics (2W typical)
   * - k = torque-dependent power (10 W/(N·m) typical)
   */

  if (!enabled)
  {
    powerConsumption = 0.0;
    return;
  }

  powerConsumption = idlePower + torquePowerCoeff * std::abs(actualTorque);
}
