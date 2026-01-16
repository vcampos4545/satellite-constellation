#include "PIDController.h"
#include <glm/gtc/constants.hpp>
#include <algorithm>

PIDController::PIDController()
    : proportionalGain(0.07),        // Default gain for ~50 kg·m² satellite
      integralGain(0.0),             // Conservative default (disabled)
      derivativeGain(0.5),           // Default damping gain
      integralError(0.0),            // Start with no accumulated error
      integralErrorMax(10.0)         // Anti-windup limit (rad·s)
{
}

void PIDController::autoTune(const glm::dvec3 &inertiaTensor,
                              double settlingTime,
                              double dampingRatio)
{
  // Calculate natural frequency for desired settling time (2% criterion)
  // For 2% settling: t_s ≈ 4/(ζ·ω_n), so ω_n = 4/(ζ·t_s)
  double omega_n = 4.0 / (dampingRatio * settlingTime);

  // Use average moment of inertia
  // For anisotropic satellites, could tune per-axis, but average works well in practice
  double I_avg = (inertiaTensor.x + inertiaTensor.y + inertiaTensor.z) / 3.0;

  // Calculate gains for desired damping response
  // Kp = I·ω_n² (stiffness term - restoring torque proportional to error)
  // Kd = 2·ζ·I·ω_n (damping term - opposes angular velocity)
  // Ki = small fraction of Kp (eliminates steady-state error, but risk of instability)
  proportionalGain = I_avg * omega_n * omega_n;
  derivativeGain = 2.0 * dampingRatio * I_avg * omega_n;
  integralGain = 0.01 * proportionalGain; // Very conservative (1% of Kp) to avoid instability

  // Reset integral error when re-tuning
  resetIntegralError();
}

glm::dvec3 PIDController::computeControl(const glm::dvec3 &error,
                                         const glm::dvec3 &errorRate,
                                         double deltaTime)
{
  // Proportional term: Kp * error
  glm::dvec3 proportionalTerm = proportionalGain * error;

  // Integral term: Ki * ∫error·dt
  // Accumulate error over time
  integralError += error * deltaTime;
  applyAntiWindup(); // Prevent windup
  glm::dvec3 integralTerm = integralGain * integralError;

  // Derivative term: Kd * (de/dt)
  // Note: errorRate is already the derivative of error
  glm::dvec3 derivativeTerm = derivativeGain * errorRate;

  // Total control torque
  glm::dvec3 controlTorque = proportionalTerm + integralTerm + derivativeTerm;

  return controlTorque;
}

void PIDController::resetIntegralError()
{
  integralError = glm::dvec3(0.0);
}

void PIDController::applyAntiWindup()
{
  // Clamp each component of integral error to prevent windup
  integralError.x = glm::clamp(integralError.x, -integralErrorMax, integralErrorMax);
  integralError.y = glm::clamp(integralError.y, -integralErrorMax, integralErrorMax);
  integralError.z = glm::clamp(integralError.z, -integralErrorMax, integralErrorMax);
}
