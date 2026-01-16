#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <glm/glm.hpp>

/**
 * PID Controller for Attitude Control
 *
 * Implements a Proportional-Integral-Derivative controller for satellite attitude control.
 * Used to compute control torques based on attitude errors.
 *
 * The controller computes torque as:
 *   T = Kp*e + Ki*∫e*dt + Kd*de/dt
 *
 * Where:
 *   e = attitude error (radians)
 *   Kp = proportional gain (N·m/rad)
 *   Ki = integral gain (N·m/(rad·s))
 *   Kd = derivative gain (N·m·s/rad)
 *
 * Features:
 * - Auto-tuning based on desired settling time and damping ratio
 * - Integral anti-windup to prevent saturation
 * - Separate gains for each axis (supports anisotropic inertia)
 *
 * Usage:
 *   PIDController pid;
 *   pid.autoTune(inertiaTensor, settlingTime, dampingRatio);
 *   glm::dvec3 torque = pid.computeControl(error, errorRate, deltaTime);
 */
class PIDController
{
public:
  /**
   * Constructor
   * Initializes PID controller with default gains
   */
  PIDController();

  /**
   * Auto-tune PID gains based on system dynamics
   *
   * Uses Ziegler-Nichols-inspired tuning for second-order systems.
   * Computes gains to achieve desired settling time and damping ratio.
   *
   * @param inertiaTensor Moment of inertia tensor [Ixx, Iyy, Izz] (kg·m²)
   * @param settlingTime Desired 2% settling time (seconds)
   * @param dampingRatio Desired damping ratio (0-1, typically 0.7-0.9)
   *
   * Theory:
   * For a second-order system: I·θ'' + Kd·θ' + Kp·θ = 0
   * Natural frequency: ωn = sqrt(Kp/I)
   * Damping ratio: ζ = Kd/(2·sqrt(Kp·I))
   *
   * Settling time (2% criterion): ts ≈ 4/(ζ·ωn)
   * Rearranging: ωn = 4/(ζ·ts)
   *
   * Therefore:
   * Kp = I·ωn²
   * Kd = 2·ζ·I·ωn
   * Ki = small fraction of Kp (to eliminate steady-state error)
   */
  void autoTune(const glm::dvec3 &inertiaTensor,
                double settlingTime = 20.0,
                double dampingRatio = 0.9);

  /**
   * Compute control torque using PID algorithm
   *
   * @param error Current attitude error (radians, body frame)
   * @param errorRate Rate of change of error (rad/s, body frame)
   * @param deltaTime Time step (seconds)
   * @return Control torque (N·m, body frame)
   */
  glm::dvec3 computeControl(const glm::dvec3 &error,
                            const glm::dvec3 &errorRate,
                            double deltaTime);

  /**
   * Reset integral error accumulation
   * Call this when changing control modes or after large disturbances
   */
  void resetIntegralError();

  // Setters
  void setProportionalGain(double kp) { proportionalGain = kp; }
  void setIntegralGain(double ki) { integralGain = ki; }
  void setDerivativeGain(double kd) { derivativeGain = kd; }
  void setIntegralErrorMax(double max) { integralErrorMax = max; }

  // Getters
  double getProportionalGain() const { return proportionalGain; }
  double getIntegralGain() const { return integralGain; }
  double getDerivativeGain() const { return derivativeGain; }
  double getIntegralErrorMax() const { return integralErrorMax; }
  glm::dvec3 getIntegralError() const { return integralError; }

private:
  // PID gains
  double proportionalGain; // Kp - Proportional gain (N·m/rad)
  double integralGain;     // Ki - Integral gain (N·m/(rad·s))
  double derivativeGain;   // Kd - Derivative gain (N·m·s/rad)

  // Integral error state
  glm::dvec3 integralError;  // Accumulated error (rad·s)
  double integralErrorMax;   // Anti-windup limit (rad·s)

  /**
   * Apply anti-windup to integral error
   * Clamps each component to [-max, +max]
   */
  void applyAntiWindup();
};

#endif // PID_CONTROLLER_H
