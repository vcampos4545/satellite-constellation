#ifndef RK4_INTEGRATOR_H
#define RK4_INTEGRATOR_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <functional>

/**
 * Runge-Kutta 4th Order Integrator
 *
 * Provides 4th-order accurate numerical integration for orbital dynamics,
 * attitude dynamics, and other differential equations.
 *
 * More accurate and stable than Euler integration, especially critical
 * for long-duration orbital simulations.
 *
 * Reference: Numerical Recipes, Press et al.
 */
class RK4Integrator
{
public:
  /**
   * Integrate position and velocity (orbital dynamics)
   *
   * @param position Current position (will be updated)
   * @param velocity Current velocity (will be updated)
   * @param deltaTime Time step
   * @param accelFunc Function that computes acceleration given (pos, vel)
   */
  static void integratePositionVelocity(
      glm::dvec3 &position,
      glm::dvec3 &velocity,
      double deltaTime,
      std::function<glm::dvec3(const glm::dvec3 &, const glm::dvec3 &)> accelFunc);

  /**
   * Integrate angular velocity and quaternion (attitude dynamics)
   *
   * @param quaternion Current attitude quaternion (will be updated and normalized)
   * @param angularVelocity Current angular velocity (will be updated)
   * @param deltaTime Time step
   * @param angularAccelFunc Function that computes angular acceleration given omega
   * @param quatDotFunc Function that computes quaternion derivative given (q, omega)
   */
  static void integrateAttitude(
      glm::dquat &quaternion,
      glm::dvec3 &angularVelocity,
      double deltaTime,
      std::function<glm::dvec3(const glm::dvec3 &)> angularAccelFunc,
      std::function<glm::dquat(const glm::dquat &, const glm::dvec3 &)> quatDotFunc);
};

#endif // RK4_INTEGRATOR_H
