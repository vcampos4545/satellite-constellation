#include "RK4Integrator.h"

void RK4Integrator::integratePositionVelocity(
    glm::dvec3 &position,
    glm::dvec3 &velocity,
    double deltaTime,
    std::function<glm::dvec3(const glm::dvec3 &, const glm::dvec3 &)> accelFunc)
{
  // RK4 integration for position and velocity
  // More accurate than Euler: O(h^4) vs O(h) local error
  //
  // State: y = [position, velocity]
  // Derivative: dy/dt = [velocity, acceleration]

  // k1 = f(t, y)
  glm::dvec3 k1_vel = accelFunc(position, velocity);
  glm::dvec3 k1_pos = velocity;

  // k2 = f(t + dt/2, y + k1*dt/2)
  glm::dvec3 pos_k2 = position + k1_pos * (deltaTime * 0.5);
  glm::dvec3 vel_k2 = velocity + k1_vel * (deltaTime * 0.5);
  glm::dvec3 k2_vel = accelFunc(pos_k2, vel_k2);
  glm::dvec3 k2_pos = vel_k2;

  // k3 = f(t + dt/2, y + k2*dt/2)
  glm::dvec3 pos_k3 = position + k2_pos * (deltaTime * 0.5);
  glm::dvec3 vel_k3 = velocity + k2_vel * (deltaTime * 0.5);
  glm::dvec3 k3_vel = accelFunc(pos_k3, vel_k3);
  glm::dvec3 k3_pos = vel_k3;

  // k4 = f(t + dt, y + k3*dt)
  glm::dvec3 pos_k4 = position + k3_pos * deltaTime;
  glm::dvec3 vel_k4 = velocity + k3_vel * deltaTime;
  glm::dvec3 k4_vel = accelFunc(pos_k4, vel_k4);
  glm::dvec3 k4_pos = vel_k4;

  // Update: y_new = y + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
  position += (deltaTime / 6.0) * (k1_pos + 2.0 * k2_pos + 2.0 * k3_pos + k4_pos);
  velocity += (deltaTime / 6.0) * (k1_vel + 2.0 * k2_vel + 2.0 * k3_vel + k4_vel);
}

void RK4Integrator::integrateAttitude(
    glm::dquat &quaternion,
    glm::dvec3 &angularVelocity,
    double deltaTime,
    std::function<glm::dvec3(const glm::dvec3 &)> angularAccelFunc,
    std::function<glm::dquat(const glm::dquat &, const glm::dvec3 &)> quatDotFunc)
{
  // RK4 integration for attitude (quaternion + angular velocity)
  //
  // State: [quaternion, angularVelocity]
  // Derivatives:
  //   dq/dt = 0.5 * Ω(ω) * q  (quaternion kinematics)
  //   dω/dt = I^-1 * (τ - ω × (I*ω))  (Euler's rotational dynamics)

  // k1 = f(t, y)
  glm::dvec3 k1_omega = angularAccelFunc(angularVelocity);
  glm::dquat k1_quat = quatDotFunc(quaternion, angularVelocity);

  // k2 = f(t + dt/2, y + k1*dt/2)
  glm::dvec3 omega_k2 = angularVelocity + k1_omega * (deltaTime * 0.5);
  glm::dquat quat_k2 = quaternion + k1_quat * (deltaTime * 0.5);
  glm::dvec3 k2_omega = angularAccelFunc(omega_k2);
  glm::dquat k2_quat = quatDotFunc(quat_k2, omega_k2);

  // k3 = f(t + dt/2, y + k2*dt/2)
  glm::dvec3 omega_k3 = angularVelocity + k2_omega * (deltaTime * 0.5);
  glm::dquat quat_k3 = quaternion + k2_quat * (deltaTime * 0.5);
  glm::dvec3 k3_omega = angularAccelFunc(omega_k3);
  glm::dquat k3_quat = quatDotFunc(quat_k3, omega_k3);

  // k4 = f(t + dt, y + k3*dt)
  glm::dvec3 omega_k4 = angularVelocity + k3_omega * deltaTime;
  glm::dquat quat_k4 = quaternion + k3_quat * deltaTime;
  glm::dvec3 k4_omega = angularAccelFunc(omega_k4);
  glm::dquat k4_quat = quatDotFunc(quat_k4, omega_k4);

  // Update: y_new = y + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
  angularVelocity += (deltaTime / 6.0) * (k1_omega + 2.0 * k2_omega + 2.0 * k3_omega + k4_omega);
  quaternion += (deltaTime / 6.0) * (k1_quat + 2.0 * k2_quat + 2.0 * k3_quat + k4_quat);

  // Normalize quaternion to prevent drift from unit magnitude
  quaternion = glm::normalize(quaternion);
}
