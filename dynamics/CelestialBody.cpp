#include "CelestialBody.h"
#include "Constants.h"
#include <cmath>
#include <memory>
#include <vector>

CelestialBody::CelestialBody(const glm::dvec3 &position, double mass, double radius, const glm::vec3 &color, const glm::vec3 &rotationAxis, const double rotationAngularVelocity)
    : position(position), velocity(0.0, 0.0, 0.0), mass(mass), radius(radius), color(color), rotation(0.0f), rotationAxis(glm::normalize(rotationAxis)), rotationAngularVelocity(rotationAngularVelocity), enablePhysics(false)
{
  // Add initial position to orbit path
  orbitPath.push_back(position);
}

void CelestialBody::update(double deltaTime, const std::vector<std::shared_ptr<CelestialBody>> &allBodies)
{
  // Always update rotation (independent of orbital physics)
  rotation += rotationAngularVelocity * deltaTime;

  // Only update orbital physics if enabled
  if (!enablePhysics)
    return;

  // RK4 integration for better accuracy
  // Lambda to compute acceleration at a given position
  auto computeAccelAtPos = [this, &allBodies](const glm::dvec3 &pos) -> glm::dvec3
  {
    glm::dvec3 accel(0.0);
    for (const auto &body : allBodies)
    {
      if (body.get() == this)
        continue;

      glm::dvec3 toBody = body->getPosition() - pos;
      double distance = glm::length(toBody);

      if (distance < 1.0)
        continue;

      accel += (G * body->getMass() / (distance * distance * distance)) * toBody;
    }
    return accel;
  };

  // k1 = f(t, y)
  glm::dvec3 k1_vel = computeAccelAtPos(position);
  glm::dvec3 k1_pos = velocity;

  // k2 = f(t + dt/2, y + k1*dt/2)
  glm::dvec3 pos2 = position + k1_pos * (deltaTime * 0.5);
  glm::dvec3 vel2 = velocity + k1_vel * (deltaTime * 0.5);
  glm::dvec3 k2_vel = computeAccelAtPos(pos2);
  glm::dvec3 k2_pos = vel2;

  // k3 = f(t + dt/2, y + k2*dt/2)
  glm::dvec3 pos3 = position + k2_pos * (deltaTime * 0.5);
  glm::dvec3 vel3 = velocity + k2_vel * (deltaTime * 0.5);
  glm::dvec3 k3_vel = computeAccelAtPos(pos3);
  glm::dvec3 k3_pos = vel3;

  // k4 = f(t + dt, y + k3*dt)
  glm::dvec3 pos4 = position + k3_pos * deltaTime;
  glm::dvec3 vel4 = velocity + k3_vel * deltaTime;
  glm::dvec3 k4_vel = computeAccelAtPos(pos4);
  glm::dvec3 k4_pos = vel4;

  // Update: y_new = y + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
  velocity += (deltaTime / 6.0) * (k1_vel + 2.0 * k2_vel + 2.0 * k3_vel + k4_vel);
  position += (deltaTime / 6.0) * (k1_pos + 2.0 * k2_pos + 2.0 * k3_pos + k4_pos);

  // ========== ORBIT PATH HISTORY ==========
  // Increment update iteration counter
  updateIterationCount++;

  // Add current position to historical trail every N iterations
  // This gives smooth trails regardless of time warp speed
  // Saves complete history since simulation start
  if (updateIterationCount % orbitPathSaveInterval == 0)
  {
    orbitPath.push_back(position);
  }
}
