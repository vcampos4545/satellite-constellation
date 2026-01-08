#include "CelestialBody.h"
#include "Constants.h"
#include "RK4Integrator.h"
#include <cmath>
#include <memory>
#include <vector>

CelestialBody::CelestialBody(const glm::dvec3 &position, double mass, double radius, const glm::vec3 &rotationAxis, const double rotationAngularVelocity)
    : position(position), velocity(0.0, 0.0, 0.0), mass(mass), radius(radius), rotation(0.0f), rotationAxis(glm::normalize(rotationAxis)), rotationAngularVelocity(rotationAngularVelocity), enablePhysics(false)
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

  // Use modular RK4 integrator for orbital dynamics
  // Lambda to compute gravitational acceleration from all other bodies
  auto accelFunc = [this, &allBodies](const glm::dvec3 &pos, const glm::dvec3 &vel) -> glm::dvec3
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

      // Gravitational acceleration: a = (G * M / r²) * direction
      accel += (G * body->getMass() / (distance * distance * distance)) * toBody;
    }
    return accel;
  };

  RK4Integrator::integratePositionVelocity(position, velocity, deltaTime, accelFunc);
  recordPosition();
}
