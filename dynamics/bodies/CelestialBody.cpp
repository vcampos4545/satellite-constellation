#include "CelestialBody.h"
#include "Constants.h"
#include "RK4Integrator.h"
#include "GravityModel.h"
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

  // Find Earth (central body at origin) for differential gravity calculations
  // In an Earth-centered frame, we need to account for the frame's acceleration
  glm::dvec3 earthPos(0.0);
  for (const auto &body : allBodies)
  {
    if (glm::length(body->getPosition()) < 1.0) // Body at origin is Earth
    {
      earthPos = body->getPosition();
      break;
    }
  }

  // Use modular RK4 integrator for orbital dynamics
  // Lambda to compute gravitational acceleration using differential gravity
  auto accelFunc = [this, &allBodies, earthPos](const glm::dvec3 &pos, const glm::dvec3 &vel) -> glm::dvec3
  {
    glm::dvec3 accel(0.0);

    for (const auto &body : allBodies)
    {
      if (body.get() == this)
        continue;

      glm::dvec3 bodyPos = body->getPosition();
      double bodyMass = body->getMass();

      // Check if this is Earth (central body at origin)
      bool isEarth = (glm::length(bodyPos - earthPos) < 1.0);

      if (isEarth)
      {
        // Direct gravity from Earth (central body)
        // Standard point-mass gravity since Earth is our reference frame
        accel += GravityModel::calculatePointMassGravity(pos, bodyPos, bodyMass);
      }
      else
      {
        // Differential gravity for third bodies (Sun, other planets, etc.)
        // This accounts for the fact that we're in an Earth-centered (non-inertial) frame
        // The frame itself is accelerating toward these bodies, so we only feel
        // the DIFFERENCE between the gravity on this body vs gravity on Earth
        // This is the "tidal effect" - the same physics that causes ocean tides
        accel += GravityModel::calculateThirdBodyPerturbation(pos, earthPos, bodyPos, bodyMass);
      }
    }
    return accel;
  };

  RK4Integrator::integratePositionVelocity(position, velocity, deltaTime, accelFunc);
  recordPosition();
}
