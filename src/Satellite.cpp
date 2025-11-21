#include "Satellite.h"
#include <cmath>

const double G = 6.67430e-11; // Gravitational constant (m^3 kg^-1 s^-2)

Satellite::Satellite(const glm::dvec3 &position, const glm::dvec3 &velocity, const glm::vec3 &color, int planeId, int indexInPlane)
    : position(position), velocity(velocity), color(color), planeId(planeId), indexInPlane(indexInPlane)
{
}

void Satellite::update(double deltaTime, const glm::dvec3 &earthPosition, double earthMass)
{
  // Calculate gravitational force from Earth
  glm::dvec3 toEarth = earthPosition - position;
  double distance = glm::length(toEarth);

  if (distance < 1.0) return; // Avoid division by zero

  // F = G * M * m / r^2, but we only need acceleration (F/m = G * M / r^2)
  glm::dvec3 direction = glm::normalize(toEarth);
  double accelerationMagnitude = G * earthMass / (distance * distance);
  glm::dvec3 acceleration = direction * accelerationMagnitude;

  // Update velocity and position (simple Euler integration)
  velocity += acceleration * deltaTime;
  position += velocity * deltaTime;
}

void Satellite::calculateFullOrbit(const glm::dvec3 &earthPosition, double earthMass, int numPoints)
{
  orbitPath.clear();

  // Calculate orbital period using vis-viva equation
  double distance = glm::length(position - earthPosition);
  double speed = glm::length(velocity);

  // For circular orbit: T = 2*pi*r/v
  // Using semi-major axis approximation
  double orbitalRadius = distance;
  double orbitalPeriod = 2.0 * 3.14159265359 * sqrt((orbitalRadius * orbitalRadius * orbitalRadius) / (G * earthMass));

  // Simulate one complete orbit
  double timeStep = orbitalPeriod / numPoints;

  // Save current state
  glm::dvec3 savedPos = position;
  glm::dvec3 savedVel = velocity;

  // Simulate forward
  for (int i = 0; i < numPoints; ++i)
  {
    orbitPath.push_back(position);

    // Calculate gravitational acceleration
    glm::dvec3 toEarth = earthPosition - position;
    double dist = glm::length(toEarth);

    if (dist > 1.0)
    {
      glm::dvec3 direction = glm::normalize(toEarth);
      double accelMag = G * earthMass / (dist * dist);
      glm::dvec3 acceleration = direction * accelMag;

      // Update velocity and position
      velocity += acceleration * timeStep;
      position += velocity * timeStep;
    }
  }

  // Restore original state
  position = savedPos;
  velocity = savedVel;
}
