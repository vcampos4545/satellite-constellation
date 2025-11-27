#include "Satellite.h"
#include "Constants.h"
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>

Satellite::Satellite(const glm::dvec3 &position, const glm::dvec3 &velocity, const glm::vec3 &color, int planeId, int indexInPlane)
    : position(position), velocity(velocity), color(color), planeId(planeId), indexInPlane(indexInPlane), attitude(glm::dvec3(0.0, 0.0, 0.0))
{
}

void Satellite::update(double deltaTime, const glm::dvec3 &earthCenter, double earthMass)
{
  // Calculate gravitational force from Earth
  glm::dvec3 toEarth = earthCenter - position;
  double distance = glm::length(toEarth);

  if (distance < 1.0)
    return; // Avoid division by zero

  // F = G * M * m / r^2, but we only need acceleration (F/m = G * M / r^2)
  glm::dvec3 direction = glm::normalize(toEarth);
  double accelerationMagnitude = G * earthMass / (distance * distance);
  glm::dvec3 acceleration = direction * accelerationMagnitude;

  // Update velocity and position (simple Euler integration)
  velocity += acceleration * deltaTime;
  position += velocity * deltaTime;

  // Update footprint every frame
  calculateFootprint(earthCenter, 60);
}

void Satellite::calculateFullOrbit(const glm::dvec3 &earthCenter, double earthMass, int numPoints)
{
  orbitPath.clear();

  // Calculate orbital period using vis-viva equation
  double distance = glm::length(position - earthCenter);
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

  // Simulate forward (numPoints + 1 to ensure that the full circle is connected)
  for (int i = 0; i <= numPoints; ++i)
  {
    orbitPath.push_back(position);

    // Calculate gravitational acceleration
    glm::dvec3 toEarth = earthCenter - position;
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

void Satellite::calculateFootprint(const glm::dvec3 &earthCenter, int numPoints)
{
  footprintCircle.clear();

  // Vector from Earth center to satellite
  glm::dvec3 toSatellite = position - earthCenter;
  double satelliteDistance = glm::length(toSatellite);
  glm::dvec3 nadirDir = glm::normalize(toSatellite);

  // Horizon angle (in radians)
  double lambda0 = acos(EARTH_RADIUS / satelliteDistance);

  // TODO: Decrease angle by some amount for feasible footprint (i.e. lambda0 - 5 degrees)
  // This accounts for ground station minimum elevation angle

  // Find a perpendicular vector to nadirDir to start with
  glm::dvec3 perpendicular = glm::normalize(glm::cross(nadirDir, glm::dvec3(0.0, 1.0, 0.0)));

  // Create initial horizon point by rotating perpendicular vector around another perpendicular
  glm::dvec3 secondPerp = glm::normalize(glm::cross(nadirDir, perpendicular));
  glm::dvec3 initialHorizonDir = nadirDir * cos(lambda0) + secondPerp * sin(lambda0);
  glm::dvec3 initialHorizonPoint = earthCenter + initialHorizonDir * EARTH_RADIUS;

  // Rotate this initial point around the nadirDir axis to create the footprint circle
  for (int i = 0; i <= numPoints; ++i)
  {
    double angle = (2.0 * PI * i) / numPoints;

    // Create rotation matrix around nadirDir axis
    glm::dmat4 rotationMatrix = glm::rotate(glm::dmat4(1.0), angle, nadirDir);

    // Rotate the initial horizon point around the axis
    glm::dvec3 rotatedPoint = glm::dvec3(rotationMatrix * glm::dvec4(initialHorizonPoint - earthCenter, 1.0)) + earthCenter;

    footprintCircle.push_back(rotatedPoint);
  }
}
