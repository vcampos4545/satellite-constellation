#include "Universe.h"
#include <cmath>
#include <algorithm>
#include <glm/gtc/matrix_transform.hpp>

// Real physical constants
const double EARTH_MASS = 5.972e24;   // kg
const double EARTH_RADIUS = 6.371e6;  // meters
const double SUN_MASS = 1.989e30;     // kg
const double SUN_RADIUS = 6.96e8;     // meters
const double AU = 1.496e11;           // Astronomical Unit in meters
const double G = 6.67430e-11;         // Gravitational constant
const double GEO_ALTITUDE = 35.786e6; // GEO altitude above Earth surface (meters)
const double LEO_ALTITUDE = 550e3;    // Starlink altitude ~550 km
const double PI = 3.14159265359;

Universe::Universe()
{
}

void Universe::addBody(std::shared_ptr<CelestialBody> body)
{
  bodies.push_back(body);
}

void Universe::addSatellite(std::shared_ptr<Satellite> satellite)
{
  satellites.push_back(satellite);
}

void Universe::initializeEarthAndSun()
{
  // Create Earth at origin
  earth = std::make_shared<CelestialBody>(
      glm::dvec3(0.0, 0.0, 0.0),
      EARTH_MASS,
      EARTH_RADIUS,
      glm::vec3(0.2f, 0.4f, 0.8f) // Bluish color
  );
  bodies.push_back(earth);

  // Create Sun at distance (1 AU away, positioned for lighting)
  sun = std::make_shared<CelestialBody>(
      glm::dvec3(AU, 0.0, 0.0),
      SUN_MASS,
      SUN_RADIUS,
      glm::vec3(1.0f, 0.9f, 0.6f) // Yellowish color
  );
  bodies.push_back(sun);
}

void Universe::addGEOSatellite()
{
  // Calculate GEO orbit parameters
  double orbitalRadius = EARTH_RADIUS + GEO_ALTITUDE;            // Distance from Earth center
  double orbitalVelocity = sqrt(G * EARTH_MASS / orbitalRadius); // Circular orbit velocity

  // Start satellite at (orbital radius, 0, 0) with velocity in +Z direction
  glm::dvec3 position(orbitalRadius, 0.0, 0.0);
  glm::dvec3 velocity(0.0, 0.0, -orbitalVelocity);

  // Create satellite with bright color for visibility
  auto satellite = std::make_shared<Satellite>(position, velocity, glm::vec3(1.0f, 1.0f, 0.0f));

  // Calculate complete orbital path
  satellite->calculateFullOrbit(earth->getPosition(), earth->getMass(), 120);

  satellites.push_back(satellite);
}

void Universe::addStarlinkConstellation(int numPlanes, int satellitesPerPlane)
{
  // Starlink-like parameters
  double orbitalRadius = EARTH_RADIUS + LEO_ALTITUDE;
  double orbitalVelocity = sqrt(G * EARTH_MASS / orbitalRadius);
  double inclination = 53.0 * PI / 180.0; // 53 degrees inclination (typical for Starlink)

  // Create satellites in multiple orbital planes
  for (int plane = 0; plane < numPlanes; ++plane)
  {
    // Right ascension of ascending node (RAAN) - evenly distribute planes
    double raan = (2.0 * PI * plane) / numPlanes;

    // Create satellites in this plane
    for (int sat = 0; sat < satellitesPerPlane; ++sat)
    {
      // True anomaly - position along the orbit
      double trueAnomaly = (2.0 * PI * sat) / satellitesPerPlane;

      // Start with position in orbital plane (x-z plane, circular orbit)
      glm::dvec3 position(
          orbitalRadius * cos(trueAnomaly),
          0.0,
          orbitalRadius * sin(trueAnomaly));

      // Velocity perpendicular to position (tangent to orbit)
      glm::dvec3 velocity(
          -orbitalVelocity * sin(trueAnomaly),
          0.0,
          orbitalVelocity * cos(trueAnomaly));

      // Apply inclination (rotate around x-axis)
      glm::dmat4 inclinationMatrix = glm::rotate(glm::dmat4(1.0), inclination, glm::dvec3(1.0, 0.0, 0.0));
      position = glm::dvec3(inclinationMatrix * glm::dvec4(position, 1.0));
      velocity = glm::dvec3(inclinationMatrix * glm::dvec4(velocity, 0.0));

      // Apply RAAN (rotate around y-axis)
      glm::dmat4 raanMatrix = glm::rotate(glm::dmat4(1.0), raan, glm::dvec3(0.0, 1.0, 0.0));
      position = glm::dvec3(raanMatrix * glm::dvec4(position, 1.0));
      velocity = glm::dvec3(raanMatrix * glm::dvec4(velocity, 0.0));

      // Create satellite with bright cyan/white color for better visibility
      auto satellite = std::make_shared<Satellite>(
          position,
          velocity,
          glm::vec3(0.3f, 0.9f, 1.0f) // Bright cyan color
      );

      // Calculate complete orbital path
      satellite->calculateFullOrbit(earth->getPosition(), earth->getMass(), 120);

      satellites.push_back(satellite);
    }
  }
}

void Universe::update(double deltaTime, double maxPhysicsStep)
{
  // Use sub-stepping to keep physics stable even with large time warps
  // Break large time steps into smaller chunks
  double remainingTime = deltaTime;

  while (remainingTime > 0.0)
  {
    // Take the smaller of: remaining time or max physics step
    double stepTime = std::min(remainingTime, maxPhysicsStep);

    // Update all satellites with this smaller step
    for (auto &satellite : satellites)
    {
      satellite->update(stepTime, earth->getPosition(), earth->getMass());
    }

    remainingTime -= stepTime;
  }
}
