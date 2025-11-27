#include "Universe.h"
#include "Constants.h"
#include "Config.h"
#include "MathUtils.h"
#include "GroundStation.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>

Universe::Universe() : moonOrbitAngle(0.0)
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

std::shared_ptr<Satellite> Universe::createSatelliteWithOrbit(
    const glm::dvec3 &position,
    const glm::dvec3 &velocity,
    const glm::vec3 &color,
    int planeId,
    int indexInPlane)
{
  auto satellite = std::make_shared<Satellite>(position, velocity, color, planeId, indexInPlane);

  // Calculate complete orbital path
  satellite->calculateFullOrbit(earth->getPosition(), earth->getMass(), 120);

  // Calculate footprint circle on Earth's surface
  satellite->calculateFootprint(earth->getPosition(), 60);

  return satellite;
}

void Universe::initializeEarthSunAndMoon()
{
  // Create Earth at origin with 23.5 degree axial tilt
  // Rotation axis tilted from Y-axis by 23.5 degrees towards the Sun (+X direction)
  float axialTiltDeg = 23.5f;
  float axialTiltRad = axialTiltDeg * PI / 180.0f;
  glm::vec3 earthRotationAxis(
      sin(axialTiltRad), // x component
      cos(axialTiltRad), // y component
      0.0f               // z component
  );

  earth = std::make_shared<CelestialBody>(
      glm::dvec3(0.0, 0.0, 0.0),
      EARTH_MASS,
      EARTH_RADIUS,
      glm::vec3(0.2f, 0.4f, 0.8f) // Bluish color
                                  // earthRotationAxis             // Tilted rotation axis
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

  // Create Moon orbiting Earth
  moon = std::make_shared<CelestialBody>(
      glm::dvec3(MOON_ORBIT_RADIUS, 0.0, 0.0),
      MOON_MASS,
      MOON_RADIUS,
      glm::vec3(0.7f, 0.7f, 0.7f) // Gray color
  );
  bodies.push_back(moon);

  // Generate moon's orbit path (circular path in x-z plane)
  const int numOrbitPoints = 120;
  moonOrbitPath.clear();
  for (int i = 0; i < numOrbitPoints; ++i)
  {
    double angle = (2.0 * PI * i) / numOrbitPoints;
    glm::dvec3 orbitPoint(
        MOON_ORBIT_RADIUS * cos(angle),
        0.0,
        MOON_ORBIT_RADIUS * sin(angle));
    moonOrbitPath.push_back(orbitPoint);
  }
}

void Universe::addGEOSatellite()
{
  // Calculate GEO orbit parameters
  double orbitalRadius = EARTH_RADIUS + GEO_ALTITUDE;            // Distance from Earth center
  double orbitalVelocity = sqrt(G * EARTH_MASS / orbitalRadius); // Circular orbit velocity

  // Start satellite at (orbital radius, 0, 0) with velocity in +Z direction
  glm::dvec3 position(orbitalRadius, 0.0, 0.0);
  glm::dvec3 velocity(0.0, 0.0, -orbitalVelocity);

  // Create satellite with orbit and footprint
  auto satellite = createSatelliteWithOrbit(
      position,
      velocity,
      glm::vec3(0.3f, 0.9f, 1.0f),
      -1, // planeId (-1 for GEO satellites)
      0   // indexInPlane
  );

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

      // Create satellite with orbit and footprint
      auto satellite = createSatelliteWithOrbit(
          position,
          velocity,
          glm::vec3(0.3f, 0.9f, 1.0f), // Bright cyan color
          plane,                       // planeId
          sat                          // indexInPlane
      );

      satellites.push_back(satellite);
    }
  }
}

void Universe::addGroundStations()
{
  // Add ground stations at major city locations
  const int numCities = sizeof(MAJOR_CITIES) / sizeof(MAJOR_CITIES[0]);

  for (int i = 0; i < numCities; ++i)
  {
    const City &city = MAJOR_CITIES[i];
    auto groundStation = std::make_shared<GroundStation>(city.name, city.latitude, city.longitude);
    groundStations.push_back(groundStation);
  }

  std::cout << "Added " << numCities << " ground stations at major city locations" << std::endl;
}

void Universe::update(double deltaTime, double maxPhysicsStep)
{
  // Update earths rotation
  getEarth()->rotate(EARTH_ROTATION_SPEED * deltaTime);

  // Update ground station positions to rotate with Earth
  for (auto &groundStation : groundStations)
  {
    groundStation->updatePosition(earth->getRotation(), earth->getRotationAxis());
  }

  moonOrbitAngle += MOON_ANGULAR_VELOCITY * deltaTime;

  // Update moon's position (circular orbit around Earth in x-z plane)
  if (moon)
  {
    glm::dvec3 earthPos = earth->getPosition();
    moon->setPosition(glm::dvec3(
        earthPos.x + MOON_ORBIT_RADIUS * cos(moonOrbitAngle),
        earthPos.y,
        earthPos.z + MOON_ORBIT_RADIUS * sin(moonOrbitAngle)));
  }

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

  // Update ground station connections
  for (auto &groundStation : groundStations)
  {
    // Clear previous frame's visible satellites
    groundStation->clearVisibleSatellites();

    // Find all visible satellites for this ground station
    for (auto &satellite : satellites)
    {
      if (groundStation->isSatelliteVisible(satellite->getPosition(), earth->getPosition()))
      {
        groundStation->addVisibleSatellite(satellite);
      }
    }
  }
}
