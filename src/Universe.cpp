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
  /* CUSTOMIZE SIMULATION HERE */

  initializeEarthSunAndMoon();

  // Add Starlink-like LEO constellation
  addStarlinkConstellation(2, 2); // 2 orbital planes, 2 satellites per plane = 4 satellites

  // Add Molniya constellation (highly elliptical orbit for high latitude coverage)
  addMolniyaConstellation(3); // 3 satellites for continuous coverage

  // Add ground stations for power reception at major cities
  addGroundStations();
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
  satellite->calculateFullOrbit(earth->getPosition(), earth->getMass(), sun->getPosition(), 120);

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

void Universe::addMolniyaConstellation(int numSatellites)
{
  // Molniya orbit parameters
  double semiMajorAxis = MOLNIYA_SEMI_MAJOR_AXIS;
  double eccentricity = MOLNIYA_ECCENTRICITY;
  double inclination = glm::radians(MOLNIYA_INCLINATION); // Convert to radians

  // Argument of perigee: 270° (apogee over northern hemisphere)
  double argOfPerigee = 270.0 * PI / 180.0;

  // For a typical Molniya constellation, satellites are evenly spaced in their orbits
  // with 8-hour phase separation (since period is ~12 hours, 3 satellites give continuous coverage)
  for (int sat = 0; sat < numSatellites; ++sat)
  {
    // Right ascension of ascending node - evenly distribute satellites
    double raan = (2.0 * PI * sat) / numSatellites;

    // Start all satellites at apogee for maximum dwell time visualization
    // True anomaly at apogee: 90° relative to argument of perigee
    double trueAnomaly = 90.0 * PI / 180.0;

    // Calculate position and velocity in orbital plane using orbital mechanics
    // r = a(1 - e²) / (1 + e*cos(ν))
    double radius = semiMajorAxis * (1.0 - eccentricity * eccentricity) /
                    (1.0 + eccentricity * cos(trueAnomaly));

    // Position in perifocal coordinates (orbital plane with perigee on x-axis)
    glm::dvec3 positionPerifocal(
        radius * cos(trueAnomaly),
        radius * sin(trueAnomaly),
        0.0);

    // Velocity in perifocal coordinates
    // v = sqrt(μ/p) where p = a(1-e²)
    double p = semiMajorAxis * (1.0 - eccentricity * eccentricity);
    double mu = G * EARTH_MASS;
    double velocityMagnitude = sqrt(mu / p);

    glm::dvec3 velocityPerifocal(
        -velocityMagnitude * sin(trueAnomaly),
        velocityMagnitude * (eccentricity + cos(trueAnomaly)),
        0.0);

    // Transform from perifocal to inertial frame
    // Use the same rotation sequence as Starlink for consistency
    // Perifocal frame has orbit in X-Y plane initially

    // Rotate perifocal coordinates to start in X-Z plane (matching Starlink convention)
    glm::dvec3 position(positionPerifocal.x, 0.0, positionPerifocal.y);
    glm::dvec3 velocity(velocityPerifocal.x, 0.0, velocityPerifocal.y);

    // 1. Rotate by argument of perigee around y-axis
    glm::dmat4 argPerigeeMatrix = glm::rotate(glm::dmat4(1.0), argOfPerigee, glm::dvec3(0.0, 1.0, 0.0));
    position = glm::dvec3(argPerigeeMatrix * glm::dvec4(position, 1.0));
    velocity = glm::dvec3(argPerigeeMatrix * glm::dvec4(velocity, 0.0));

    // 2. Rotate by inclination around x-axis (same as Starlink)
    glm::dmat4 inclinationMatrix = glm::rotate(glm::dmat4(1.0), inclination, glm::dvec3(1.0, 0.0, 0.0));
    position = glm::dvec3(inclinationMatrix * glm::dvec4(position, 1.0));
    velocity = glm::dvec3(inclinationMatrix * glm::dvec4(velocity, 0.0));

    // 3. Rotate by RAAN around y-axis (same as Starlink)
    glm::dmat4 raanMatrix = glm::rotate(glm::dmat4(1.0), raan, glm::dvec3(0.0, 1.0, 0.0));
    position = glm::dvec3(raanMatrix * glm::dvec4(position, 1.0));
    velocity = glm::dvec3(raanMatrix * glm::dvec4(velocity, 0.0));

    // Create satellite with orbit and footprint
    auto satellite = createSatelliteWithOrbit(
        position,
        velocity,
        glm::vec3(1.0f, 0.5f, 0.2f), // Orange/red color for Molniya
        -2,                          // planeId (-2 for Molniya satellites)
        sat                          // indexInPlane
    );

    satellites.push_back(satellite);
  }

  std::cout << "Added Molniya constellation with " << numSatellites << " satellites" << std::endl;
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
      satellite->update(stepTime, earth->getPosition(), earth->getMass(), sun->getPosition());
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
