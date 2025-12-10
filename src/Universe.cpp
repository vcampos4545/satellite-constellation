#include "Universe.h"
#include "Constants.h"
#include "Config.h"
#include "MathUtils.h"
#include "GroundStation.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>
#include <glm/gtc/matrix_transform.hpp>

Universe::Universe() : moonOrbitAngle(0.0)
{
  /* CUSTOMIZE SIMULATION HERE */

  initializeEarthSunAndMoon();

  addGPSConstellation();

  // Add Starlink-like LEO constellation
  // addStarlinkConstellation(8, 1); // 2 orbital planes, 2 satellites per plane = 4 satellites

  // Add Molniya constellation (highly elliptical orbit for high latitude coverage)
  // addMolniyaConstellation(3); // 3 satellites for continuous coverage

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

  // Enable ADCS with reaction wheels
  satellite->enableReactionWheels(true);
  satellite->setControlMode(AttitudeControlMode::TARGET_TRACKING);
  satellite->setControlAlgorithm(ControlAlgorithm::PID);
  // satellite->setControlAlgorithm(ControlAlgorithm::MPC);

  // Calculate complete orbital path
  satellite->calculateFullOrbit(earth->getPosition(), earth->getMass(), sun->getPosition(), moon->getPosition(), 120);

  // Calculate footprint circle on Earth's surface
  satellite->calculateFootprint(earth->getPosition(), 60);

  return satellite;
}

void Universe::initializeEarthSunAndMoon()
{
  // ========== EARTH ==========
  // Create Earth at origin
  // Equatorial plane is horizontal (XZ plane), rotation axis is Y-axis
  // North pole points up (+Y), equator is in XZ plane
  glm::vec3 earthRotationAxis(0.0f, 1.0f, 0.0f); // Vertical (Y-axis)

  earth = std::make_shared<CelestialBody>(
      glm::dvec3(0.0, 0.0, 0.0), // Position at origin
      EARTH_MASS,
      EARTH_RADIUS,
      glm::vec3(0.2f, 0.4f, 0.8f), // Bluish color
      earthRotationAxis,
      EARTH_ROTATION_ANGULAR_VELOCITY);
  bodies.push_back(earth);

  // ========== SUN ==========
  // Position sun for northern hemisphere summer (June 21st)
  // At summer solstice, sun is 23.44° above the ecliptic (which is tilted from equator)
  // Since equatorial plane is XZ (horizontal), ecliptic is tilted by obliquity
  // Sun should be above the equatorial plane by +23.44° for northern summer
  double sunElevationAngle = ECLIPTIC_OBLIQUITY * PI / 180.0; // radians

  // Place sun at 1 AU distance, elevated above equatorial plane for northern summer
  // Sun in +X direction (local noon for observer at 0° longitude), elevated in +Y
  glm::dvec3 sunPosition(
      AU * cos(sunElevationAngle), // X component (reduced due to elevation)
      AU * sin(sunElevationAngle), // Y component (elevated for northern summer)
      0.0                          // Z component
  );

  glm::vec3 sunRotationAxis(0.0f, 1.0f, 0.0f);      // Sun also rotates around Y-axis
  double sunRotationPeriod = 25.38 * 24.0 * 3600.0; // 25.38 days at equator
  double sunRotationAngularVelocity = 2.0 * PI / sunRotationPeriod;

  sun = std::make_shared<CelestialBody>(
      sunPosition,
      SUN_MASS,
      SUN_RADIUS,
      glm::vec3(1.0f, 0.9f, 0.6f), // Yellowish color
      sunRotationAxis,
      sunRotationAngularVelocity);
  bodies.push_back(sun);

  // ========== MOON ==========
  // Create Moon with accurate orbital parameters
  // Moon's orbit is inclined 5.145° to the ecliptic
  // Ecliptic itself is tilted 23.44° from Earth's equator
  // Total inclination from equator = 5.145° (to ecliptic) + obliquity effects

  // Start moon at periapsis (closest approach)
  double periapsis = MOON_SEMI_MAJOR_AXIS * (1.0 - MOON_ECCENTRICITY);

  // Moon's orbital plane is inclined to ecliptic by 5.145°
  // Ecliptic is the XZ plane tilted by 23.44° from equator
  // We'll place moon in a plane inclined from the ecliptic
  double moonInclination = MOON_INCLINATION_TO_ECLIPTIC * PI / 180.0;

  // Initial position: periapsis in XZ plane (ecliptic), then apply inclination
  glm::dvec3 moonPosition(
      periapsis * cos(moonInclination), // X
      periapsis * sin(moonInclination), // Y (small, due to inclination)
      0.0                               // Z
  );

  // Calculate orbital velocity at periapsis using vis-viva equation
  // v = sqrt(μ * (2/r - 1/a))
  double mu = G * EARTH_MASS;
  double moonVelocity = sqrt(mu * (2.0 / periapsis - 1.0 / MOON_SEMI_MAJOR_AXIS));

  // Velocity perpendicular to position, in the orbital plane
  glm::dvec3 moonVel(
      0.0,                                 // X
      moonVelocity * cos(moonInclination), // Y component
      -moonVelocity                        // Z component (tangent to orbit)
  );

  // Moon's rotation axis is tilted 6.68° from its orbital plane normal
  // Since orbital plane is inclined, we need to compute the rotation axis
  // For simplicity, approximate rotation axis close to Y with small tilt
  double moonAxisTilt = MOON_AXIAL_TILT * PI / 180.0;
  glm::vec3 moonRotationAxis(
      sin(moonAxisTilt), // Small X component due to tilt
      cos(moonAxisTilt), // Mostly vertical (Y)
      0.0f               // Z
  );

  moon = std::make_shared<CelestialBody>(
      moonPosition,
      MOON_MASS,
      MOON_RADIUS,
      glm::vec3(0.7f, 0.7f, 0.7f), // Gray color
      moonRotationAxis,
      MOON_ROTATION_ANGULAR_VELOCITY);
  moon->setVelocity(moonVel);
  moon->enablePhysicsUpdate(true); // Enable physics simulation for the moon
  bodies.push_back(moon);
}

void Universe::addGPSConstellation()
{
  // GPS has ~ 24 satellites in 6 planes at 55 degree inclination
  double inclination = glm::radians(55.0);
  double orbitalRadius = EARTH_RADIUS + GPS_ALTITUDE;
  double orbitalVelocity = sqrt(G * EARTH_MASS / orbitalRadius);
  int numPlanes = 6;
  int numSatellites = 24;
  int numSatellitesPerPlane = numSatellites / numPlanes;

  for (int plane = 0; plane < numPlanes; ++plane)
  {
    double raan = (2.0 * PI * plane) / numPlanes;

    for (int sat = 0; sat < numSatellitesPerPlane; ++sat)
    {
      double trueAnomaly = (2.0 * PI * sat) / numSatellitesPerPlane;

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
          glm::vec3(0.3f, 0.9f, 0.0f), // Bright cyan color
          plane,                       // planeId
          sat                          // indexInPlane
      );

      satellites.push_back(satellite);
    }
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
  // Use sub-stepping to keep physics stable even with large time warps
  // Break large time steps into smaller chunks
  double remainingTime = deltaTime;

  while (remainingTime > 0.0)
  {
    // Take the smaller of: remaining time or max physics step
    double stepTime = std::min(remainingTime, maxPhysicsStep);

    // ========== UPDATE ALL CELESTIAL BODIES ==========
    // Each celestial body updates its rotation and (if physics enabled) orbital position
    for (auto &body : bodies)
    {
      body->update(stepTime, bodies);
    }

    // ========== UPDATE ALL SATELLITES ==========
    for (auto &satellite : satellites)
    {
      // Find closest ground station for target tracking
      if (!groundStations.empty())
      {
        double minDistance = std::numeric_limits<double>::max();
        glm::dvec3 closestStationPos;

        for (const auto &station : groundStations)
        {
          glm::dvec3 stationPos = station->getPosition();
          double distance = glm::length(satellite->getPosition() - stationPos);

          if (distance < minDistance)
          {
            minDistance = distance;
            closestStationPos = stationPos;
          }
        }

        // Update satellite's tracking target
        satellite->updateTargetTracking(closestStationPos);
      }

      // Update satellite (handles both ADCS and orbital dynamics internally)
      satellite->update(stepTime, earth->getPosition(), earth->getMass(), sun->getPosition(), moon->getPosition());
    }

    remainingTime -= stepTime;
  }

  // ========== UPDATE GROUND STATIONS ==========
  // Ground stations rotate with Earth
  for (auto &groundStation : groundStations)
  {
    groundStation->updatePosition(earth->getRotation(), earth->getRotationAxis());

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
