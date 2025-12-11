#include "Universe.h"
#include "Constants.h"
#include "Config.h"
#include "MathUtils.h"
#include "GroundStation.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <glm/gtc/matrix_transform.hpp>

Universe::Universe()
{
  /* CUSTOMIZE SIMULATION HERE */

  initializeEarthSunAndMoon();

  addGPSConstellation();

  addGEOConstellation();

  // Add Starlink-like LEO constellation
  addStarlinkConstellation(8, 4); // 2 orbital planes, 2 satellites per plane = 4 satellites

  addReflectConstellation();

  // Add Molniya constellation (highly elliptical orbit for high latitude coverage)
  addMolniyaConstellation(3); // 3 satellites for continuous coverage

  // Add ground stations for power reception at major cities
  addGroundStations();
}

void Universe::addBody(std::shared_ptr<CelestialBody> body)
{
  bodies.push_back(body);
}

std::shared_ptr<Satellite> Universe::createSatelliteWithOrbit(
    const Orbit &orbit,
    const glm::vec3 &color,
    int planeId,
    int indexInPlane,
    const std::string &name)
{
  // Convert orbital elements to Cartesian position and velocity
  glm::dvec3 position, velocity;
  orbit.toCartesian(position, velocity, G * EARTH_MASS);

  // Create satellite with computed position/velocity
  auto satellite = std::make_shared<Satellite>(orbit, position, velocity, color, planeId, indexInPlane, name);

  // Enable ADCS with reaction wheels
  satellite->enableReactionWheels(true);
  satellite->setControlMode(AttitudeControlMode::TARGET_TRACKING);
  satellite->setControlAlgorithm(ControlAlgorithm::PID);

  // Calculate and visualize the orbital path
  satellite->calculateOrbitPath(100);

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
  // GPS constellation: 24 satellites in 6 planes at 55° inclination, ~20,200 km altitude
  const double semiMajorAxis = EARTH_RADIUS + GPS_ALTITUDE;
  const double inclination = glm::radians(55.0);
  const int numPlanes = 6;
  const int satellitesPerPlane = 4;

  for (int plane = 0; plane < numPlanes; ++plane)
  {
    double raan = (2.0 * PI * plane) / numPlanes;

    for (int sat = 0; sat < satellitesPerPlane; ++sat)
    {
      double trueAnomaly = (2.0 * PI * sat) / satellitesPerPlane;

      // Create orbit object (circular orbit: e=0, w=0)
      Orbit orbit{semiMajorAxis, 0.0, inclination, raan, 0.0, trueAnomaly};

      // Generate satellite name
      std::string satName = "GPS-" + std::to_string(plane * satellitesPerPlane + sat + 1);

      // Create satellite (position/velocity computed from orbit)
      auto satellite = createSatelliteWithOrbit(
          orbit,
          glm::vec3(0.3f, 0.9f, 0.0f), // Yellow-green color
          plane,
          sat,
          satName);

      satellites.push_back(satellite);
    }
  }
}

void Universe::addGEOConstellation(int numSatellites)
{
  const double semiMajorAxis = EARTH_RADIUS + GEO_ALTITUDE;

  for (int sat = 0; sat < numSatellites; ++sat)
  {
    double trueAnomaly = (2.0 * PI * sat) / numSatellites;

    Orbit orbit{semiMajorAxis, 0.0, 0.0, 0.0, 0.0, trueAnomaly};
    std::string satName = "GEO" + std::to_string(sat);

    auto satellite = createSatelliteWithOrbit(
        orbit,
        glm::vec3(1.0f, 0.0f, 0.0f), // red
        -4,
        sat,
        satName);

    satellites.push_back(satellite);
  }
}

void Universe::addStarlinkConstellation(int numPlanes, int satellitesPerPlane)
{
  // Starlink constellation: ~550 km altitude, 53° inclination (optimized for mid-latitudes)
  const double semiMajorAxis = EARTH_RADIUS + LEO_ALTITUDE;
  const double inclination = glm::radians(53.0);

  for (int plane = 0; plane < numPlanes; ++plane)
  {
    double raan = (2.0 * PI * plane) / numPlanes;

    for (int sat = 0; sat < satellitesPerPlane; ++sat)
    {
      double trueAnomaly = (2.0 * PI * sat) / satellitesPerPlane;

      // Create orbit object (circular orbit: e=0, w=0)
      Orbit orbit{semiMajorAxis, 0.0, inclination, raan, 0.0, trueAnomaly};

      // Generate satellite name
      std::string satName = "Starlink-" + std::to_string(plane * satellitesPerPlane + sat + 1);

      // Create satellite (position/velocity computed from orbit)
      auto satellite = createSatelliteWithOrbit(
          orbit,
          glm::vec3(0.3f, 0.9f, 1.0f), // Bright cyan color
          plane,
          sat,
          satName);

      // Enable station keeping (Starlink uses Hall effect thrusters)
      satellite->enableStationKeeping(true, 550e3); // Maintain 550 km altitude

      satellites.push_back(satellite);
    }
  }
}

void Universe::addReflectConstellation(int numSatellites)
{
  const double semiMajorAxis = EARTH_RADIUS + 600e3;
  const double inclination = glm::radians(98.0); // SSO

  for (int sat = 0; sat < numSatellites; ++sat)
  {
    double trueAnomaly = (2.0 * PI * sat) / numSatellites;

    Orbit orbit{semiMajorAxis, 0.0, inclination, 0.0, 0.0, trueAnomaly};
    std::string satName = "Reflect" + std::to_string(sat);

    auto satellite = createSatelliteWithOrbit(
        orbit,
        glm::vec3(1.0f, 0.0f, 0.0f), // red
        -3,
        sat,
        satName);

    satellites.push_back(satellite);
  }
}

void Universe::addMolniyaConstellation(int numSatellites)
{
  // Molniya constellation: Highly elliptical orbit for high-latitude coverage
  // 3 satellites with 8-hour phase separation gives continuous coverage (period ~12 hours)
  const double semiMajorAxis = MOLNIYA_SEMI_MAJOR_AXIS;
  const double eccentricity = MOLNIYA_ECCENTRICITY;
  const double inclination = glm::radians(MOLNIYA_INCLINATION); // 63.4° - critical inclination
  const double argOfPerigee = glm::radians(270.0);              // Apogee over northern hemisphere

  for (int sat = 0; sat < numSatellites; ++sat)
  {
    double raan = (2.0 * PI * sat) / numSatellites;
    double trueAnomaly = glm::radians(90.0); // Start at apogee (maximum dwell time)

    // Create orbit object (elliptical orbit with e=0.72)
    Orbit orbit{semiMajorAxis, eccentricity, inclination, raan, argOfPerigee, trueAnomaly};

    // Generate satellite name
    std::string satName = "Molniya-" + std::to_string(sat + 1);

    // Create satellite (position/velocity computed from orbit)
    auto satellite = createSatelliteWithOrbit(
        orbit,
        glm::vec3(1.0f, 0.5f, 0.2f), // Orange/red color
        -2,                          // planeId (-2 for Molniya - each in different plane)
        sat,
        satName);

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
