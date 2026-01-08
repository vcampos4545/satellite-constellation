#include "Universe.h"
#include "Constants.h"
#include "Orbit.h"
#include "MathUtils.h"
#include "GroundStation.h"
#include "GroundStationData.h"
#include "StandardFSW.h"
#include "PassiveFSW.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <memory>
#include <glm/gtc/matrix_transform.hpp>

/* ==================== EXAMPLES =================== */

// Satellite orbit altitudes
const double GEO_ALTITUDE = 35.786e6; // GEO altitude above Earth surface (meters)
const double LEO_ALTITUDE = 550e3;    // Starlink altitude ~550 km
const double GPS_ALTITUDE = 20.2e6;   // Altitude of GPS satellites

// Molniya orbit parameters (highly elliptical orbit for high latitude coverage)
const double MOLNIYA_SEMI_MAJOR_AXIS = 26.6e6; // Semi-major axis (meters from Earth center)
const double MOLNIYA_ECCENTRICITY = 0.72;      // Eccentricity (highly elliptical)
const double MOLNIYA_INCLINATION = 63.4;       // Inclination (degrees) - critical angle
const double MOLNIYA_APOGEE_ALTITUDE = 39.8e6; // Apogee altitude above surface (~40,000 km)
const double MOLNIYA_PERIGEE_ALTITUDE = 500e3; // Perigee altitude above surface (~500 km)

void addGPSConstellation(Universe *universe)
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
      universe->addSatelliteWithOrbit(
          orbit,
          plane,
          sat,
          satName);
    }
  }
}

void addGEOConstellation(Universe *universe, int numSatellites)
{
  const double semiMajorAxis = EARTH_RADIUS + GEO_ALTITUDE;

  for (int sat = 0; sat < numSatellites; ++sat)
  {
    double trueAnomaly = (2.0 * PI * sat) / numSatellites;

    Orbit orbit{semiMajorAxis, 0.0, 0.0, 0.0, 0.0, trueAnomaly};
    std::string satName = "GEO" + std::to_string(sat);

    universe->addSatelliteWithOrbit(
        orbit,
        -4,
        sat,
        satName);
  }
}

void addStarlinkConstellation(Universe *universe, int numPlanes, int satellitesPerPlane)
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
      universe->addSatelliteWithOrbit(
          orbit,
          plane,
          sat,
          satName,
          SatelliteType::STARLINK);
    }
  }
}

void addReflectConstellation(Universe *universe, int numSatellites)
{
  const double semiMajorAxis = EARTH_RADIUS + 600e3;
  const double inclination = glm::radians(98.0); // SSO

  for (int sat = 0; sat < numSatellites; ++sat)
  {
    double trueAnomaly = (2.0 * PI * sat) / numSatellites;

    Orbit orbit{semiMajorAxis, 0.0, inclination, 0.0, 0.0, trueAnomaly};
    std::string satName = "Reflect-" + std::to_string(sat);

    universe->addSatelliteWithOrbit(
        orbit,
        -3,
        sat,
        satName);
  }
}

void addMolniyaConstellation(Universe *universe, int numSatellites)
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
    universe->addSatelliteWithOrbit(
        orbit,
        -2, // planeId (-2 for Molniya - each in different plane)
        sat,
        satName);
  }

  std::cout << "Added Molniya constellation with " << numSatellites << " satellites" << std::endl;
}

void addCities(Universe *universe)
{
  // Add ground stations at major city locations
  const int numCities = sizeof(MAJOR_CITIES) / sizeof(MAJOR_CITIES[0]);

  for (int i = 0; i < numCities; ++i)
  {
    const City &city = MAJOR_CITIES[i];
    universe->addGroundStation(city.name, city.latitude, city.longitude);
  }

  std::cout << "Added " << numCities << " ground stations at major city locations" << std::endl;
}

/* ================== UNIVERSE IMPLEMENTATION =================== */

Universe::Universe()
{
  /* DO NOT CHANGE */
  initializeEarth();
  initializeSun();
  initializeMoon();

  /*=============== CUSTOMIZE SIMULATION HERE ===============*/

  addGPSConstellation(this);
  addGEOConstellation(this, 3);         // 3 GEO satellites
  addStarlinkConstellation(this, 8, 4); // 8 planes, 4 sats per plane
  addReflectConstellation(this, 1);     // 1 SSO satellite
  addMolniyaConstellation(this, 3);     // 3 satellites for continuous coverage
  addCities(this);

  // Add cubesats
  Orbit orbit{EARTH_RADIUS + 700e3, 0.0, 45.0, 0.0, 0.0, 0.0};
  addSatelliteWithOrbit(
      orbit,
      9,
      0,
      "Cubesat1U",
      SatelliteType::CUBESAT_1U);
  addSatelliteWithOrbit(
      orbit,
      9,
      1,
      "Cubesat2U",
      SatelliteType::CUBESAT_2U);

  /*=============== CUSTOMIZE SIMULATION HERE ===============*/
}

void Universe::initializeEarth()
{
  // ========== EARTH ==========
  // Create Earth at origin with spin axis as Z axis (ECI)

  earth = std::make_shared<CelestialBody>(
      glm::dvec3(0.0, 0.0, 0.0), // Position at origin
      EARTH_MASS,
      EARTH_RADIUS,
      glm::vec3(0.0f, 0.0f, 1.0f), // rotate around Z axis
      EARTH_ROTATION_ANGULAR_VELOCITY);
  bodies.push_back(earth);
}

void Universe::initializeSun()
{
  // ========== SUN ==========
  // Position sun for northern hemisphere summer (June 21st)
  // At summer solstice, sun is 23.44° above the ecliptic (which is tilted from equator)
  // Since equatorial plane is XY (horizontal), ecliptic is tilted by obliquity
  // Sun should be above the equatorial plane by +23.44° for northern summer
  double sunElevationAngle = glm::radians(ECLIPTIC_OBLIQUITY);

  // Place sun at 1 AU distance, elevated above equatorial plane for northern summer
  // Sun in +X direction (local noon for observer at 0° longitude), elevated in +Z
  glm::dvec3 sunPosition(
      AU * cos(sunElevationAngle), // X component
      0.0,                         // Y component
      AU * sin(sunElevationAngle)  // Z component
  );

  sun = std::make_shared<CelestialBody>(
      sunPosition,
      SUN_MASS,
      SUN_RADIUS,
      glm::vec3(0.0f, 0.0f, 0.0f), // No rotation axis for simplicity
      0.0);                        // No rotation for simplicity
  bodies.push_back(sun);
}

void Universe::initializeMoon()
{
  // ========== MOON ==========
  // Create Moon with accurate orbital parameters
  // Moon's orbit is inclined 5.145° to the ecliptic
  // Ecliptic itself is tilted 23.44° from Earth's equator
  // Total inclination from equator = 5.145° (to ecliptic) + obliquity effects

  // Start moon at periapsis (closest approach)
  double periapsis = MOON_SEMI_MAJOR_AXIS * (1.0 - MOON_ECCENTRICITY);

  // Moon's orbital plane is inclined to ecliptic by 5.145°
  // For Z-up: Ecliptic is tilted from XY (equatorial) plane by 23.44°
  // We'll place moon in the ecliptic plane with additional 5.145° inclination
  double eclipticTilt = glm::radians(ECLIPTIC_OBLIQUITY);
  double moonInclination = glm::radians(MOON_INCLINATION_TO_ECLIPTIC);
  double totalInclination = eclipticTilt + moonInclination;

  // Initial position: periapsis in the orbital plane
  // Place moon at X direction with inclination applied
  glm::dvec3 moonPosition(
      periapsis * cos(totalInclination), // X component
      0.0,                               // Y component
      periapsis * sin(totalInclination)  // Z component
  );

  // Calculate orbital velocity at periapsis using vis-viva equation
  // v = sqrt(μ * (2/r - 1/a))
  double mu = G * EARTH_MASS;
  double moonVelocity = sqrt(mu * (2.0 / periapsis - 1.0 / MOON_SEMI_MAJOR_AXIS));

  // Velocity perpendicular to position, in the orbital plane (mainly Y direction)
  glm::dvec3 moonVel(
      0.0,
      moonVelocity,
      0.0);

  // Moon's rotation axis is tilted 6.68° from its orbital plane normal
  double moonAxisTilt = glm::radians(totalInclination - MOON_AXIAL_TILT);
  glm::vec3 moonRotationAxis(
      sin(moonAxisTilt), // Small X component due to tilt
      0.0f,              // Y
      cos(moonAxisTilt)  // Mostly vertical (Z)
  );

  moon = std::make_shared<CelestialBody>(
      moonPosition,
      MOON_MASS,
      MOON_RADIUS,
      moonRotationAxis,
      MOON_ROTATION_ANGULAR_VELOCITY);
  moon->setVelocity(moonVel);
  moon->enablePhysicsUpdate(true); // Enable physics simulation for the moon
  bodies.push_back(moon);
}

void Universe::addSatelliteWithOrbit(
    const Orbit &orbit,
    int planeId,
    int indexInPlane,
    const std::string &name,
    SatelliteType type)
{
  // Convert orbital elements to Cartesian position and velocity
  glm::dvec3 position, velocity;
  orbit.toCartesian(position, velocity, G * EARTH_MASS);

  // Create satellite with computed position/velocity
  auto satellite = std::make_shared<Satellite>(orbit, position, velocity, planeId, indexInPlane, name, type);

  satellites.push_back(satellite);
}

void Universe::addGroundStation(const std::string name, double latitude, double longitude)
{
  auto groundStation = std::make_shared<GroundStation>(name, latitude, longitude);
  groundStations.push_back(groundStation);
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
      // Update satellite (handles both ADCS and orbital dynamics internally)
      satellite->update(stepTime, earth->getPosition(), earth->getMass(), sun->getPosition(), moon->getPosition());
    }

    remainingTime -= stepTime;
  }

  // ========== UPDATE GROUND STATIONS ==========
  // Ground stations rotate with Earth
  for (auto &groundStation : groundStations)
  {
    groundStation->update(earth->getRotation(), earth->getRotationAxis());
  }
}

glm::dvec3 Universe::getObjectPosition(void *object) const
{
  if (!object)
    return glm::dvec3(0.0);

  // Check if it's one of the celestial bodies
  for (const auto &body : bodies)
  {
    if (body.get() == object)
    {
      return body->getPosition();
    }
  }

  // Check if it's a satellite
  for (const auto &sat : satellites)
  {
    if (sat.get() == object)
    {
      return sat->getPosition();
    }
  }

  return glm::dvec3(0.0);
}