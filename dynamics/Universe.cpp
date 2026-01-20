#include "Universe.h"
#include "Constants.h"
#include "Orbit.h"
#include "MathUtils.h"
#include "GroundStation.h"
#include "GroundStationData.h"
#include "SpacecraftEnvironment.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <memory>
#include <glm/gtc/matrix_transform.hpp>

Universe::Universe()
    : simulationTime(0.0)
{
  // Initialize celestial bodies
  initializeEarth();
  initializeSun();
  initializeMoon();
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
  earth->setTextureName("earth"); // Use earth texture for rendering
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
  sun->setTextureName("sun");      // Use sun texture for rendering
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

  // Velocity must be perpendicular to position AND in the inclined orbital plane
  // Position is at (periapsis*cos(θ), 0, periapsis*sin(θ)) where θ = totalInclination
  // Perpendicular velocity in the orbital plane: (-sin(θ), cos(θ), 0) rotated by inclination
  // For orbit in XZ plane inclined from XY, velocity at periapsis points mainly in Y
  // but with components to keep it in the inclined plane
  glm::dvec3 moonVel(
      0.0,          // X component (retrograde due to inclination)
      moonVelocity, // Y component (main orbital motion)
      0.0);         // Z component (stays in orbital plane)

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
  moon->setTextureName("moon");    // Use moon texture for rendering
  bodies.push_back(moon);
}

std::shared_ptr<Spacecraft> Universe::addSpacecraft(
    const Orbit &orbit,
    const std::string &name)
{
  // Create spacecraft from orbit (position/velocity computed internally)
  auto sc = std::make_shared<Spacecraft>(orbit, name);
  spacecraft.push_back(sc);
  return sc;
}

FlightSoftware *Universe::createFlightSoftware(std::shared_ptr<Spacecraft> spacecraft)
{
  // Create flight software manager for this spacecraft
  auto fsw = std::make_unique<FlightSoftware>(spacecraft.get());
  FlightSoftware *ptr = fsw.get();
  flightSoftware.push_back(std::move(fsw));
  return ptr;
}

FlightSoftware *Universe::getFlightSoftware(Spacecraft *spacecraft)
{
  for (auto &fsw : flightSoftware)
  {
    if (fsw->getSpacecraft() == spacecraft)
    {
      return fsw.get();
    }
  }
  return nullptr;
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

    // ========== UPDATE SIMULATION TIME ==========
    simulationTime += stepTime;

    // ========== UPDATE SUN POSITION KINEMATICALLY ==========
    // Sun moves along Earth's orbit (inverted) - no physics simulation
    // This keeps Earth at (0,0,0) while maintaining correct relative motion
    updateSunPosition();

    // ========== UPDATE ALL CELESTIAL BODIES ==========
    // Each celestial body updates its rotation and (if physics enabled) orbital position
    for (auto &body : bodies)
    {
      body->update(stepTime, bodies);
    }

    // ========== UPDATE ALL SPACECRAFT ==========
    for (auto &sc : spacecraft)
    {
      // Update spacecraft physics (handles orbital dynamics and components)
      sc->update(stepTime, earth->getPosition(), earth->getMass(), sun->getPosition(), moon->getPosition());
    }

    remainingTime -= stepTime;
  }

  // ========== UPDATE GROUND STATIONS ==========
  // Ground stations rotate with Earth
  for (auto &groundStation : groundStations)
  {
    groundStation->update(earth->getRotation(), earth->getRotationAxis());
  }

  // ========== UPDATE FLIGHT SOFTWARE ==========
  // Create environment data for flight software
  SpacecraftEnvironment environment;
  environment.earthPosition = earth->getPosition();
  environment.sunPosition = sun->getPosition();
  environment.moonPosition = moon->getPosition();
  environment.groundStations = &groundStations;
  environment.otherSpacecraft = &spacecraft;

  // Update each flight software system with environment data
  for (size_t i = 0; i < flightSoftware.size(); ++i)
  {
    // Set self index to exclude own spacecraft from collision checks
    environment.selfIndex = i;
    flightSoftware[i]->update(deltaTime, environment);
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

  // Check if it's a spacecraft
  for (const auto &sc : spacecraft)
  {
    if (sc.get() == object)
    {
      return sc->getPosition();
    }
  }

  return glm::dvec3(0.0);
}

void Universe::updateSunPosition()
{
  /**
   * Update Sun's position kinematically using Earth's orbital elements (inverted)
   *
   * In reality, Earth orbits the Sun. In our Earth-centered frame, we need the Sun
   * to appear to orbit Earth with the same relative motion. This is achieved by
   * calculating where Earth would be in a heliocentric orbit, then placing the Sun
   * at the negative of that position.
   *
   * Uses Kepler's equations to compute elliptical orbit with:
   * - Semi-major axis: 1 AU (149,597,870,700 m)
   * - Eccentricity: 0.0167 (slightly elliptical)
   * - Orbital period: 365.25 days
   * - Inclination: 23.44° (ecliptic obliquity)
   *
   * This gives us:
   * - Perihelion in January (~147 million km)
   * - Aphelion in July (~152 million km)
   * - Correct seasonal lighting variations
   */

  // ========== EARTH'S ORBITAL ELEMENTS (HELIOCENTRIC) ==========
  const double a = AU;                                       // Semi-major axis: 1 AU
  const double e = 0.0167;                                   // Eccentricity (Earth's orbit is nearly circular)
  const double period = 365.25 * 86400.0;                    // Orbital period: 365.25 days in seconds
  const double obliquity = glm::radians(ECLIPTIC_OBLIQUITY); // Tilt of Earth's axis: 23.44°

  // ========== MEAN MOTION ==========
  // Angular velocity of Earth's orbit (radians per second)
  double n = 2.0 * PI / period;

  // ========== MEAN ANOMALY ==========
  // Angle that increases uniformly with time
  // Start at summer solstice (approximately M ≈ π/2)
  // At t=0: Northern hemisphere summer, Sun appears north of equator
  double M = n * simulationTime + PI / 2.0;

  // ========== SOLVE KEPLER'S EQUATION FOR ECCENTRIC ANOMALY ==========
  // Kepler's equation: M = E - e*sin(E)
  // Solved iteratively using fixed-point iteration (converges quickly for small e)
  double E = M; // Initial guess
  for (int i = 0; i < 10; i++)
  {
    E = M + e * sin(E); // Fixed-point iteration
  }

  // ========== CALCULATE TRUE ANOMALY ==========
  // True anomaly is the actual angle in the orbit from perihelion
  // Formula: tan(ν/2) = sqrt((1+e)/(1-e)) * tan(E/2)
  double nu = 2.0 * atan2(sqrt(1.0 + e) * sin(E / 2.0), sqrt(1.0 - e) * cos(E / 2.0));

  // ========== CALCULATE ORBITAL RADIUS ==========
  // Distance from Sun (heliocentric orbit)
  // Using orbit equation: r = a(1-e²)/(1+e*cos(ν))
  double r = a * (1.0 - e * e) / (1.0 + e * cos(nu));

  // ========== POSITION IN ORBITAL PLANE (ECLIPTIC) ==========
  // Ecliptic plane: plane of Earth's orbit around Sun
  // X-axis points to vernal equinox, Z-axis perpendicular to ecliptic
  double x_ecliptic = r * cos(nu); // X component in ecliptic
  double y_ecliptic = r * sin(nu); // Y component in ecliptic

  // ========== TRANSFORM FROM ECLIPTIC TO EQUATORIAL COORDINATES ==========
  // Equatorial plane is tilted by obliquity (23.44°) relative to ecliptic
  // Rotation around X-axis by obliquity angle
  glm::dvec3 earthPosHeliocentric(
      x_ecliptic,                  // X stays the same
      y_ecliptic * cos(obliquity), // Y component (in equatorial plane)
      y_ecliptic * sin(obliquity)  // Z component (perpendicular to equator)
  );

  // ========== INVERT POSITION FOR GEOCENTRIC FRAME ==========
  // In geocentric (Earth-centered) frame, Sun is at negative of Earth's heliocentric position
  // If Earth is at (+X, +Y, +Z) from Sun, then Sun is at (-X, -Y, -Z) from Earth
  glm::dvec3 sunPosGeocentric = -earthPosHeliocentric;

  // ========== CALCULATE ORBITAL VELOCITY ==========
  // Use vis-viva equation: v² = μ(2/r - 1/a)
  // where μ = G*M_sun for heliocentric orbit
  double mu = G * SUN_MASS;
  double v_mag = sqrt(mu * (2.0 / r - 1.0 / a));

  // Velocity is perpendicular to radius vector in the orbital plane
  // In perifocal coordinates: v_x = -(μ/h)*sin(ν), v_y = (μ/h)*(e+cos(ν))
  // where h = sqrt(μ*a*(1-e²)) is specific angular momentum
  double h = sqrt(mu * a * (1.0 - e * e));
  double vx_ecliptic = -(mu / h) * sin(nu);
  double vy_ecliptic = (mu / h) * (e + cos(nu));

  // Transform velocity to equatorial coordinates
  glm::dvec3 earthVelHeliocentric(
      vx_ecliptic,
      vy_ecliptic * cos(obliquity),
      vy_ecliptic * sin(obliquity));

  // ========== INVERT VELOCITY FOR GEOCENTRIC FRAME ==========
  // Sun's geocentric velocity is negative of Earth's heliocentric velocity
  glm::dvec3 sunVelGeocentric = -earthVelHeliocentric;

  // ========== UPDATE SUN'S POSITION AND VELOCITY ==========
  sun->setPosition(sunPosGeocentric);
  sun->setVelocity(sunVelGeocentric);

  // Note: Sun's physics is disabled (enablePhysics = false)
  // It moves purely kinematically based on this calculation
  // This is the correct approach since Earth's gravity is far too weak to
  // actually make the Sun orbit - we're just maintaining the relative motion
}