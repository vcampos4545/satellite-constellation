#include "Satellite.h"
#include "Constants.h"
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>

Satellite::Satellite(const glm::dvec3 &position, const glm::dvec3 &velocity, const glm::vec3 &color, int planeId, int indexInPlane)
    : position(position), velocity(velocity), color(color), planeId(planeId), indexInPlane(indexInPlane), attitude(glm::dvec3(0.0, 0.0, 0.0))
{
}

glm::dvec3 Satellite::calculateGravitationalAcceleration(const glm::dvec3 &pos, const glm::dvec3 &bodyPos, double bodyMass) const
{
  glm::dvec3 toBody = bodyPos - pos;
  double distance = glm::length(toBody);

  if (distance < 1.0)
    return glm::dvec3(0.0); // Avoid division by zero

  glm::dvec3 direction = glm::normalize(toBody);
  double accelMagnitude = G * bodyMass / (distance * distance);
  return direction * accelMagnitude;
}

glm::dvec3 Satellite::calculateDragAcceleration(const glm::dvec3 &pos, const glm::dvec3 &vel, const glm::dvec3 &earthCenter) const
{
  // Calculate altitude above Earth's surface
  double distance = glm::length(pos - earthCenter);
  double altitude = distance - EARTH_RADIUS;

  // Only apply drag below DRAG_ALTITUDE_MAX
  if (altitude <= 0.0 || altitude >= DRAG_ALTITUDE_MAX)
    return glm::dvec3(0.0);

  // Exponential atmosphere model: ρ(h) = ρ₀ * e^(-h/H)
  double atmosphericDensity = RHO_0 * exp(-altitude / H_SCALE);

  // Drag force: F_drag = 0.5 * ρ * v² * Cd * A
  // Drag acceleration: a_drag = F_drag / m
  double speedSquared = glm::dot(vel, vel);

  if (speedSquared <= 0.0)
    return glm::dvec3(0.0);

  double speed = sqrt(speedSquared);
  glm::dvec3 velocityDirection = vel / speed;

  // Drag acts opposite to velocity
  double dragAccelMag = 0.5 * atmosphericDensity * speedSquared * dragCoefficient * crossSectionalArea / mass;
  return -velocityDirection * dragAccelMag;
}

glm::dvec3 Satellite::calculateSolarRadiationPressure(const glm::dvec3 &pos, const glm::dvec3 &sunPos, const glm::dvec3 &earthCenter) const
{
  glm::dvec3 toSun = sunPos - pos;
  double distanceSun = glm::length(toSun);

  if (distanceSun < 1.0)
    return glm::dvec3(0.0);

  glm::dvec3 sunDirection = glm::normalize(toSun);

  // Check if satellite is in Earth's shadow (simple cylindrical shadow model)
  glm::dvec3 satToEarth = earthCenter - pos;
  double projectionOnSunLine = glm::dot(satToEarth, -sunDirection);

  bool inShadow = false;
  if (projectionOnSunLine > 0.0) // Satellite is on night side
  {
    // Distance from satellite to sun-earth line
    glm::dvec3 perpComponent = satToEarth + sunDirection * projectionOnSunLine;
    double perpDistance = glm::length(perpComponent);

    if (perpDistance < EARTH_RADIUS)
      inShadow = true;
  }

  // Only apply SRP if satellite is in sunlight
  if (inShadow)
    return glm::dvec3(0.0);

  // Solar radiation pressure at satellite's distance from sun
  // P = P₀ * (AU / r)²
  double distanceRatio = AU / distanceSun;
  double pressure = SOLAR_PRESSURE * distanceRatio * distanceRatio;

  // SRP force: F = P * A * Cr
  // SRP acceleration: a = F / m = P * A * Cr / m
  double srpAccelMag = pressure * crossSectionalArea * reflectivity / mass;
  return sunDirection * srpAccelMag;
}

glm::dvec3 Satellite::calculateAcceleration(const glm::dvec3 &pos, const glm::dvec3 &vel, const glm::dvec3 &earthCenter, double earthMass, const glm::dvec3 &sunPos, const glm::dvec3 &moonPos) const
{
  // Calculate all acceleration components
  glm::dvec3 gravAccelEarth = calculateGravitationalAcceleration(pos, earthCenter, earthMass);
  glm::dvec3 gravAccelMoon = calculateGravitationalAcceleration(pos, moonPos, MOON_MASS);
  glm::dvec3 gravAccelSun = calculateGravitationalAcceleration(pos, sunPos, SUN_MASS);
  glm::dvec3 dragAccel = calculateDragAcceleration(pos, vel, earthCenter);
  glm::dvec3 srpAccel = calculateSolarRadiationPressure(pos, sunPos, earthCenter);

  // Total acceleration (all forces combined)
  return gravAccelEarth + gravAccelMoon + gravAccelSun + dragAccel + srpAccel;
}

void Satellite::update(double deltaTime, const glm::dvec3 &earthCenter, double earthMass, const glm::dvec3 &sunPosition, const glm::dvec3 &moonPosition)
{
  // RK4 (Runge-Kutta 4th order) integration for better accuracy
  // This is much more stable than Euler integration for orbital mechanics

  // k1 = f(t, y)
  glm::dvec3 k1_vel = calculateAcceleration(position, velocity, earthCenter, earthMass, sunPosition, moonPosition);
  glm::dvec3 k1_pos = velocity;

  // k2 = f(t + dt/2, y + k1*dt/2)
  glm::dvec3 k2_vel = calculateAcceleration(
    position + k1_pos * (deltaTime * 0.5),
    velocity + k1_vel * (deltaTime * 0.5),
    earthCenter, earthMass, sunPosition, moonPosition);
  glm::dvec3 k2_pos = velocity + k1_vel * (deltaTime * 0.5);

  // k3 = f(t + dt/2, y + k2*dt/2)
  glm::dvec3 k3_vel = calculateAcceleration(
    position + k2_pos * (deltaTime * 0.5),
    velocity + k2_vel * (deltaTime * 0.5),
    earthCenter, earthMass, sunPosition, moonPosition);
  glm::dvec3 k3_pos = velocity + k2_vel * (deltaTime * 0.5);

  // k4 = f(t + dt, y + k3*dt)
  glm::dvec3 k4_vel = calculateAcceleration(
    position + k3_pos * deltaTime,
    velocity + k3_vel * deltaTime,
    earthCenter, earthMass, sunPosition, moonPosition);
  glm::dvec3 k4_pos = velocity + k3_vel * deltaTime;

  // Update: y_new = y + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
  velocity += (deltaTime / 6.0) * (k1_vel + 2.0 * k2_vel + 2.0 * k3_vel + k4_vel);
  position += (deltaTime / 6.0) * (k1_pos + 2.0 * k2_pos + 2.0 * k3_pos + k4_pos);

  // Update footprint every frame
  calculateFootprint(earthCenter, 60);
}

void Satellite::calculateFullOrbit(const glm::dvec3 &earthCenter, double earthMass, const glm::dvec3 &sunPosition, const glm::dvec3 &moonPosition, int numPoints)
{
  orbitPath.clear();

  // Calculate orbital period using Kepler's third law
  // Need to determine semi-major axis from current state using vis-viva equation
  double r = glm::length(position - earthCenter);  // Current distance
  double v = glm::length(velocity);                // Current speed
  double mu = G * earthMass;

  // Specific orbital energy: ε = v²/2 - μ/r
  double specificEnergy = (v * v) / 2.0 - mu / r;

  // Semi-major axis from energy: a = -μ/(2ε)
  double semiMajorAxis;
  if (specificEnergy < 0.0)  // Bound orbit (elliptical)
  {
    semiMajorAxis = -mu / (2.0 * specificEnergy);
  }
  else  // Parabolic or hyperbolic - use current radius as approximation
  {
    semiMajorAxis = r;
  }

  // Orbital period: T = 2π√(a³/μ)
  double orbitalPeriod = 2.0 * PI * sqrt((semiMajorAxis * semiMajorAxis * semiMajorAxis) / mu);

  // Simulate one complete orbit
  double timeStep = orbitalPeriod / numPoints;

  // Save current state
  glm::dvec3 savedPos = position;
  glm::dvec3 savedVel = velocity;

  // Simulate forward (numPoints + 1 to ensure that the full circle is connected)
  for (int i = 0; i <= numPoints; ++i)
  {
    orbitPath.push_back(position);

    // Use the same acceleration model as update() for consistency
    glm::dvec3 acceleration = calculateAcceleration(position, velocity, earthCenter, earthMass, sunPosition, moonPosition);

    // Simple Euler integration for orbit path (RK4 would be slower and orbit path is just visual)
    velocity += acceleration * timeStep;
    position += velocity * timeStep;
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
