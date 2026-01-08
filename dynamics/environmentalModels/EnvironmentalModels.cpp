#include "EnvironmentalModels.h"
#include "Constants.h"
#include <cmath>

glm::dvec3 EnvironmentalModels::calculateAtmosphericDrag(
    const glm::dvec3 &pos,
    const glm::dvec3 &vel,
    const glm::dvec3 &earthCenter,
    double mass,
    double crossSectionalArea,
    double dragCoefficient)
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

glm::dvec3 EnvironmentalModels::calculateSolarRadiationPressure(
    const glm::dvec3 &pos,
    const glm::dvec3 &sunPos,
    const glm::dvec3 &earthCenter,
    double mass,
    double reflectiveArea,
    double reflectivityCoeff)
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
  double srpAccelMag = pressure * reflectiveArea * reflectivityCoeff / mass;
  return sunDirection * srpAccelMag;
}
