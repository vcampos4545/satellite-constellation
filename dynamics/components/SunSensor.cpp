#include "SunSensor.h"
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>

SunSensor::SunSensor()
    : Sensor(),
      fieldOfView(120.0),        // 120 degree full cone (±60 degrees)
      angularAccuracy(2.0),      // 2 degree 1-sigma (coarse sun sensor)
      boresightAxis(1.0, 0.0, 0.0), // +X axis default
      measuredSunVector(0.0),
      trueSunVector(0.0),
      sunVisible(false),
      inEclipse(false),
      sunIntensity(1.0),
      sunAngle(0.0),
      calculateEclipse(true),
      rng(std::random_device{}()),
      noiseDist(0.0, 1.0)
{
}

SunSensor::SunSensor(const std::string &name)
    : Sensor(name),
      fieldOfView(120.0),        // 120 degree full cone (±60 degrees)
      angularAccuracy(2.0),      // 2 degree 1-sigma (coarse sun sensor)
      boresightAxis(1.0, 0.0, 0.0), // +X axis default
      measuredSunVector(0.0),
      trueSunVector(0.0),
      sunVisible(false),
      inEclipse(false),
      sunIntensity(1.0),
      sunAngle(0.0),
      calculateEclipse(true),
      rng(std::random_device{}()),
      noiseDist(0.0, 1.0)
{
}

SunSensor::SunSensor(Type type, const std::string &name)
    : Sensor(name),
      boresightAxis(1.0, 0.0, 0.0),
      measuredSunVector(0.0),
      trueSunVector(0.0),
      sunVisible(false),
      inEclipse(false),
      sunIntensity(1.0),
      sunAngle(0.0),
      calculateEclipse(true),
      rng(std::random_device{}()),
      noiseDist(0.0, 1.0)
{
  // Set characteristics based on sensor type
  switch (type)
  {
  case Type::COARSE:
    fieldOfView = 120.0;     // Wide FOV for sun acquisition
    angularAccuracy = 2.0;   // ~2 degree accuracy
    break;

  case Type::FINE:
    fieldOfView = 60.0;      // Narrow FOV for precision
    angularAccuracy = 0.05;  // ~0.05 degree accuracy
    break;

  case Type::DIGITAL:
    fieldOfView = 90.0;      // Moderate FOV
    angularAccuracy = 0.3;   // ~0.3 degree accuracy
    break;
  }
}

void SunSensor::measure(
    const glm::dvec3 &sunPositionECI,
    const glm::dvec3 &spacecraftPositionECI,
    const glm::dquat &attitude,
    const glm::dvec3 &earthPositionECI,
    double earthRadius)
{
  // Reset state
  sunVisible = false;
  sunIntensity = 1.0;

  // Calculate sun direction in ECI frame
  glm::dvec3 sunDirECI = glm::normalize(sunPositionECI - spacecraftPositionECI);

  // Transform sun direction from ECI to body frame
  // attitude is body-to-ECI, so we need the inverse (ECI-to-body)
  glm::dmat3 R_eci_to_body = glm::mat3_cast(glm::inverse(attitude));
  trueSunVector = glm::normalize(R_eci_to_body * sunDirECI);

  // Calculate angle between sun and sensor boresight
  double cosAngle = glm::dot(trueSunVector, boresightAxis);
  cosAngle = glm::clamp(cosAngle, -1.0, 1.0); // Clamp for numerical stability
  sunAngle = glm::degrees(std::acos(cosAngle));

  // Check if sun is within field of view
  double halfFOV = fieldOfView / 2.0;
  if (sunAngle > halfFOV)
  {
    // Sun outside FOV
    sunVisible = false;
    measuredSunVector = glm::dvec3(0.0);
    return;
  }

  // Check for eclipse if enabled
  if (calculateEclipse)
  {
    inEclipse = checkEclipse(sunPositionECI, spacecraftPositionECI, earthPositionECI, earthRadius);
    if (inEclipse)
    {
      sunVisible = false;
      sunIntensity = 0.0;
      measuredSunVector = glm::dvec3(0.0);
      return;
    }
  }

  // Sun is visible - add measurement noise
  sunVisible = true;
  measuredSunVector = addAngularNoise(trueSunVector);
}

bool SunSensor::checkEclipse(
    const glm::dvec3 &sunPos,
    const glm::dvec3 &scPos,
    const glm::dvec3 &earthPos,
    double earthRadius)
{
  /**
   * Cylindrical shadow model (simplified umbra)
   *
   * Check if spacecraft is in Earth's shadow by:
   * 1. Computing vector from Earth to spacecraft
   * 2. Computing vector from Earth to Sun
   * 3. Checking if spacecraft is behind Earth (relative to Sun)
   * 4. Checking if spacecraft is within Earth's shadow cylinder
   */

  // Vector from Earth to spacecraft
  glm::dvec3 earthToSC = scPos - earthPos;
  double scDistance = glm::length(earthToSC);

  // Vector from Earth to Sun
  glm::dvec3 earthToSun = sunPos - earthPos;
  glm::dvec3 sunDir = glm::normalize(earthToSun);

  // Project spacecraft position onto sun direction
  double projection = glm::dot(earthToSC, sunDir);

  // If projection is positive, spacecraft is on sun side of Earth (not eclipsed)
  if (projection >= 0.0)
  {
    return false;
  }

  // Spacecraft is behind Earth - check if within shadow cylinder
  // Distance from spacecraft to Earth-Sun line
  glm::dvec3 projectedPoint = earthPos + projection * sunDir;
  double distanceFromAxis = glm::length(scPos - projectedPoint);

  // Simple cylindrical model: shadow cylinder has radius = Earth radius
  // (This ignores the umbra/penumbra cone geometry for simplicity)
  return distanceFromAxis < earthRadius;
}

glm::dvec3 SunSensor::addAngularNoise(const glm::dvec3 &trueVector)
{
  /**
   * Add angular noise to sun vector measurement
   *
   * Method:
   * 1. Generate random rotation axis perpendicular to true vector
   * 2. Generate random rotation angle from Gaussian distribution
   * 3. Rotate true vector by small angle around random axis
   */

  // Generate noise angle (degrees -> radians)
  double noiseAngleRad = glm::radians(noiseDist(rng) * angularAccuracy);

  // Generate random rotation axis perpendicular to true vector
  // First, find any vector not parallel to trueVector
  glm::dvec3 arbitrary = (std::abs(trueVector.x) < 0.9)
                             ? glm::dvec3(1.0, 0.0, 0.0)
                             : glm::dvec3(0.0, 1.0, 0.0);

  // Create perpendicular vector
  glm::dvec3 perpendicular = glm::normalize(glm::cross(trueVector, arbitrary));

  // Random angle around the true vector to choose rotation axis direction
  double randomAngle = noiseDist(rng) * glm::pi<double>();
  glm::dquat rotateAroundTrue = glm::angleAxis(randomAngle, trueVector);
  glm::dvec3 rotationAxis = rotateAroundTrue * perpendicular;

  // Rotate true vector by noise angle around the random perpendicular axis
  glm::dquat noiseRotation = glm::angleAxis(noiseAngleRad, rotationAxis);
  glm::dvec3 noisyVector = noiseRotation * trueVector;

  return glm::normalize(noisyVector);
}
