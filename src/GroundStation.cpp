#include "GroundStation.h"
#include "MathUtils.h"
#include "Constants.h"
#include <glm/gtc/matrix_transform.hpp>

GroundStation::GroundStation(const std::string &name, double latitude, double longitude)
    : name(name), latitude(latitude), longitude(longitude), connectedSatellite(nullptr)
{
  // Calculate initial position at rotation = 0
  position = latLonToCartesian(latitude, longitude);
}

void GroundStation::updatePosition(double earthRotation, const glm::vec3 &rotationAxis)
{
  // Recalculate position from lat/lon (unrotated)
  glm::dvec3 basePosition = latLonToCartesian(latitude, longitude);

  // Apply Earth's rotation around its axis
  glm::dmat4 rotationMatrix = glm::rotate(glm::dmat4(1.0), (double)earthRotation, glm::dvec3(rotationAxis));
  position = glm::dvec3(rotationMatrix * glm::dvec4(basePosition, 1.0));
}

// Helper function to check if a ground station is visible from a satellite position
// Returns true if the satellite has line-of-sight to the ground station
bool GroundStation::isSatelliteVisible(const glm::dvec3 &satellitePos, const glm::dvec3 &earthCenter)
{
  // Vector from Earth center to ground station (surface normal)
  glm::dvec3 toStation = glm::normalize(position - earthCenter);

  // Vector from ground station to satellite
  glm::dvec3 toSatellite = glm::normalize(satellitePos - position);

  // Calculate angle between surface normal and direction to satellite
  double angleBetween = acos(glm::dot(toStation, toSatellite));

  // Satellite is visible if angle is less than 90° - minimum elevation
  // (i.e., satellite is above the horizon by at least MIN_ELEVATION)
  const double MIN_ELEVATION = glm::radians(5.0);
  const double MAX_ANGLE = PI / 2.0 - MIN_ELEVATION; // 85 degrees

  return angleBetween < MAX_ANGLE;
}
