#include "GroundStation.h"
#include "MathUtils.h"
#include "Constants.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <iostream>

GroundStation::GroundStation(const std::string &name, double latitude, double longitude)
    : name(name), latitude(latitude), longitude(longitude)
{
  // Calculate initial position at rotation = 0
  position = latLonToCartesian(latitude, longitude);
}

void GroundStation::update(double earthRotation, const glm::vec3 &rotationAxis)
{
  // Recalculate position from lat/lon (unrotated)
  glm::dvec3 basePosition = latLonToCartesian(latitude, longitude);

  // Apply Earth's rotation around its axis
  glm::dmat4 rotationMatrix = glm::rotate(glm::dmat4(1.0), (double)earthRotation, glm::dvec3(rotationAxis));
  position = glm::dvec3(rotationMatrix * glm::dvec4(basePosition, 1.0));
}