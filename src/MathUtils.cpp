#include "MathUtils.h"
#include "Constants.h"
#include <random>

glm::dvec3 latLonToCartesian(double latitudeDeg, double longitudeDeg)
{
  // Convert degrees to radians
  double latitude = glm::radians(latitudeDeg);
  double longitude = glm::radians(longitudeDeg);

  // Convert spherical coordinates to Cartesian (on Earth's surface)
  // Z-up coordinate system: Z is vertical (north/south)
  // X is vernal equinox (0 degrees longitude)

  double z = EARTH_RADIUS * sin(latitude);
  double x = EARTH_RADIUS * cos(latitude) * cos(longitude);
  double y = EARTH_RADIUS * cos(latitude) * sin(longitude);

  return glm::dvec3(x, y, z);
}

bool raySphereIntersect(const glm::vec3 &rayOrigin, const glm::vec3 &rayDirection,
                        const glm::vec3 &sphereCenter, float sphereRadius,
                        float &distance)
{
  // Vector from ray origin to sphere center
  glm::vec3 oc = rayOrigin - sphereCenter;

  // Quadratic equation coefficients for ray-sphere intersection
  // Ray equation: P(t) = rayOrigin + t * rayDirection
  // Sphere equation: |P - sphereCenter|^2 = sphereRadius^2
  float a = glm::dot(rayDirection, rayDirection);
  float b = 2.0f * glm::dot(oc, rayDirection);
  float c = glm::dot(oc, oc) - sphereRadius * sphereRadius;

  // Discriminant
  float discriminant = b * b - 4 * a * c;

  // No intersection if discriminant is negative
  if (discriminant < 0.0f)
  {
    return false;
  }

  // Calculate nearest intersection point
  float sqrtDiscriminant = sqrt(discriminant);
  float t1 = (-b - sqrtDiscriminant) / (2.0f * a);
  float t2 = (-b + sqrtDiscriminant) / (2.0f * a);

  // Use nearest positive intersection
  if (t1 > 0.0f)
  {
    distance = t1;
    return true;
  }
  else if (t2 > 0.0f)
  {
    distance = t2;
    return true;
  }

  return false; // Both intersections behind ray origin
}
