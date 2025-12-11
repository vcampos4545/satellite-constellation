#include "MathUtils.h"
#include "Constants.h"
#include <random>

glm::dvec3 latLonToCartesian(double latitudeDeg, double longitudeDeg)
{
  // Convert degrees to radians
  double latitude = glm::radians(latitudeDeg);
  double longitude = glm::radians(longitudeDeg);

  // Convert spherical coordinates to Cartesian (on Earth's surface)
  double x = EARTH_RADIUS * cos(latitude) * sin(longitude);
  double y = EARTH_RADIUS * sin(latitude);
  double z = EARTH_RADIUS * cos(latitude) * cos(longitude);

  return glm::dvec3(x, y, z);
}

glm::dvec3 generateRandomEarthSurfacePosition()
{
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_real_distribution<double> latDist(-90.0, 90.0);   // -90 to +90 degrees
  static std::uniform_real_distribution<double> lonDist(-180.0, 180.0); // -180 to +180 degrees

  double latitude = latDist(gen);
  double longitude = lonDist(gen);

  return latLonToCartesian(latitude, longitude);
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
