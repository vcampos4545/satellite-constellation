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
