#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <glm/glm.hpp>

// Convert latitude/longitude (in degrees) to Cartesian coordinates on Earth's surface
glm::dvec3 latLonToCartesian(double latitudeDeg, double longitudeDeg);

// Generate a random position on Earth's surface
glm::dvec3 generateRandomEarthSurfacePosition();

#endif // MATH_UTILS_H
