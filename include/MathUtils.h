#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <glm/glm.hpp>

// Convert latitude/longitude (in degrees) to Cartesian coordinates on Earth's surface
glm::dvec3 latLonToCartesian(double latitudeDeg, double longitudeDeg);

// Ray-sphere intersection test for object picking
// Returns true if ray intersects sphere, and sets distance to intersection point
bool raySphereIntersect(const glm::vec3 &rayOrigin, const glm::vec3 &rayDirection,
                        const glm::vec3 &sphereCenter, float sphereRadius,
                        float &distance);

#endif // MATH_UTILS_H
