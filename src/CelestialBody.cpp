#include "CelestialBody.h"

CelestialBody::CelestialBody(const glm::dvec3 &position, double mass, double radius, const glm::vec3 &color)
    : position(position), mass(mass), radius(radius), color(color), rotation(0.0f)
{
}
