#include "CelestialBody.h"

CelestialBody::CelestialBody(const glm::dvec3 &position, double mass, double radius, const glm::vec3 &color, const glm::vec3 &rotationAxis)
    : position(position), mass(mass), radius(radius), color(color), rotation(0.0f), rotationAxis(glm::normalize(rotationAxis))
{
}
