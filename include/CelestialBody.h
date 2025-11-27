#ifndef CELESTIAL_BODY_H
#define CELESTIAL_BODY_H

#include <glm/glm.hpp>

class CelestialBody
{
public:
  CelestialBody(const glm::dvec3 &position, double mass, double radius, const glm::vec3 &color, const glm::vec3 &rotationAxis = glm::vec3(0.0f, 1.0f, 0.0f));

  // Getters
  glm::dvec3 getPosition() const { return position; }
  double getMass() const { return mass; }
  double getRadius() const { return radius; }
  glm::vec3 getColor() const { return color; }
  float getRotation() const { return rotation; }
  glm::vec3 getRotationAxis() const { return rotationAxis; }

  // Setters
  void setPosition(const glm::dvec3 &pos) { position = pos; }
  void setRotation(float rot) { rotation = rot; }
  void setRotationAxis(const glm::vec3 &axis) { rotationAxis = glm::normalize(axis); }
  void rotate(float deltaRotation) { rotation += deltaRotation; }

private:
  glm::dvec3 position; // Position in meters (x, y, z)
  double mass;         // Mass in kilograms
  double radius;       // Radius in meters
  glm::vec3 color;     // RGB color for rendering
  float rotation;      // Rotation angle in radians
  glm::vec3 rotationAxis; // Normalized axis of rotation (default: Y-axis)
};

#endif // CELESTIAL_BODY_H
