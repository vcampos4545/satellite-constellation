#ifndef CELESTIAL_BODY_H
#define CELESTIAL_BODY_H

#include <glm/glm.hpp>

class CelestialBody
{
public:
  CelestialBody(const glm::dvec3 &position, double mass, double radius, const glm::vec3 &color);

  // Getters
  glm::dvec3 getPosition() const { return position; }
  double getMass() const { return mass; }
  double getRadius() const { return radius; }
  glm::vec3 getColor() const { return color; }
  float getRotation() const { return rotation; }

  // Setters
  void setPosition(const glm::dvec3 &pos) { position = pos; }
  void setRotation(float rot) { rotation = rot; }
  void rotate(float deltaRotation) { rotation += deltaRotation; }

private:
  glm::dvec3 position; // Position in meters (x, y, z)
  double mass;         // Mass in kilograms
  double radius;       // Radius in meters
  glm::vec3 color;     // RGB color for rendering
  float rotation;      // Rotation angle around Y axis in radians
};

#endif // CELESTIAL_BODY_H
