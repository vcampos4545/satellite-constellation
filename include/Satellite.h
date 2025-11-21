#ifndef SATELLITE_H
#define SATELLITE_H

#include <glm/glm.hpp>
#include <vector>

class Satellite
{
public:
  Satellite(const glm::dvec3 &position, const glm::dvec3 &velocity, const glm::vec3 &color, int planeId = 0, int indexInPlane = 0);

  // Update physics
  void update(double deltaTime, const glm::dvec3 &earthPosition, double earthMass);

  // Calculate complete orbital path (full orbit prediction)
  void calculateFullOrbit(const glm::dvec3 &earthPosition, double earthMass, int numPoints = 100);

  // Getters
  glm::dvec3 getPosition() const { return position; }
  glm::dvec3 getVelocity() const { return velocity; }
  glm::vec3 getColor() const { return color; }
  const std::vector<glm::dvec3> &getOrbitPath() const { return orbitPath; }
  int getPlaneId() const { return planeId; }
  int getIndexInPlane() const { return indexInPlane; }
  bool shouldDrawOrbit() const { return indexInPlane == 0; } // Only draw orbit for first satellite in each plane

  // Clear and rebuild orbit path
  void clearOrbitPath() { orbitPath.clear(); }
  void addToOrbitPath(const glm::dvec3 &pos) { orbitPath.push_back(pos); }

private:
  glm::dvec3 position; // Position in meters (x, y, z)
  glm::dvec3 velocity; // Velocity in meters/second
  glm::vec3 color;     // RGB color for rendering

  std::vector<glm::dvec3> orbitPath; // Historical positions for drawing orbit
  int orbitPathMaxSize = 1000; // Maximum number of points to store

  int planeId;        // Orbital plane identifier
  int indexInPlane;   // Index of this satellite within its plane
};

#endif // SATELLITE_H
