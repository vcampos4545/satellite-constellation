#ifndef SATELLITE_H
#define SATELLITE_H

#include <glm/glm.hpp>
#include <vector>

class Satellite
{
public:
  Satellite(const glm::dvec3 &position, const glm::dvec3 &velocity, const glm::vec3 &color, int planeId = 0, int indexInPlane = 0);

  // Update physics
  void update(double deltaTime, const glm::dvec3 &earthCenter, double earthMass, const glm::dvec3 &sunPosition);

  // Calculate complete orbital path (full orbit prediction)
  void calculateFullOrbit(const glm::dvec3 &earthCenter, double earthMass, const glm::dvec3 &sunPosition, int numPoints = 100);

  // Calculate footprint circle on Earth's surface
  void calculateFootprint(const glm::dvec3 &earthCenter, int numPoints = 100);

  // Getters
  glm::dvec3 getPosition() const { return position; }
  glm::dvec3 getVelocity() const { return velocity; }
  glm::vec3 getColor() const { return color; }
  const std::vector<glm::dvec3> &getOrbitPath() const { return orbitPath; }
  const std::vector<glm::dvec3> &getFootprintCircle() const { return footprintCircle; }
  int getPlaneId() const { return planeId; }
  int getIndexInPlane() const { return indexInPlane; }
  bool shouldDrawOrbit() const
  {
    // For Molniya satellites (planeId -2), draw all orbits since each is in a different plane
    // For other constellations, only draw orbit for first satellite in each plane
    return (planeId == -2) || (indexInPlane == 0);
  }

  // Setters for physical properties
  void setMass(double m) { mass = m; }
  void setDragCoefficient(double cd) { dragCoefficient = cd; }
  void setCrossSectionalArea(double area) { crossSectionalArea = area; }
  void setReflectivity(double cr) { reflectivity = cr; }

  // Clear and rebuild orbit path
  void clearOrbitPath() { orbitPath.clear(); }
  void addToOrbitPath(const glm::dvec3 &pos) { orbitPath.push_back(pos); }

private:
  // Helper function to calculate total acceleration
  glm::dvec3 calculateAcceleration(const glm::dvec3 &pos, const glm::dvec3 &vel, const glm::dvec3 &earthCenter, double earthMass, const glm::dvec3 &sunPos) const;

  glm::dvec3 position; // Position in meters (x, y, z)
  glm::dvec3 velocity; // Velocity in meters/second
  glm::dvec3 attitude; // Normalized Attitude
  glm::vec3 color;     // RGB color for rendering

  // Physical properties
  double mass = 260.0;                  // Satellite mass in kg (typical for Starlink)
  double dragCoefficient = 2.2;         // Drag coefficient (dimensionless, ~2.2 for satellites)
  double crossSectionalArea = 10.0;     // Cross-sectional area in m^2
  double reflectivity = 1.3;            // Reflectivity coefficient (1.0 = absorbing, 2.0 = perfect mirror)

  std::vector<glm::dvec3> footprintCircle; // Positions for drawing footprint circle
  std::vector<glm::dvec3> orbitPath;       // Historical positions for drawing orbit
  int orbitPathMaxSize = 1000;             // Maximum number of points to store

  int planeId;      // Orbital plane identifier
  int indexInPlane; // Index of this satellite within its plane
};

#endif // SATELLITE_H
