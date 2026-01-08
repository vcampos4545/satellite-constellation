#ifndef CELESTIAL_BODY_H
#define CELESTIAL_BODY_H

#include <glm/glm.hpp>

class CelestialBody
{
public:
  CelestialBody(const glm::dvec3 &position, double mass, double radius, const glm::vec3 &rotationAxis = glm::vec3(0.0f, 1.0f, 0.0f), const double rotationAngularVelocity = 0.0);

  // Getters
  glm::dvec3 getPosition() const { return position; }
  glm::dvec3 getVelocity() const { return velocity; }
  double getMass() const { return mass; }
  double getRadius() const { return radius; }
  const std::vector<glm::dvec3> &getOrbitPath() const { return orbitPath; }
  float getRotation() const { return rotation; }
  glm::vec3 getRotationAxis() const { return rotationAxis; }
  bool isPhysicsEnabled() const { return enablePhysics; }

  // Setters
  void setPosition(const glm::dvec3 &pos) { position = pos; }
  void setVelocity(const glm::dvec3 &vel) { velocity = vel; }
  void enablePhysicsUpdate(bool enable) { enablePhysics = enable; }

  // Physics update (for bodies with physics enabled, like Moon)
  void update(double deltaTime, const std::vector<std::shared_ptr<CelestialBody>> &allBodies);

private:
  glm::dvec3 position;               // Position in meters (x, y, z)
  glm::dvec3 velocity;               // Velocity in meters/second
  double mass;                       // Mass in kilograms
  double radius;                     // Radius in meters
  std::vector<glm::dvec3> orbitPath; // Historical position trail (actual path traveled) - complete history since simulation start
  int orbitPathSaveInterval = 10;
  int updateIterationCount = 0;
  float rotation;                 // Rotation angle in radians
  glm::vec3 rotationAxis;         // Normalized axis of rotation (default: Y-axis)
  double rotationAngularVelocity; // Angular velocity in rad/s
  bool enablePhysics;             // Whether to update physics (position/velocity)
  void recordPosition()
  {
    // ========== ORBIT PATH HISTORY ==========
    // Increment update iteration counter
    updateIterationCount++;

    // Add current position to historical trail every N iterations
    // This gives smooth trails regardless of time warp speed
    // Saves complete history since simulation start
    if (updateIterationCount % orbitPathSaveInterval == 0)
    {
      orbitPath.push_back(position);
    }
  };
};

#endif // CELESTIAL_BODY_H
