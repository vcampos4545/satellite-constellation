#ifndef GROUND_STATION_H
#define GROUND_STATION_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <chrono>
class GroundStation
{
public:
  GroundStation(const std::string &name, double latitude, double longitude);

  // Getters
  std::string getName() const { return name; }
  glm::dvec3 getPosition() const { return position; }

  // Update position based on Earth's rotation
  void update(double earthRotation, const glm::vec3 &rotationAxis);

private:
  std::string name;    // City name
  double latitude;     // Latitude in degrees
  double longitude;    // Longitude in degrees
  glm::dvec3 position; // Current world position (recalculated each frame)
};

#endif // GROUND_STATION_H
