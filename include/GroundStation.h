#ifndef GROUND_STATION_H
#define GROUND_STATION_H

#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

class Satellite;

class GroundStation
{
public:
  GroundStation(const std::string &name, double latitude, double longitude);

  // Getters
  std::string getName() const { return name; }
  glm::dvec3 getPosition() const { return position; }
  std::shared_ptr<Satellite> getConnectedSatellite() const { return connectedSatellite; }
  const std::vector<std::shared_ptr<Satellite>> &getVisibleSatellites() const { return visibleSatellites; }

  // Setters
  void setConnectedSatellite(std::shared_ptr<Satellite> satellite) { connectedSatellite = satellite; }
  void addVisibleSatellite(std::shared_ptr<Satellite> satellite) { visibleSatellites.push_back(satellite); }
  void clearVisibleSatellites() { visibleSatellites.clear(); }

  // Update position based on Earth's rotation
  void updatePosition(double earthRotation, const glm::vec3 &rotationAxis);

  // Check if connected to a satellite
  bool isConnected() const { return connectedSatellite != nullptr; }

  bool isSatelliteVisible(const glm::dvec3 &satellitePos, const glm::dvec3 &earthCenter);

private:
  std::string name;                              // City name
  double latitude;                               // Latitude in degrees
  double longitude;                              // Longitude in degrees
  glm::dvec3 position;                           // Current world position (recalculated each frame)
  std::shared_ptr<Satellite> connectedSatellite; // Currently connected satellite (nullptr if none)
  std::vector<std::shared_ptr<Satellite>> visibleSatellites; // List of satellites currently in view
};

#endif // GROUND_STATION_H
