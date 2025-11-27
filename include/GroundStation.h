#ifndef GROUND_STATION_H
#define GROUND_STATION_H

#include <glm/glm.hpp>
#include <memory>

class Satellite;

class GroundStation
{
public:
  GroundStation(const glm::dvec3 &position);

  // Getters
  glm::dvec3 getPosition() const { return position; }
  std::shared_ptr<Satellite> getConnectedSatellite() const { return connectedSatellite; }

  // Setters
  void setConnectedSatellite(std::shared_ptr<Satellite> satellite) { connectedSatellite = satellite; }

  // Check if connected to a satellite
  bool isConnected() const { return connectedSatellite != nullptr; }

private:
  glm::dvec3 position;                          // Position on Earth's surface (x, y, z)
  std::shared_ptr<Satellite> connectedSatellite; // Currently connected satellite (nullptr if none)
};

#endif // GROUND_STATION_H
