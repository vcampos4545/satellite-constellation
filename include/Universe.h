#ifndef UNIVERSE_H
#define UNIVERSE_H

#include "CelestialBody.h"
#include "Satellite.h"
#include "GroundStation.h"
#include <vector>
#include <memory>

class Universe
{
public:
  Universe();

  // Add celestial bodies
  void addBody(std::shared_ptr<CelestialBody> body);

  // Add satellites
  void addSatellite(std::shared_ptr<Satellite> satellite);

  // Get all bodies
  const std::vector<std::shared_ptr<CelestialBody>> &getBodies() const { return bodies; }
  const std::vector<std::shared_ptr<Satellite>> &getSatellites() const { return satellites; }
  const std::vector<std::shared_ptr<GroundStation>> &getGroundStations() const { return groundStations; }

  // Get specific bodies (if needed)
  std::shared_ptr<CelestialBody> getEarth() const { return earth; }
  std::shared_ptr<CelestialBody> getSun() const { return sun; }
  std::shared_ptr<CelestialBody> getMoon() const { return moon; }
  const std::vector<glm::dvec3> &getMoonOrbitPath() const { return moonOrbitPath; }

  // Initialize with Earth and Sun
  void initializeEarthSunAndMoon();

  // Initialize GPS Constellation
  void addGPSConstellation();

  // Initialize a GEO satellite
  void addGEOSatellite();

  // Initialize a Starlink-like LEO constellation
  void addStarlinkConstellation(int numPlanes = 6, int satellitesPerPlane = 10);

  // Initialize a Molniya constellation (highly elliptical orbit for high latitude coverage)
  void addMolniyaConstellation(int numSatellites = 3);

  // Initialize ground stations for power reception at major cities
  void addGroundStations();

  // Update physics with sub-stepping for stability
  void update(double deltaTime, double maxPhysicsStep = 0.1);

private:
  // Helper method to create a satellite with orbit and footprint calculations
  std::shared_ptr<Satellite> createSatelliteWithOrbit(
      const glm::dvec3 &position,
      const glm::dvec3 &velocity,
      const glm::vec3 &color,
      int planeId,
      int indexInPlane);

  std::vector<std::shared_ptr<CelestialBody>> bodies;
  std::vector<std::shared_ptr<Satellite>> satellites;
  std::vector<std::shared_ptr<GroundStation>> groundStations;
  std::shared_ptr<CelestialBody> earth;
  std::shared_ptr<CelestialBody> sun;
  std::shared_ptr<CelestialBody> moon;
  double moonOrbitAngle;                 // Current angle of moon's orbit around Earth
  std::vector<glm::dvec3> moonOrbitPath; // Visualization of moon's orbit path
};

#endif // UNIVERSE_H
