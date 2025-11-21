#ifndef UNIVERSE_H
#define UNIVERSE_H

#include "CelestialBody.h"
#include "Satellite.h"
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

  // Get specific bodies (if needed)
  std::shared_ptr<CelestialBody> getEarth() const { return earth; }
  std::shared_ptr<CelestialBody> getSun() const { return sun; }

  // Initialize with Earth and Sun
  void initializeEarthAndSun();

  // Initialize a GEO satellite
  void addGEOSatellite();

  // Initialize a Starlink-like LEO constellation
  void addStarlinkConstellation(int numPlanes = 6, int satellitesPerPlane = 10);

  // Update physics with sub-stepping for stability
  void update(double deltaTime, double maxPhysicsStep = 0.1);

private:
  std::vector<std::shared_ptr<CelestialBody>> bodies;
  std::vector<std::shared_ptr<Satellite>> satellites;
  std::shared_ptr<CelestialBody> earth;
  std::shared_ptr<CelestialBody> sun;
};

#endif // UNIVERSE_H
