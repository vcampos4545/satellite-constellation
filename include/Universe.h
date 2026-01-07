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

  // Get all bodies
  const std::vector<std::shared_ptr<CelestialBody>> &getBodies() const { return bodies; }
  const std::vector<std::shared_ptr<Satellite>> &getSatellites() const { return satellites; }
  const std::vector<std::shared_ptr<GroundStation>> &getGroundStations() const { return groundStations; }

  // Get specific bodies (if needed)
  std::shared_ptr<CelestialBody> getEarth() const { return earth; }
  std::shared_ptr<CelestialBody> getSun() const { return sun; }
  std::shared_ptr<CelestialBody> getMoon() const { return moon; }

  void initializeEarth();
  void initializeSun();
  void initializeMoon();

  void addGPSConstellation();
  void addGEOConstellation(int numSatellites = 10);
  void addStarlinkConstellation(int numPlanes = 6, int satellitesPerPlane = 10);
  void addReflectConstellation(int numSatellites = 10);
  void addMolniyaConstellation(int numSatellites = 3);

  void addGroundStations();

  // Update physics with sub-stepping for stability
  void update(double deltaTime, double maxPhysicsStep = 0.1);

  // Get position of any body by pointer (for camera tracking)
  glm::dvec3 getObjectPosition(void *object) const;

private:
  // Helper method to create a satellite from orbital elements
  // Automatically converts orbital elements to position/velocity
  std::shared_ptr<Satellite> createSatelliteWithOrbit(
      const Orbit &orbit,
      const glm::vec3 &color,
      int planeId = 0,
      int indexInPlane = 0,
      const std::string &name = "");

  std::vector<std::shared_ptr<CelestialBody>> bodies;
  std::vector<std::shared_ptr<Satellite>> satellites;
  std::vector<std::shared_ptr<GroundStation>> groundStations;
  std::shared_ptr<CelestialBody> earth;
  std::shared_ptr<CelestialBody> sun;
  std::shared_ptr<CelestialBody> moon;
};

#endif // UNIVERSE_H
