#ifndef UNIVERSE_H
#define UNIVERSE_H

#include "CelestialBody.h"
#include "Spacecraft.h"
#include "GroundStation.h"
#include <vector>
#include <memory>

class Universe
{
public:
  Universe();

  // Get all bodies
  const std::vector<std::shared_ptr<CelestialBody>> &getBodies() const { return bodies; }
  const std::vector<std::shared_ptr<Spacecraft>> &getSpacecrafts() const { return spacecraft; }
  const std::vector<std::shared_ptr<GroundStation>> &getGroundStations() const { return groundStations; }

  // Get specific bodies (if needed)
  std::shared_ptr<CelestialBody> getEarth() const { return earth; }
  std::shared_ptr<CelestialBody> getSun() const { return sun; }
  std::shared_ptr<CelestialBody> getMoon() const { return moon; }

  void initializeEarth();
  void initializeSun();
  void initializeMoon();

  void addSpacecraftWithOrbit(
      const Orbit &orbit,
      const std::string &name = "");
  void addGroundStation(const std::string name, double latitude, double longitude);

  // Update physics with sub-stepping for stability
  void update(double deltaTime, double maxPhysicsStep = 0.1);

  // Get position of any body by pointer (for camera tracking)
  glm::dvec3 getObjectPosition(void *object) const;

private:
  // Update Sun's position kinematically (Earth's orbit inverted)
  void updateSunPosition();

  std::vector<std::shared_ptr<CelestialBody>> bodies;
  std::vector<std::shared_ptr<Spacecraft>> spacecraft;
  std::vector<std::shared_ptr<GroundStation>> groundStations;
  std::shared_ptr<CelestialBody> earth;
  std::shared_ptr<CelestialBody> sun;
  std::shared_ptr<CelestialBody> moon;

  // Simulation time tracking for Sun's kinematic orbit
  double simulationTime;
};

#endif // UNIVERSE_H
