#include "StarlinkScenario.h"
#include "ScenarioRegistry.h"
#include "Universe.h"
#include "Simulation.h"
#include "Orbit.h"
#include "Spacecraft.h"
#include "Constants.h"
#include "GroundStationData.h"
#include <cstdio>
#include <string>

// Auto-register this scenario
REGISTER_SCENARIO(StarlinkScenario);

void StarlinkScenario::setup(Universe &universe)
{
  printf("\033[36m[Starlink] Setting up scenario...\033[0m\n");

  // Starlink constellation: ~550 km altitude, 53° inclination
  const double LEO_ALTITUDE = 550e3;
  const double semiMajorAxis = EARTH_EQUATORIAL_RADIUS + LEO_ALTITUDE;
  const double inclination = 53.0 * PI / 180.0;
  const int numPlanes = 8;
  const int satellitesPerPlane = 4;

  for (int plane = 0; plane < numPlanes; ++plane)
  {
    double raan = (2.0 * PI * plane) / numPlanes;

    for (int sat = 0; sat < satellitesPerPlane; ++sat)
    {
      double trueAnomaly = (2.0 * PI * sat) / satellitesPerPlane;

      // Create orbit object (circular orbit: e=0, w=0)
      Orbit orbit{semiMajorAxis, 0.0, inclination, raan, 0.0, trueAnomaly};

      // Generate satellite name
      std::string satName = "Starlink-" + std::to_string(plane * satellitesPerPlane + sat + 1);

      // Create satellite
      universe.addSpacecraftWithOrbit(
          orbit,
          satName);
    }
  }

  // Add ground stations at major cities
  const int numCities = sizeof(MAJOR_CITIES) / sizeof(MAJOR_CITIES[0]);
  for (int i = 0; i < numCities; ++i)
  {
    const City &city = MAJOR_CITIES[i];
    universe.addGroundStation(city.name, city.latitude, city.longitude);
  }

  printf("\033[36m[Starlink] Created 32 satellites in 8 orbital planes\033[0m\n");
  printf("\033[36m[Starlink]   Altitude: 550 km (LEO)\033[0m\n");
  printf("\033[36m[Starlink]   Inclination: 53°\033[0m\n");
  printf("\033[36m[Starlink]   Orbital period: ~95 minutes\033[0m\n");
  printf("\033[36m[Starlink] Added %d ground stations\033[0m\n", numCities);
  printf("\033[32m[Starlink] Setup complete!\033[0m\n");
}

void StarlinkScenario::run(Simulation &sim)
{
  printf("\033[36m[Starlink] Running simulation...\033[0m\n");
  printf("\033[36m[Starlink]   Duration: %.1f hours\033[0m\n", getDuration() / 3600.0);
  printf("\033[36m[Starlink]   Time warp: %.0fx\033[0m\n", getTimeWarp());

  sim.setTimeWarp(getTimeWarp());
  sim.run();

  printf("\033[32m[Starlink] Simulation complete!\033[0m\n");
}

void StarlinkScenario::teardown()
{
  printf("\033[36m[Starlink] Cleaning up scenario...\033[0m\n");
}
