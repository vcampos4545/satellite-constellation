#include "GEOScenario.h"
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
REGISTER_SCENARIO(GEOScenario);

void GEOScenario::setup(Universe &universe)
{
  printf("\033[36m[GEO] Setting up scenario...\033[0m\n");

  // GEO constellation: 3 satellites evenly spaced around equator
  const double GEO_ALTITUDE = 35.786e6;
  const double semiMajorAxis = EARTH_EQUATORIAL_RADIUS + GEO_ALTITUDE;
  const int numSpacecrafts = 3;

  for (int sat = 0; sat < numSpacecrafts; ++sat)
  {
    double trueAnomaly = (2.0 * PI * sat) / numSpacecrafts;

    // Create orbit object (circular, equatorial orbit: e=0, i=0, w=0)
    Orbit orbit{semiMajorAxis, 0.0, 0.0, 0.0, 0.0, trueAnomaly};

    std::string satName = "GEO-" + std::to_string(sat + 1);

    // Create satellite
    universe.addSpacecraft(orbit, satName);
  }

  // Add ground stations at major cities
  const int numCities = sizeof(MAJOR_CITIES) / sizeof(MAJOR_CITIES[0]);
  for (int i = 0; i < numCities; ++i)
  {
    const City &city = MAJOR_CITIES[i];
    universe.addGroundStation(city.name, city.latitude, city.longitude);
  }

  printf("\033[36m[GEO] Created 3 geostationary satellites\033[0m\n");
  printf("\033[36m[GEO]   Altitude: 35,786 km (GEO)\033[0m\n");
  printf("\033[36m[GEO]   Inclination: 0° (equatorial)\033[0m\n");
  printf("\033[36m[GEO]   Orbital period: 24 hours (Earth-synchronous)\033[0m\n");
  printf("\033[36m[GEO] Added %d ground stations\033[0m\n", numCities);
  printf("\033[32m[GEO] Setup complete!\033[0m\n");
}

void GEOScenario::run(Simulation &sim)
{
  printf("\033[36m[GEO] Running simulation...\033[0m\n");
  printf("\033[36m[GEO]   Duration: %.1f hours\033[0m\n", getDuration() / 3600.0);
  printf("\033[36m[GEO]   Time warp: %.0fx\033[0m\n", getTimeWarp());

  sim.setTimeWarp(getTimeWarp());
  sim.run();

  printf("\033[32m[GEO] Simulation complete!\033[0m\n");
}

void GEOScenario::teardown()
{
  printf("\033[36m[GEO] Cleaning up scenario...\033[0m\n");
}
