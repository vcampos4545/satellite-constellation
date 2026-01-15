#include "MolniyaScenario.h"
#include "ScenarioRegistry.h"
#include "Universe.h"
#include "Simulation.h"
#include "Orbit.h"
#include "Satellite.h"
#include "Constants.h"
#include "GroundStationData.h"
#include <cstdio>
#include <string>
#include <iostream>

// Auto-register this scenario
REGISTER_SCENARIO(MolniyaScenario);

void MolniyaScenario::setup(Universe &universe)
{
  printf("\033[36m[Molniya] Setting up scenario...\033[0m\n");

  // Molniya orbit parameters
  const double semiMajorAxis = 26.6e6;                  // Semi-major axis (meters from Earth center)
  const double eccentricity = 0.72;                     // Highly elliptical
  const double inclination = 63.4 * PI / 180.0;         // Critical inclination (radians)
  const double argOfPerigee = 270.0 * PI / 180.0;       // Apogee over northern hemisphere
  const int numSatellites = 3;

  for (int sat = 0; sat < numSatellites; ++sat)
  {
    double raan = (2.0 * PI * sat) / numSatellites;
    double trueAnomaly = 90.0 * PI / 180.0; // Start at apogee

    // Create orbit object
    Orbit orbit{semiMajorAxis, eccentricity, inclination, raan, argOfPerigee, trueAnomaly};

    std::string satName = "Molniya-" + std::to_string(sat + 1);

    // Create satellite
    universe.addSatelliteWithOrbit(orbit, -2, sat, satName);
  }

  // Add ground stations at major cities
  const int numCities = sizeof(MAJOR_CITIES) / sizeof(MAJOR_CITIES[0]);
  for (int i = 0; i < numCities; ++i)
  {
    const City &city = MAJOR_CITIES[i];
    universe.addGroundStation(city.name, city.latitude, city.longitude);
  }

  printf("\033[36m[Molniya] Created 3 satellites in Molniya orbits\033[0m\n");
  printf("\033[36m[Molniya]   Semi-major axis: 26,600 km\033[0m\n");
  printf("\033[36m[Molniya]   Eccentricity: 0.72 (highly elliptical)\033[0m\n");
  printf("\033[36m[Molniya]   Inclination: 63.4° (critical angle)\033[0m\n");
  printf("\033[36m[Molniya]   Apogee: ~39,800 km, Perigee: ~500 km\033[0m\n");
  printf("\033[36m[Molniya]   Orbital period: ~12 hours\033[0m\n");
  printf("\033[36m[Molniya] Added %d ground stations\033[0m\n", numCities);
  printf("\033[32m[Molniya] Setup complete!\033[0m\n");
}

void MolniyaScenario::run(Simulation &sim)
{
  printf("\033[36m[Molniya] Running simulation...\033[0m\n");
  printf("\033[36m[Molniya]   Duration: %.1f hours\033[0m\n", getDuration() / 3600.0);
  printf("\033[36m[Molniya]   Time warp: %.0fx\033[0m\n", getTimeWarp());

  sim.setTimeWarp(getTimeWarp());
  sim.run();

  printf("\033[32m[Molniya] Simulation complete!\033[0m\n");
}

void MolniyaScenario::teardown()
{
  printf("\033[36m[Molniya] Cleaning up scenario...\033[0m\n");
}
