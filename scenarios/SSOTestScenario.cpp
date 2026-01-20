#include "SSOTestScenario.h"
#include "ScenarioRegistry.h"
#include "Universe.h"
#include "Simulation.h"
#include "Orbit.h"
#include "Spacecraft.h"
#include "ReactionWheel.h"
#include "ADCSController.h"
#include "GroundStationData.h"
#include "Constants.h"
#include <cstdio>

// Auto-register this scenario
REGISTER_SCENARIO(SSOTestScenario);

void SSOTestScenario::setup(Universe &universe)
{
  printf("\033[36m[SSO Test] Setting up scenario...\033[0m\n");

  // Create Sun-Synchronous Orbit
  // Altitude: 700 km
  // Inclination: 98° (sun-synchronous for 700km altitude)
  double altitude = 700e3;                                   // 700 km
  double semiMajorAxis = EARTH_EQUATORIAL_RADIUS + altitude; // ~7078 km
  double eccentricity = 0.001;                               // Nearly circular
  double inclination = 98.0 * PI / 180.0;                    // 98° in radians
  double raan = 0.0;                                         // Right Ascension of Ascending Node
  double argOfPerigee = 0.0;                                 // Argument of perigee
  double trueAnomaly = 0.0;                                  // Start at perigee

  Orbit ssoOrbit{semiMajorAxis, eccentricity, inclination, raan, argOfPerigee, trueAnomaly};

  // Add satellite to universe and get spacecraft pointer
  auto sc = universe.addSpacecraft(ssoOrbit, "SSO-Sat-1");

  // Add 3 reaction wheels (one for each axis)
  auto rwX = sc->addComponent<ReactionWheel>("RW-X", 0.1, 10.0, glm::dvec3(1.0, 0.0, 0.0));
  auto rwY = sc->addComponent<ReactionWheel>("RW-Y", 0.1, 10.0, glm::dvec3(0.0, 1.0, 0.0));
  auto rwZ = sc->addComponent<ReactionWheel>("RW-Z", 0.1, 10.0, glm::dvec3(0.0, 0.0, 1.0));

  // Create flight software for the spacecraft
  auto fsw = universe.createFlightSoftware(sc);

  // Add ADCS module and configure it
  auto adcs = fsw->addModule<ADCSController>();
  adcs->setMode(ADCSController::Mode::NADIR_POINTING);

  // Add ground stations at major cities
  const int numCities = sizeof(MAJOR_CITIES) / sizeof(MAJOR_CITIES[0]);
  for (int i = 0; i < numCities; ++i)
  {
    const City &city = MAJOR_CITIES[i];
    universe.addGroundStation(city.name, city.latitude, city.longitude);
  }

  printf("\033[36m[SSO Test] Created satellite 'SSO-Sat-1'\033[0m\n");
  printf("\033[36m[SSO Test]   Added 3 reaction wheels (0.1 N·m max torque, 10.0 N·m·s max momentum)\033[0m\n");
  printf("\033[36m[SSO Test]   ADCS Mode: Ground Station Pointing\033[0m\n");
  printf("\033[36m[SSO Test]   Altitude: %.1f km\033[0m\n", altitude / 1000.0);
  printf("\033[36m[SSO Test]   Inclination: 98.0°\033[0m\n");
  printf("\033[36m[SSO Test]   Orbital period: ~98.8 minutes\033[0m\n");
  printf("\033[36m[SSO Test] Added %d ground stations\033[0m\n", numCities);

  printf("\033[32m[SSO Test] Setup complete!\033[0m\n");
}

void SSOTestScenario::run(Simulation &sim)
{
  printf("\033[36m[SSO Test] Running simulation...\033[0m\n");
  printf("\033[36m[SSO Test]   Duration: %.1f minutes\033[0m\n", getDuration() / 60.0);
  printf("\033[36m[SSO Test]   Time warp: %.0fx\033[0m\n", getTimeWarp());

  // Set time warp
  sim.setTimeWarp(getTimeWarp());

  // Run the simulation
  // Note: Simulation::run() is the main loop, which will handle GUI and updates
  // For scenario-based control, we'll just call the existing run() method
  sim.run();

  printf("\033[32m[SSO Test] Simulation complete!\033[0m\n");
}

void SSOTestScenario::teardown()
{
  printf("\033[36m[SSO Test] Cleaning up scenario...\033[0m\n");
  // No cleanup needed for this simple scenario
}
