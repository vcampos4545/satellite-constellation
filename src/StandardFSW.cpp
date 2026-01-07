#include "StandardFSW.h"
#include "Satellite.h"
#include <iostream>

void StandardFSW::execute(Satellite *satellite, double deltaTime)
{
  /**
   * Standard Flight Software Execution
   *
   * This FSW implements typical autonomous satellite operations:
   * 1. ADCS (Attitude control)
   * 2. Power management
   * 3. Station keeping (if enabled)
   *
   * This replicates the behavior of the old runFlightSoftware() method.
   */

  // Get current state from satellite sensors
  glm::dvec3 position = satellite->getPosition();
  glm::dvec3 earthCenter(0.0); // Earth at origin

  // Simple sun position estimate (should be passed from outside in real FSW)
  // For now, we'll get it from the satellite's perspective
  // In a real scenario, FSW would use a sun sensor or ephemeris data

  // ========== POWER MANAGEMENT ==========
  // Update power system (solar generation, battery charging/discharging)
  // Note: This requires sun position, which FSW should compute from ephemeris
  // For now, we pass earthCenter; the satellite will compute sunPos internally
  glm::dvec3 sunPosition(1.496e11, 0.0, 0.0); // Simplified: sun at 1 AU in +X direction
  satellite->updatePowerSystem(deltaTime, sunPosition, earthCenter);

  // ========== ATTITUDE DETERMINATION AND CONTROL ==========
  // Run ADCS control loop (determine attitude, compute control, command actuators)
  satellite->adcsControlLoop(deltaTime, earthCenter, sunPosition);

  // ========== STATION KEEPING ==========
  // If station keeping is enabled, perform orbital maintenance
  if (satellite->getStationKeepingEnabled())
  {
    satellite->performStationKeeping(deltaTime, earthCenter);
  }

  // ========== HOUSEKEEPING ==========
  // Check telemetry limits and generate alerts
  timeSinceLastCheck += deltaTime;
  if (timeSinceLastCheck >= 1.0) // Check every 1 second
  {
    satellite->checkTelemetryLimits();
    timeSinceLastCheck = 0.0;
  }
}
