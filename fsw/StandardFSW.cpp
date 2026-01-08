#include "StandardFSW.h"
#include "Satellite.h"
#include <iostream>

void StandardFSW::execute(Satellite *satellite, double deltaTime)
{
  /**
   * Standard Flight Software Execution
   *
   * This FSW implements typical autonomous satellite operations:
   * 1. Power management
   * 2. ADCS (Attitude Determination and Control)
   * 3. Station keeping (if enabled)
   * 4. Housekeeping and health monitoring
   *
   * Key Principle: FSW uses SENSOR DATA, not truth
   * - IMU for angular velocity (with noise and bias)
   * - Future: Star tracker for attitude, GPS for position, etc.
   */

  // ========== SENSOR READS ==========
  // FSW reads from sensors (NOT truth!)
  const IMU &imu = satellite->getIMU();
  glm::dvec3 measuredAngularVelocity = imu.getLastMeasurement(); // Noisy IMU data

  // Get state estimates (for now, still using truth - would be replaced by nav filter)
  glm::dvec3 position = satellite->getPosition(); // TODO: Replace with GPS or orbit propagation
  glm::dvec3 earthCenter(0.0);                    // Earth at origin

  // Sun position from ephemeris (simplified - real FSW would compute from time)
  glm::dvec3 sunPosition(1.496e11, 0.0, 0.0); // 1 AU in +X direction

  // ========== POWER MANAGEMENT ==========
  // Monitor solar power, battery charge, manage power modes
  satellite->updatePowerSystem(deltaTime, sunPosition, earthCenter);

  // ========== ATTITUDE DETERMINATION AND CONTROL ==========
  // ADCS uses sensor data (IMU measurements) for control
  // Note: Currently adcsControlLoop() uses truth internally - needs refactoring
  // For now, it still works but future improvement: pass IMU data to ADCS
  satellite->adcsControlLoop(deltaTime, earthCenter, sunPosition);

  // ========== STATION KEEPING ==========
  // Orbital maintenance: counteract drag, maintain target altitude
  if (satellite->getStationKeepingEnabled())
  {
    satellite->performStationKeeping(deltaTime, earthCenter);
  }

  // ========== HOUSEKEEPING ==========
  // Monitor health, check limits, generate telemetry alerts
  timeSinceLastCheck += deltaTime;
  if (timeSinceLastCheck >= 1.0) // Check every 1 second
  {
    satellite->checkTelemetryLimits();
    timeSinceLastCheck = 0.0;
  }

  // Future FSW tasks:
  // - Navigation filter (Kalman filter) to estimate attitude/orbit from sensors
  // - Thermal management
  // - Communications scheduling
  // - Payload operations
}
