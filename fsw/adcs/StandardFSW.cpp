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
   */

  // ========== SENSOR READS ==========
  const IMU &imu = satellite->getIMU();
  glm::dvec3 measuredAngularVelocity = imu.getLastMeasurement(); // Noisy IMU data

  // Get state estimates (for now, still using truth - would be replaced by nav filter)
  glm::dvec3 position = satellite->getPosition(); // TODO: Replace with GPS or orbit propagation
  glm::dvec3 earthCenter(0.0);                    // Earth at origin

  // Sun position from ephemeris (simplified - real FSW would compute from time)
  glm::dvec3 sunPosition(1.496e11, 0.0, 0.0); // 1 AU in +X direction

  // ========== ATTITUDE DETERMINATION AND CONTROL ==========
  // ADCS uses sensor data (IMU measurements) for control
  adcsController.executeControlLoop(satellite, deltaTime, earthCenter, sunPosition);

  // Future FSW tasks:
  // - Navigation filter (Kalman filter) to estimate attitude/orbit from sensors
  // - Thermal management
  // - Communications scheduling
  // - Payload operations
}
