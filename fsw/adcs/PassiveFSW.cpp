#include "PassiveFSW.h"
#include "Satellite.h"
#include <glm/glm.hpp>

void PassiveFSW::execute(Satellite *satellite, double deltaTime)
{
  /**
   * Passive Flight Software for CubeSats
   *
   * Minimal FSW for power-constrained satellites:
   * 1. Power management (solar charging, battery monitoring)
   * 2. Detumbling only (reduce spin, then go passive to save power)
   *
   * Key Principle: Use SENSOR measurements, not truth
   */

  glm::dvec3 earthCenter(0.0);
  glm::dvec3 sunPosition(1.496e11, 0.0, 0.0); // Simplified sun position

  // ========== SENSOR READS ==========
  // Read IMU to estimate angular velocity (with noise)
  const IMU &imu = satellite->getIMU();
  glm::dvec3 measuredAngularVelocity = imu.getLastMeasurement();

  // ========== POWER MANAGEMENT ==========
  // Always monitor power (critical for survival)
  powerManager.updatePowerSystem(satellite, deltaTime, sunPosition, earthCenter);

  // ========== DETUMBLING ==========
  // Only activate ADCS if spinning too fast (conserve power)
  // Use IMU measurement (noisy) to determine if we're spinning
  double measuredSpin = glm::length(measuredAngularVelocity);

  if (measuredSpin > detumbleThreshold)
  {
    // Spinning too fast - activate detumble mode
    if (satellite->getControlMode() != AttitudeControlMode::DETUMBLE)
    {
      satellite->setControlMode(AttitudeControlMode::DETUMBLE);
    }

    // Run ADCS to reduce angular velocity
    adcsController.executeControlLoop(satellite, deltaTime, earthCenter, sunPosition);
  }
  else
  {
    // Detumbled successfully - go passive to save power
    // CubeSat will slowly tumble again over time, but that's acceptable
    if (satellite->getControlMode() != AttitudeControlMode::NONE)
    {
      satellite->setControlMode(AttitudeControlMode::NONE);
    }
  }

  // Passive satellites don't perform station keeping
  // They accept orbit decay as part of their limited lifetime
}
