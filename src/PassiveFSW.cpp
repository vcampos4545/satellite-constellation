#include "PassiveFSW.h"
#include "Satellite.h"
#include <glm/glm.hpp>

void PassiveFSW::execute(Satellite *satellite, double deltaTime)
{
  /**
   * Passive Flight Software Execution
   *
   * Minimal FSW for simple satellites:
   * - Detumble if spinning too fast
   * - Manage power (charge battery from solar)
   */

  glm::dvec3 earthCenter(0.0);
  glm::dvec3 sunPosition(1.496e11, 0.0, 0.0); // Simplified sun position

  // ========== POWER MANAGEMENT ==========
  // Always keep power system updated (solar charging, battery management)
  satellite->updatePowerSystem(deltaTime, sunPosition, earthCenter);

  // ========== DETUMBLING ==========
  // Only detumble if spinning too fast (saves power)
  double angularVelocityMagnitude = glm::length(satellite->getAngularVelocity());

  if (angularVelocityMagnitude > detumbleThreshold)
  {
    // Switch to detumble mode
    if (satellite->getControlMode() != AttitudeControlMode::DETUMBLE)
    {
      satellite->setControlMode(AttitudeControlMode::DETUMBLE);
    }

    // Run ADCS to reduce angular velocity
    satellite->adcsControlLoop(deltaTime, earthCenter, sunPosition);
  }
  else
  {
    // Not spinning fast - go to passive mode (no active control)
    if (satellite->getControlMode() != AttitudeControlMode::NONE)
    {
      satellite->setControlMode(AttitudeControlMode::NONE);
    }
  }

  // Passive satellites don't perform station keeping - they let their orbit decay naturally
}
