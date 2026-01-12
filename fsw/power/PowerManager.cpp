#include "PowerManager.h"
#include "Satellite.h"
#include "Constants.h"
#include <cstdio>

void PowerManager::updatePowerSystem(Satellite *satellite, double deltaTime,
                                     const glm::dvec3 &sunPosition,
                                     const glm::dvec3 &earthCenter)
{
  /**
   * Update satellite power system:
   * 1. Check if in eclipse (Earth's shadow)
   * 2. Calculate solar flux and power generation
   * 3. Calculate power consumption from all systems
   * 4. Update battery charge
   */

  // Calculate power generation (accounts for eclipse internally)
  double powerGeneration = calculateSolarPowerGeneration(satellite, sunPosition, earthCenter);

  // Calculate power consumption from all active systems
  double powerConsumption = calculateSystemPowerConsumption(satellite);

  // Update battery charge based on net power
  manageBattery(satellite, deltaTime, powerGeneration, powerConsumption);

  // Update satellite power telemetry
  satellite->setPowerGeneration(powerGeneration);
  satellite->setPowerConsumption(powerConsumption);
}

double PowerManager::calculateSolarPowerGeneration(Satellite *satellite,
                                                   const glm::dvec3 &sunPosition,
                                                   const glm::dvec3 &earthCenter)
{
  // Check eclipse status
  bool inEclipse = checkEclipse(satellite->getPosition(), sunPosition, earthCenter);
  satellite->setInEclipse(inEclipse);

  // No power generation in eclipse
  if (inEclipse)
  {
    return 0.0;
  }

  // Calculate solar flux at satellite position
  double solarFlux = calculateSolarFlux(satellite->getPosition(), sunPosition);

  // Get sun direction in satellite body frame
  glm::dvec3 sunDirection = glm::normalize(sunPosition - satellite->getPosition());

  /**
   * Calculate power generation from solar panels
   *
   * Power = Solar Flux × Panel Area × Efficiency × cos(angle)
   *
   * The cos(angle) term accounts for the angle between panel normal and sun direction
   * We assume panels can track the sun (body-mounted tracking or articulated panels)
   */

  // Get satellite body Y-axis (assume solar panels are mounted perpendicular to Y-axis)
  glm::dvec3 panelNormal = satellite->getBodyYAxis();

  // Calculate angle between panel normal and sun direction
  double cosAngle = glm::dot(panelNormal, sunDirection);

  // If panels are facing away from sun, no power generation
  if (cosAngle < 0.0)
  {
    cosAngle = 0.0;
  }

  // Get solar panel parameters from satellite hardware
  double solarPanelArea = satellite->getSolarPanelArea();
  double solarPanelEfficiency = satellite->getSolarPanelEfficiency();
  double solarPanelDegradation = satellite->getSolarPanelDegradation();

  // Power = Flux × Area × Efficiency × cos(angle) × degradation
  double power = solarFlux * solarPanelArea * solarPanelEfficiency * cosAngle * solarPanelDegradation;

  return power;
}

double PowerManager::calculateSystemPowerConsumption(Satellite *satellite)
{
  /**
   * Calculate total power consumption from all satellite systems
   *
   * Systems:
   * - Baseline avionics (always on)
   * - Reaction wheels (when ADCS active)
   * - Magnetorquers (when in detumble mode)
   * - CMGs (if equipped and active)
   * - Thrusters (when firing)
   */

  double totalPower = satellite->getBasePowerConsumption(); // Always have baseline consumption

  // Get current control mode
  AttitudeControlMode controlMode = satellite->getControlMode();

  // Reaction wheels consume power when ADCS is active
  if (satellite->hasReactionWheelsAvailable() && controlMode != AttitudeControlMode::NONE)
  {
    // Assume 3 reaction wheels active for 3-axis control
    totalPower += 3.0 * satellite->getReactionWheelPower();
  }

  // Magnetorquers consume power in detumble mode
  if (satellite->hasMagnetorquersAvailable() && controlMode == AttitudeControlMode::DETUMBLE)
  {
    totalPower += satellite->getMagnetorquerPower();
  }

  // CMGs consume power when active
  if (satellite->hasCMGsAvailable() && controlMode != AttitudeControlMode::NONE)
  {
    totalPower += satellite->getNumCMGs() * satellite->getCMGPower();
  }

  // Thrusters consume significant power when firing
  if (satellite->getStationKeepingEnabled() && satellite->hasThrustersAvailable())
  {
    // Thrusters don't fire continuously, but when they do, they use a lot of power
    // For now, we'll add a fraction based on whether station keeping is enabled
    totalPower += satellite->getThrusterPower() * 0.01; // 1% duty cycle average
  }

  return totalPower;
}

void PowerManager::manageBattery(Satellite *satellite, double deltaTime,
                                 double powerGeneration, double powerConsumption)
{
  // Net power = Generation - Consumption
  double netPower = powerGeneration - powerConsumption;

  // Get current battery state
  double batteryCharge = satellite->getBatteryCharge();
  double batteryCapacity = satellite->getBatteryCapacity();
  double chargeEfficiency = satellite->getBatteryChargeEfficiency();
  double dischargeEfficiency = satellite->getBatteryDischargeEfficiency();

  if (netPower > 0.0)
  {
    // Charging battery
    double chargeAdded = netPower * deltaTime / 3600.0 * chargeEfficiency; // Convert W*s to Wh
    batteryCharge += chargeAdded;

    // Clamp to battery capacity
    if (batteryCharge > batteryCapacity)
    {
      batteryCharge = batteryCapacity;
    }
  }
  else
  {
    // Draining battery
    double chargeRemoved = -netPower * deltaTime / 3600.0 / dischargeEfficiency; // Convert W*s to Wh
    batteryCharge -= chargeRemoved;

    // Clamp to minimum charge
    if (batteryCharge < 0.0)
    {
      batteryCharge = 0.0;

      // Low power mode - disable non-essential systems
      if (satellite->getControlMode() != AttitudeControlMode::NONE)
      {
        // Switch to detumble mode to conserve power
        printf("\033[33m%s: Low power - switching to DETUMBLE mode\033[0m\n", satellite->getName().c_str());
        satellite->setControlMode(AttitudeControlMode::DETUMBLE);
      }
    }
  }

  // Update battery charge in hardware
  satellite->setBatteryCharge(batteryCharge);
}

bool PowerManager::checkEclipse(const glm::dvec3 &satellitePosition,
                                const glm::dvec3 &sunPosition,
                                const glm::dvec3 &earthCenter) const
{
  /**
   * Check if satellite is in Earth's shadow (eclipse)
   *
   * Method: Ray-sphere intersection from satellite toward sun
   * If ray intersects Earth before reaching sun, satellite is in eclipse
   */

  glm::dvec3 toSun = sunPosition - satellitePosition;
  double distanceToSun = glm::length(toSun);
  glm::dvec3 sunDirection = toSun / distanceToSun;

  // Vector from satellite to Earth center
  glm::dvec3 toEarth = earthCenter - satellitePosition;

  // Project toEarth onto sun direction to find closest point on ray to Earth center
  double projectionLength = glm::dot(toEarth, sunDirection);

  // If Earth is behind the satellite (relative to sun), no eclipse
  if (projectionLength < 0.0)
  {
    return false;
  }

  // If closest approach is beyond the sun, no eclipse
  if (projectionLength > distanceToSun)
  {
    return false;
  }

  // Calculate closest approach distance to Earth center
  glm::dvec3 closestPoint = satellitePosition + sunDirection * projectionLength;
  double closestDistance = glm::length(closestPoint - earthCenter);

  // If closest distance is less than Earth radius, satellite is in shadow
  return (closestDistance < EARTH_RADIUS);
}

double PowerManager::calculateSolarFlux(const glm::dvec3 &satellitePosition,
                                        const glm::dvec3 &sunPosition) const
{
  /**
   * Calculate solar flux at satellite position
   *
   * Solar constant at 1 AU: 1361 W/m²
   * Flux decreases with inverse square of distance from sun
   */

  const double SOLAR_CONSTANT = 1361.0; // W/m² at 1 AU

  glm::dvec3 toSun = sunPosition - satellitePosition;
  double distanceToSun = glm::length(toSun);

  // Solar flux = Solar constant * (1 AU / distance)²
  double solarFlux = SOLAR_CONSTANT * (AU * AU) / (distanceToSun * distanceToSun);

  return solarFlux;
}
