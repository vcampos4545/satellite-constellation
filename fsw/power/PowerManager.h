#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <glm/glm.hpp>

// Forward declaration
class Satellite;

/**
 * Power Manager
 *
 * FSW controller responsible for power management algorithms:
 * - Solar power generation calculations
 * - System power consumption tracking
 * - Battery charge management (charging/discharging logic)
 * - Eclipse detection
 * - Low power mode decisions
 *
 * The Satellite class provides hardware state (battery charge, solar panel area, etc.)
 * and hardware control commands (setBatteryCharge, etc.).
 * PowerManager implements the decision-making and calculations.
 */
class PowerManager
{
public:
  PowerManager() = default;
  ~PowerManager() = default;

  /**
   * Update satellite power system
   * - Check eclipse status
   * - Calculate solar power generation
   * - Calculate system power consumption
   * - Manage battery charging/discharging
   * - Handle low power conditions
   */
  void updatePowerSystem(Satellite *satellite, double deltaTime,
                         const glm::dvec3 &sunPosition,
                         const glm::dvec3 &earthCenter);

  /**
   * Calculate solar power generation from sun illumination
   * Returns power in watts
   */
  double calculateSolarPowerGeneration(Satellite *satellite,
                                       const glm::dvec3 &sunPosition,
                                       const glm::dvec3 &earthCenter);

  /**
   * Calculate total system power consumption
   * Returns power in watts
   */
  double calculateSystemPowerConsumption(Satellite *satellite);

  /**
   * Manage battery charge/discharge based on net power
   * Updates battery charge and handles low power conditions
   */
  void manageBattery(Satellite *satellite, double deltaTime,
                     double powerGeneration, double powerConsumption);

  /**
   * Check if satellite is in Earth's shadow (eclipse)
   * Uses ray-sphere intersection from satellite toward sun
   */
  bool checkEclipse(const glm::dvec3 &satellitePosition,
                    const glm::dvec3 &sunPosition,
                    const glm::dvec3 &earthCenter) const;

  /**
   * Calculate solar flux at satellite position
   * Returns flux in W/m²
   */
  double calculateSolarFlux(const glm::dvec3 &satellitePosition,
                            const glm::dvec3 &sunPosition) const;

private:
  // No internal state needed - all state lives in Satellite hardware
};

#endif // POWER_MANAGER_H
