#ifndef SOLAR_PANEL_H
#define SOLAR_PANEL_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include "Component.h"

/**
 * Solar Panel Component
 *
 * Represents a single solar panel or panel array on a satellite.
 * Handles:
 * - Power generation based on sun illumination angle
 * - Temperature dynamics (heating from sun, cooling via radiation)
 * - Temperature-dependent efficiency modeling
 * - Body-fixed mounting with configurable orientation
 *
 * Temperature Model:
 * - Heat input: Absorbed solar radiation (depends on sun angle)
 * - Heat output: Stefan-Boltzmann radiation to space
 * - Temperature affects efficiency: typical -0.4%/°C for silicon cells
 */
class SolarPanel : public PowerComponent
{
public:
  /**
   * Constructor
   * @param area Panel area (m²)
   * @param efficiency Base efficiency at reference temperature (0-1, typically 0.28-0.32)
   * @param normalDirection Normal vector in body frame (will be normalized)
   * @param name Panel identifier (e.g., "+X face", "Array 1")
   */
  SolarPanel(double area, double efficiency,
             const glm::dvec3 &normalDirection,
             const std::string &name = "Panel");

  // Component interface overrides
  std::string getTypeName() const override { return "SolarPanel"; }

  // PowerComponent interface overrides
  double getPowerGeneration() const override { return lastPowerOutput; }
  double getPowerConsumption() const override { return 0.0; } // Solar panels only generate

  /**
   * Calculate power generation for current conditions
   * @param sunDirectionInertial Sun direction in inertial frame (normalized)
   * @param satelliteAttitude Satellite attitude quaternion (body to inertial)
   * @param solarFlux Solar flux at satellite position (W/m²)
   * @param inEclipse Whether satellite is in Earth's shadow
   * @return Power generated in Watts
   */
  double calculatePowerGeneration(const glm::dvec3 &sunDirectionInertial,
                                   const glm::dquat &satelliteAttitude,
                                   double solarFlux,
                                   bool inEclipse);

  /**
   * Update thermal state (temperature dynamics)
   * Uses simple lumped-mass thermal model with solar heating and radiative cooling
   * @param deltaTime Time step (seconds)
   * @param sunDirectionInertial Sun direction in inertial frame
   * @param satelliteAttitude Satellite attitude quaternion
   * @param solarFlux Solar flux at satellite position (W/m²)
   * @param inEclipse Whether satellite is in eclipse
   */
  void updateTemperature(double deltaTime,
                         const glm::dvec3 &sunDirectionInertial,
                         const glm::dquat &satelliteAttitude,
                         double solarFlux,
                         bool inEclipse);

  // Getters
  double getArea() const { return area; }
  double getBaseEfficiency() const { return baseEfficiency; }
  double getCurrentEfficiency() const { return currentEfficiency; }
  double getTemperature() const { return temperature; }
  double getDegradationFactor() const { return degradationFactor; }
  glm::dvec3 getNormalDirection() const { return normalDirection; }
  // getName() inherited from Component base class
  double getLastPowerOutput() const { return lastPowerOutput; }
  double getSunIncidenceAngle() const { return sunIncidenceAngle; }

  // Setters for configuration
  void setDegradationFactor(double degradation) { degradationFactor = degradation; }
  void setThermalMass(double mass) { thermalMass = mass; }
  void setEmissivity(double emissivity) { this->emissivity = emissivity; }
  void setAbsorptivity(double absorptivity) { this->absorptivity = absorptivity; }
  void setTemperatureCoefficientEfficiency(double coeff) { tempCoeffEfficiency = coeff; }
  void setReferenceTemperature(double temp) { referenceTemperature = temp; }

private:
  // Physical properties
  double area;                 // Panel area (m²)
  double baseEfficiency;       // Base efficiency at reference temperature (0-1)
  glm::dvec3 normalDirection;  // Panel normal in body frame (normalized)
  // name is inherited from Component base class

  // Degradation (due to aging, radiation damage, etc.)
  double degradationFactor; // Degradation factor (1.0 = new, decreases over time)

  // Thermal properties
  double temperature;       // Current temperature (Kelvin)
  double thermalMass;       // Thermal mass (J/K) - mass × specific heat
  double emissivity;        // Emissivity for thermal radiation (0-1, typical ~0.85)
  double absorptivity;      // Solar absorptivity (0-1, typical ~0.92 for solar cells)

  // Temperature-efficiency relationship
  double tempCoeffEfficiency;  // Efficiency change per degree C (typically -0.004 for Si)
  double referenceTemperature; // Reference temperature for efficiency (Kelvin, typically 298K = 25°C)

  // Current state
  double currentEfficiency; // Current efficiency including temperature effects
  double lastPowerOutput;   // Last calculated power output (W)
  double sunIncidenceAngle; // Last sun incidence angle (radians)

  /**
   * Calculate temperature-dependent efficiency
   * Efficiency decreases with increasing temperature
   */
  void updateEfficiency();

  /**
   * Calculate sun incidence angle on panel
   * @param sunDirectionInertial Sun direction in inertial frame
   * @param satelliteAttitude Satellite attitude quaternion
   * @return Angle between panel normal and sun (radians)
   */
  double calculateIncidenceAngle(const glm::dvec3 &sunDirectionInertial,
                                  const glm::dquat &satelliteAttitude) const;
};

#endif // SOLAR_PANEL_H
