#include "SolarPanel.h"
#include "Constants.h"
#include <cmath>

SolarPanel::SolarPanel(double area, double efficiency,
                       const glm::dvec3 &normalDirection,
                       const std::string &name)
    : PowerComponent(name), // Initialize base class
      area(area),
      baseEfficiency(efficiency),
      normalDirection(glm::normalize(normalDirection)),
      degradationFactor(1.0),
      temperature(273.15), // Start at 0°C
      thermalMass(100.0),  // Typical: 1kg panel × 100 J/(kg·K) specific heat
      emissivity(0.85),    // Typical for solar panel back surface
      absorptivity(0.92),  // Typical for solar cells
      tempCoeffEfficiency(-0.004), // -0.4%/°C for silicon cells
      referenceTemperature(298.15), // 25°C reference
      currentEfficiency(efficiency),
      lastPowerOutput(0.0),
      sunIncidenceAngle(0.0)
{
  updateEfficiency();
}

double SolarPanel::calculatePowerGeneration(const glm::dvec3 &sunDirectionInertial,
                                            const glm::dquat &satelliteAttitude,
                                            double solarFlux,
                                            bool inEclipse)
{
  // No power in eclipse
  if (inEclipse)
  {
    lastPowerOutput = 0.0;
    sunIncidenceAngle = 0.0;
    return 0.0;
  }

  // Calculate incidence angle
  sunIncidenceAngle = calculateIncidenceAngle(sunDirectionInertial, satelliteAttitude);

  // Cosine factor: projection of panel area onto sun direction
  double cosAngle = std::cos(sunIncidenceAngle);

  // If panel is facing away from sun (angle > 90°), no power
  if (cosAngle < 0.0)
  {
    cosAngle = 0.0;
  }

  // Power = Solar Flux × Area × Efficiency × cos(angle) × Degradation
  lastPowerOutput = solarFlux * area * currentEfficiency * cosAngle * degradationFactor;

  return lastPowerOutput;
}

void SolarPanel::updateTemperature(double deltaTime,
                                   const glm::dvec3 &sunDirectionInertial,
                                   const glm::dquat &satelliteAttitude,
                                   double solarFlux,
                                   bool inEclipse)
{
  /**
   * Thermal Model:
   *
   * Energy balance: dT/dt = (Q_in - Q_out) / (m·c)
   *
   * Q_in = Solar absorption (when illuminated)
   *      = Solar Flux × Area × Absorptivity × cos(angle)
   *
   * Q_out = Radiative cooling (always)
   *       = σ × Emissivity × Area × T^4
   *
   * where σ = Stefan-Boltzmann constant = 5.67e-8 W/(m²·K^4)
   */

  const double STEFAN_BOLTZMANN = 5.67e-8; // W/(m²·K^4)

  double Q_in = 0.0;

  if (!inEclipse)
  {
    // Calculate solar heating
    double incidenceAngle = calculateIncidenceAngle(sunDirectionInertial, satelliteAttitude);
    double cosAngle = std::cos(incidenceAngle);

    if (cosAngle > 0.0) // Panel facing sun
    {
      // Absorbed solar power = flux × area × absorptivity × cos(angle)
      Q_in = solarFlux * area * absorptivity * cosAngle;
    }
  }

  // Radiative cooling (both sides of panel radiate to space at ~3K)
  // Assume both sides radiate (front and back)
  double Q_out = 2.0 * STEFAN_BOLTZMANN * emissivity * area * std::pow(temperature, 4);

  // Net heat flow
  double Q_net = Q_in - Q_out;

  // Temperature change: dT/dt = Q_net / thermal_mass
  double dT = (Q_net / thermalMass) * deltaTime;
  temperature += dT;

  // Clamp temperature to reasonable bounds
  // Lower bound: cosmic background radiation (~3K)
  // Upper bound: avoid unrealistic values (solar panels typically 200-400K)
  temperature = glm::clamp(temperature, 3.0, 500.0);

  // Update efficiency based on new temperature
  updateEfficiency();
}

void SolarPanel::updateEfficiency()
{
  // Temperature-dependent efficiency
  // Efficiency = Base Efficiency × (1 + α × ΔT)
  // where α is temperature coefficient (typically -0.004 /°C for silicon)

  double deltaT = temperature - referenceTemperature;
  currentEfficiency = baseEfficiency * (1.0 + tempCoeffEfficiency * deltaT);

  // Clamp efficiency to reasonable bounds
  currentEfficiency = glm::clamp(currentEfficiency, 0.0, 1.0);
}

double SolarPanel::calculateIncidenceAngle(const glm::dvec3 &sunDirectionInertial,
                                           const glm::dquat &satelliteAttitude) const
{
  // Transform panel normal from body frame to inertial frame
  glm::dvec3 panelNormalInertial = satelliteAttitude * normalDirection;

  // Calculate angle between panel normal and sun direction
  double dotProduct = glm::dot(panelNormalInertial, sunDirectionInertial);

  // Clamp to avoid numerical issues with acos
  dotProduct = glm::clamp(dotProduct, -1.0, 1.0);

  return std::acos(dotProduct);
}
