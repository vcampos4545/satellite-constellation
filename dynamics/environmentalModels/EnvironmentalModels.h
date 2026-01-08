#ifndef ENVIRONMENTAL_MODELS_H
#define ENVIRONMENTAL_MODELS_H

#include <glm/glm.hpp>

/**
 * Environmental Perturbation Models
 *
 * Provides models for non-gravitational forces acting on satellites:
 * - Atmospheric drag
 * - Solar radiation pressure
 *
 * These forces are small but significant for long-duration missions
 * and low Earth orbit satellites.
 */
class EnvironmentalModels
{
public:
  /**
   * Calculate atmospheric drag acceleration
   *
   * Uses exponential atmosphere model: ρ(h) = ρ₀ * e^(-h/H)
   * Drag force: F = -0.5 * ρ * v² * Cd * A
   *
   * Significant below ~1000 km altitude. Causes orbital decay.
   *
   * @param pos Satellite position
   * @param vel Satellite velocity (inertial frame)
   * @param earthCenter Earth center position
   * @param mass Satellite mass (kg)
   * @param crossSectionalArea Cross-sectional area (m²)
   * @param dragCoefficient Drag coefficient (dimensionless, typically 2.0-2.5)
   * @return Drag acceleration vector (m/s²)
   */
  static glm::dvec3 calculateAtmosphericDrag(
      const glm::dvec3 &pos,
      const glm::dvec3 &vel,
      const glm::dvec3 &earthCenter,
      double mass,
      double crossSectionalArea,
      double dragCoefficient);

  /**
   * Calculate solar radiation pressure acceleration
   *
   * Photons from the Sun exert pressure on satellite surfaces.
   * Pressure: P = Solar_Flux / c ≈ 4.56 × 10⁻⁶ N/m²
   *
   * Includes simple cylindrical shadow model for Earth eclipses.
   *
   * @param pos Satellite position
   * @param sunPos Sun position
   * @param earthCenter Earth center position
   * @param mass Satellite mass (kg)
   * @param reflectiveArea Effective reflective area (m²)
   * @param reflectivityCoeff Reflectivity coefficient (0=absorb, 1=reflect, 2=mirror)
   * @return SRP acceleration vector (m/s²)
   */
  static glm::dvec3 calculateSolarRadiationPressure(
      const glm::dvec3 &pos,
      const glm::dvec3 &sunPos,
      const glm::dvec3 &earthCenter,
      double mass,
      double reflectiveArea,
      double reflectivityCoeff);
};

#endif // ENVIRONMENTAL_MODELS_H
