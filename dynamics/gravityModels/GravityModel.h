#ifndef GRAVITY_MODEL_H
#define GRAVITY_MODEL_H

#include <glm/glm.hpp>

/**
 * Gravity Model
 *
 * Provides gravitational acceleration calculations for orbital dynamics.
 * Includes:
 * - Point mass gravity (two-body problem)
 * - Zonal harmonics (J2, J3, J4) for Earth oblateness
 * - Third-body perturbations (Sun, Moon)
 *
 * Reference: Vallado, "Fundamentals of Astrodynamics and Applications", 4th Ed.
 */
class GravityModel
{
public:
  /**
   * Calculate point-mass gravitational acceleration
   *
   * @param pos Satellite position
   * @param bodyPos Gravitating body position
   * @param bodyMass Mass of gravitating body (kg)
   * @return Acceleration vector (m/s²)
   */
  static glm::dvec3 calculatePointMassGravity(
      const glm::dvec3 &pos,
      const glm::dvec3 &bodyPos,
      double bodyMass);

  /**
   * Calculate zonal harmonics perturbations (J2, J3, J4)
   *
   * Models Earth's non-spherical mass distribution:
   * - J2: Oblateness (equatorial bulge) - dominant term
   * - J3: Pear-shaped asymmetry (North-South)
   * - J4: Higher-order oblateness correction
   *
   * @param pos Satellite position
   * @param earthCenter Earth center position
   * @param earthMass Earth mass (kg)
   * @return Perturbation acceleration vector (m/s²)
   */
  static glm::dvec3 calculateZonalHarmonics(
      const glm::dvec3 &pos,
      const glm::dvec3 &earthCenter,
      double earthMass);

  /**
   * Calculate third-body gravitational perturbations
   *
   * Computes differential gravitational acceleration from Sun and Moon.
   * Important for GEO satellites and long-duration missions.
   *
   * @param satellitePos Satellite position
   * @param earthPos Earth position
   * @param thirdBodyPos Third body (Sun/Moon) position
   * @param thirdBodyMass Third body mass (kg)
   * @return Perturbation acceleration vector (m/s²)
   */
  static glm::dvec3 calculateThirdBodyPerturbation(
      const glm::dvec3 &satellitePos,
      const glm::dvec3 &earthPos,
      const glm::dvec3 &thirdBodyPos,
      double thirdBodyMass);
};

#endif // GRAVITY_MODEL_H
