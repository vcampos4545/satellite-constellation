#include "GravityModel.h"
#include "Constants.h"
#include <cmath>

glm::dvec3 GravityModel::calculatePointMassGravity(
    const glm::dvec3 &pos,
    const glm::dvec3 &bodyPos,
    double bodyMass)
{
  glm::dvec3 toBody = bodyPos - pos;
  double distance = glm::length(toBody);

  if (distance < 1.0)
    return glm::dvec3(0.0); // Avoid division by zero

  glm::dvec3 direction = glm::normalize(toBody);
  double accelMagnitude = G * bodyMass / (distance * distance);
  return direction * accelMagnitude;
}

glm::dvec3 GravityModel::calculateZonalHarmonics(
    const glm::dvec3 &pos,
    const glm::dvec3 &earthCenter,
    double earthMass)
{
  // Calculate Earth's non-spherical gravity perturbations (J2, J3, J4)
  // These account for Earth's oblate shape and mass distribution
  //
  // Reference: Vallado, "Fundamentals of Astrodynamics and Applications", 4th Ed.
  // Chapter 8: Perturbations

  // Position vector relative to Earth center
  glm::dvec3 r = pos - earthCenter;
  double r_mag = glm::length(r);

  if (r_mag < EARTH_EQUATORIAL_RADIUS)
    return glm::dvec3(0.0); // Inside Earth - shouldn't happen

  // Normalized position components
  double x = r.x / r_mag;
  double y = r.y / r_mag;
  double z = r.z / r_mag; // z/r is sin(latitude) in Earth-centered coordinates

  double z2 = z * z;   // (z/r)²
  double z4 = z2 * z2; // (z/r)⁴

  // Common terms
  double mu = G * earthMass;
  double Re = EARTH_EQUATORIAL_RADIUS;
  double Re_r = Re / r_mag;    // Re/r
  double Re_r2 = Re_r * Re_r;  // (Re/r)²
  double Re_r3 = Re_r2 * Re_r; // (Re/r)³
  double Re_r4 = Re_r3 * Re_r; // (Re/r)⁴
  double mu_r3 = mu / (r_mag * r_mag * r_mag);

  // ========== J2 PERTURBATION (Oblateness - Equatorial Bulge) ==========
  // Dominant term: ~1000x larger than J3/J4
  // Causes RAAN and argument of perigee to precess
  double j2_factor = -1.5 * J2 * Re_r2 * mu_r3;

  glm::dvec3 accel_J2;
  accel_J2.x = j2_factor * x * (1.0 - 5.0 * z2);
  accel_J2.y = j2_factor * y * (1.0 - 5.0 * z2);
  accel_J2.z = j2_factor * z * (3.0 - 5.0 * z2);

  // ========== J3 PERTURBATION (Pear-Shape - North/South Asymmetry) ==========
  // Second-order term: Southern hemisphere slightly "heavier"
  // Important for high-inclination orbits
  double j3_factor = -0.5 * J3 * Re_r3 * mu_r3;

  glm::dvec3 accel_J3;
  accel_J3.x = j3_factor * x * (5.0 * z * (7.0 * z2 - 3.0));
  accel_J3.y = j3_factor * y * (5.0 * z * (7.0 * z2 - 3.0));
  accel_J3.z = j3_factor * (6.0 * z2 - 7.0 * z4 - 3.0 / 5.0);

  // ========== J4 PERTURBATION (Higher-Order Oblateness) ==========
  // Third-order term: Additional refinement to oblateness
  // Small correction to J2 effects
  double j4_factor = 0.625 * J4 * Re_r4 * mu_r3; // 5/8 = 0.625

  glm::dvec3 accel_J4;
  accel_J4.x = j4_factor * x * (1.0 - 14.0 * z2 + 21.0 * z4);
  accel_J4.y = j4_factor * y * (1.0 - 14.0 * z2 + 21.0 * z4);
  accel_J4.z = j4_factor * z * (5.0 - 70.0 * z2 / 3.0 + 21.0 * z4);

  // Total zonal harmonics acceleration
  return accel_J2 + accel_J3 + accel_J4;
}

glm::dvec3 GravityModel::calculateThirdBodyPerturbation(
    const glm::dvec3 &satellitePos,
    const glm::dvec3 &earthPos,
    const glm::dvec3 &thirdBodyPos,
    double thirdBodyMass)
{
  // Third-body perturbation using differential gravity
  // Effect = gravity on satellite - gravity on Earth
  //
  // This represents the tidal effect: the difference between the gravitational
  // pull on the satellite vs the pull on Earth's center
  //
  // Important for GEO satellites (Moon/Sun perturbations cause orbit drift)

  // Acceleration of satellite due to third body
  glm::dvec3 accel_sat = calculatePointMassGravity(satellitePos, thirdBodyPos, thirdBodyMass);

  // Acceleration of Earth due to third body
  glm::dvec3 accel_earth = calculatePointMassGravity(earthPos, thirdBodyPos, thirdBodyMass);

  // Differential acceleration (perturbation on satellite's orbit)
  return accel_sat - accel_earth;
}
