#include "Config.h"
#include "Constants.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>

void Orbit::toCartesian(glm::dvec3 &position, glm::dvec3 &velocity, double mu) const
{
  /**
   * ORBITAL ELEMENTS TO CARTESIAN CONVERSION
   *
   * Coordinate System (Earth-Centered Inertial):
   * - Y-axis: Points to North Pole (Earth's rotation axis)
   * - Z-axis: Points to Vernal Equinox (First Point of Aries)
   * - X-axis: 90° East of vernal equinox (completes right-handed system)
   * - Equatorial plane: XZ plane (Y = 0)
   *
   * Given Keplerian elements (a, e, i, Ω, ω, ν), compute position and velocity vectors.
   *
   * Reference: Vallado, "Fundamentals of Astrodynamics and Applications", adapted for Y-up
   */

  // ========== STEP 1: CALCULATE ORBITAL RADIUS ==========
  // From orbit equation: r = a(1-e²)/(1+e*cos(ν))
  double p = a * (1.0 - e * e); // Semi-latus rectum
  double r = p / (1.0 + e * cos(v));

  // ========== STEP 2: PERIFOCAL FRAME POSITION ==========
  // Perifocal frame: P-axis points to periapsis, Q-axis 90° ahead, W-axis is orbit normal
  glm::dvec3 posPerifocal(
      r * cos(v),  // P-component (toward periapsis)
      r * sin(v),  // Q-component (90° ahead in orbital plane)
      0.0);        // W-component (perpendicular to orbital plane)

  // ========== STEP 3: PERIFOCAL FRAME VELOCITY ==========
  // Using orbital mechanics: v_p = -(μ/h)*sin(ν), v_q = (μ/h)*(e+cos(ν))
  // where h = sqrt(μp) is specific angular momentum
  double h = sqrt(mu * p);
  glm::dvec3 velPerifocal(
      -(mu / h) * sin(v),              // P-component (radial)
      (mu / h) * (e + cos(v)),         // Q-component (tangential)
      0.0);                            // W-component (zero for planar orbits)

  // ========== STEP 4: TRANSFORMATION TO INERTIAL FRAME ==========
  // For Y-up (North), Z-vernal coordinate system
  //
  // Transformation matrix [PQW] → [XYZ]:
  // This encodes the three Euler rotations (ω, i, Ω) in the correct sequence
  //
  // Unit vectors in inertial frame:
  // P̂ (periapsis direction) = rotation of X-axis by (ω, i, Ω)
  // Q̂ (90° ahead in orbit)  = rotation of Y-axis by (ω, i, Ω)
  // Ŵ (orbit normal)         = rotation of Z-axis by (ω, i, Ω)

  double cos_omega = cos(omega);
  double sin_omega = sin(omega);
  double cos_i = cos(i);
  double sin_i = sin(i);
  double cos_w = cos(w);
  double sin_w = sin(w);

  // Transformation matrix columns are the unit vectors P̂, Q̂, Ŵ expressed in inertial frame
  // Derived from rotation sequence for Y-North, Z-Vernal system

  // P̂ (periapsis direction in inertial frame):
  glm::dvec3 p_hat(
      sin_omega * cos_w + cos_i * cos_omega * sin_w,  // X-component
      sin_i * sin_w,                                   // Y-component (north)
      cos_omega * cos_w - cos_i * sin_omega * sin_w); // Z-component (vernal)

  // Q̂ (90° ahead of periapsis in orbital plane, in inertial frame):
  glm::dvec3 q_hat(
      cos_i * cos_omega * cos_w - sin_omega * sin_w,  // X-component
      sin_i * cos_w,                                   // Y-component (north)
      -cos_i * sin_omega * cos_w - cos_omega * sin_w);// Z-component (vernal)

  // Ŵ (orbit normal in inertial frame):
  glm::dvec3 w_hat(
      sin_i * cos_omega,  // X-component
      -cos_i,             // Y-component (north) - negative because measured from +Y
      sin_i * sin_omega); // Z-component (vernal)

  // Transform position and velocity from perifocal to inertial frame
  position = posPerifocal.x * p_hat + posPerifocal.y * q_hat + posPerifocal.z * w_hat;
  velocity = velPerifocal.x * p_hat + velPerifocal.y * q_hat + velPerifocal.z * w_hat;
}
