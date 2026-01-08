#ifndef CONFIG_H
#define CONFIG_H

#include <glm/glm.hpp>

struct Orbit
{
  double a;     // Semimajor axis (meters)
  double e;     // Eccentricity
  double i;     // Inclination (radians)
  double omega; // Right ascension of ascending node / RAAN (radians)
  double w;     // Argument of perigee/periapsis (radians)
  double v;     // True anomaly (radians)

  /**
   * Convert orbital elements to Cartesian position and velocity
   *
   * Uses standard orbital mechanics transformations:
   * 1. Calculate position/velocity in perifocal frame (orbital plane)
   * 2. Rotate by argument of perigee (w)
   * 3. Rotate by inclination (i)
   * 4. Rotate by RAAN (omega)
   *
   * Reference: Vallado, "Fundamentals of Astrodynamics and Applications"
   */
  void toCartesian(glm::dvec3 &position, glm::dvec3 &velocity, double mu) const;
};

#endif // ORBIT_H
