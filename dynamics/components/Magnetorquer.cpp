#include "Magnetorquer.h"
#include <cmath>
#include <algorithm>

Magnetorquer::Magnetorquer()
    : Actuator(),
      dipoleAxis(1.0, 0.0, 0.0),   // X-axis default
      maxDipoleMoment(10.0),       // 10 Am² typical for small satellite
      timeConstant(0.1),           // 100ms response time
      powerCoefficient(0.5),       // 0.5 W per Am²
      commandedDipole(0.0),
      actualDipole(0.0),
      magneticField(0.0)
{
}

Magnetorquer::Magnetorquer(const std::string &name)
    : Actuator(name),
      dipoleAxis(1.0, 0.0, 0.0),   // X-axis default
      maxDipoleMoment(10.0),       // 10 Am²
      timeConstant(0.1),           // 100ms response time
      powerCoefficient(0.5),       // 0.5 W per Am²
      commandedDipole(0.0),
      actualDipole(0.0),
      magneticField(0.0)
{
}

Magnetorquer::Magnetorquer(const glm::dvec3 &axis, double maxDipole, const std::string &name)
    : Actuator(name),
      dipoleAxis(glm::normalize(axis)),
      maxDipoleMoment(maxDipole),
      timeConstant(0.1),
      powerCoefficient(0.5),
      commandedDipole(0.0),
      actualDipole(0.0),
      magneticField(0.0)
{
}

void Magnetorquer::update(double deltaTime)
{
  if (!enabled)
  {
    actualDipole = 0.0;
    return;
  }

  /**
   * First-order dynamics model
   * dM/dt = (M_cmd - M_actual) / tau
   *
   * Discrete update:
   * M_actual(k+1) = M_actual(k) + dt/tau * (M_cmd - M_actual(k))
   *
   * Or exponential form:
   * M_actual = M_cmd + (M_actual_prev - M_cmd) * exp(-dt/tau)
   */

  if (timeConstant > 0.0)
  {
    double alpha = 1.0 - std::exp(-deltaTime / timeConstant);
    actualDipole = actualDipole + alpha * (commandedDipole - actualDipole);
  }
  else
  {
    // Instantaneous response
    actualDipole = commandedDipole;
  }
}

glm::dvec3 Magnetorquer::getTorque() const
{
  if (!enabled)
  {
    return glm::dvec3(0.0);
  }

  /**
   * Magnetorquer torque physics:
   * τ = m × B
   *
   * where:
   * m = magnetic dipole moment vector (Am²)
   * B = local magnetic field (Tesla)
   * τ = resulting torque (Nm)
   *
   * The dipole vector is: m = actualDipole * dipoleAxis
   */

  glm::dvec3 dipoleVector = actualDipole * dipoleAxis;
  glm::dvec3 torque = glm::cross(dipoleVector, magneticField);

  return torque;
}

void Magnetorquer::commandDipole(double dipole)
{
  // Clamp to maximum dipole moment
  commandedDipole = std::clamp(dipole, -maxDipoleMoment, maxDipoleMoment);
}

void Magnetorquer::commandDipoleFraction(double fraction)
{
  // Clamp fraction to [-1, 1]
  fraction = std::clamp(fraction, -1.0, 1.0);
  commandedDipole = fraction * maxDipoleMoment;
}

double Magnetorquer::getPowerConsumption() const
{
  if (!enabled)
  {
    return 0.0;
  }

  // Power consumption proportional to dipole squared (resistive heating in coil)
  // P = k * |M|² or simplified: P = k * |M|
  return powerCoefficient * std::abs(actualDipole);
}
