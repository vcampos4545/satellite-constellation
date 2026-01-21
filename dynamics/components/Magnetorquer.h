#ifndef MAGNETORQUER_H
#define MAGNETORQUER_H

#include "Component.h"
#include <glm/glm.hpp>
#include <string>

/**
 * Magnetorquer Actuator Component
 *
 * Simulates a magnetic torque rod (magnetorquer) that generates torque by
 * interacting with Earth's magnetic field. Magnetorquers are commonly used for:
 * - Attitude control (especially for small satellites)
 * - Reaction wheel momentum desaturation
 * - Detumbling after deployment
 *
 * Physics:
 * - Torque = magnetic_dipole × magnetic_field (cross product)
 * - τ = m × B, where m is dipole moment and B is magnetic field
 * - Can only produce torque perpendicular to the local magnetic field
 *
 * Characteristics:
 * - Typical dipole moment: 0.1 - 50 Am² (depending on size)
 * - Low power consumption (~1-5W typical)
 * - No moving parts (high reliability)
 * - Cannot produce torque parallel to magnetic field
 *
 * Limitations:
 * - Torque depends on local magnetic field strength
 * - Cannot produce arbitrary torque vectors
 * - Limited control authority at equator (weak field)
 * - Slow response (limited bandwidth)
 *
 * Usage:
 *   auto mtq = spacecraft->addComponent<Magnetorquer>("MTQ-X");
 *   mtq->setAxis(glm::dvec3(1,0,0));  // X-axis torquer
 *   mtq->setMaxDipole(10.0);          // 10 Am² max
 *   mtq->commandDipole(5.0);          // Command 5 Am² dipole
 *   glm::dvec3 torque = mtq->getTorque();  // Get resulting torque
 */
class Magnetorquer : public Actuator
{
public:
  /**
   * Default constructor - X-axis magnetorquer
   */
  Magnetorquer();

  /**
   * Named constructor
   * @param name Component name (e.g., "MTQ-X", "MTQ-Y", "MTQ-Z")
   */
  explicit Magnetorquer(const std::string &name);

  /**
   * Constructor with axis
   * @param axis Magnetic dipole axis in body frame (unit vector)
   * @param maxDipole Maximum magnetic dipole moment (Am²)
   * @param name Component name
   */
  Magnetorquer(const glm::dvec3 &axis, double maxDipole, const std::string &name = "Magnetorquer");

  // ========== ACTUATOR INTERFACE ==========

  /**
   * Get component type name
   */
  std::string getTypeName() const override { return "Magnetorquer"; }

  /**
   * Get torque produced by magnetorquer
   * @return Torque vector in body frame (Nm)
   */
  glm::dvec3 getTorque() const override;

  /**
   * Update magnetorquer state
   * @param deltaTime Time step (seconds)
   */
  void update(double deltaTime) override;

  // ========== COMMAND INTERFACE ==========

  /**
   * Command magnetic dipole moment
   * @param dipole Commanded dipole along axis (Am²)
   *               Positive = dipole in +axis direction
   *               Negative = dipole in -axis direction
   */
  void commandDipole(double dipole);

  /**
   * Command dipole as fraction of maximum
   * @param fraction Commanded dipole as fraction (-1 to +1)
   */
  void commandDipoleFraction(double fraction);

  /**
   * Get commanded dipole moment
   * @return Commanded dipole (Am²)
   */
  double getCommandedDipole() const { return commandedDipole; }

  /**
   * Get actual dipole moment (with dynamics)
   * @return Actual dipole (Am²)
   */
  double getActualDipole() const { return actualDipole; }

  // ========== MAGNETIC FIELD INTERFACE ==========

  /**
   * Set local magnetic field for torque calculation
   * Must be called before getTorque() each timestep
   * @param field Magnetic field in body frame (Tesla)
   */
  void setMagneticField(const glm::dvec3 &field) { magneticField = field; }

  /**
   * Get current magnetic field
   * @return Magnetic field in body frame (Tesla)
   */
  glm::dvec3 getMagneticField() const { return magneticField; }

  // ========== CONFIGURATION ==========

  /**
   * Set magnetorquer axis
   * @param axis Unit vector defining dipole direction in body frame
   */
  void setAxis(const glm::dvec3 &axis) { dipoleAxis = glm::normalize(axis); }

  /**
   * Get magnetorquer axis
   * @return Dipole axis unit vector
   */
  glm::dvec3 getAxis() const { return dipoleAxis; }

  /**
   * Set maximum dipole moment
   * @param maxDipole Maximum dipole (Am²)
   */
  void setMaxDipole(double maxDipole) { maxDipoleMoment = maxDipole; }

  /**
   * Get maximum dipole moment
   * @return Maximum dipole (Am²)
   */
  double getMaxDipole() const { return maxDipoleMoment; }

  /**
   * Set time constant for dipole dynamics
   * @param tau Time constant (seconds)
   */
  void setTimeConstant(double tau) { timeConstant = tau; }

  /**
   * Get current power consumption
   * @return Power consumption (Watts)
   */
  double getPowerConsumption() const;

  /**
   * Set power coefficient (power per Am²)
   * @param coeff Power coefficient (W/Am²)
   */
  void setPowerCoefficient(double coeff) { powerCoefficient = coeff; }

private:
  // ========== CONFIGURATION ==========
  glm::dvec3 dipoleAxis;      // Dipole direction in body frame (unit vector)
  double maxDipoleMoment;     // Maximum dipole moment (Am²)
  double timeConstant;        // Response time constant (seconds)
  double powerCoefficient;    // Power consumption per Am² (W/Am²)

  // ========== STATE ==========
  double commandedDipole;     // Commanded dipole moment (Am²)
  double actualDipole;        // Actual dipole moment after dynamics (Am²)
  glm::dvec3 magneticField;   // Local magnetic field in body frame (Tesla)
};

#endif // MAGNETORQUER_H
