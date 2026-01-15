#ifndef REACTION_WHEEL_H
#define REACTION_WHEEL_H

#include <glm/glm.hpp>
#include <string>
#include "Component.h"

/**
 * Reaction Wheel Component
 *
 * Models a single reaction wheel or cluster for spacecraft attitude control.
 *
 * Physics:
 * - Stores angular momentum by spinning internal rotor
 * - Produces torque on spacecraft (Newton's 3rd law)
 * - Limited by maximum torque and momentum capacity
 * - Momentum must be desaturated periodically (using magnetorquers or thrusters)
 *
 * Typical Values:
 * - Small satellites: 0.1-0.5 N·m torque, 5-20 N·m·s momentum
 * - Large satellites: 0.5-2.0 N·m torque, 50-200 N·m·s momentum
 *
 * Usage:
 *   auto* rw = spacecraft->addComponent<ReactionWheel>("RW+X", 0.5, 10.0, glm::dvec3(1,0,0));
 *   rw->commandTorque(glm::dvec3(0.1, 0, 0)); // Command 0.1 N·m
 *   double torque = rw->getTorque(); // Returns commanded torque (limited by constraints)
 */
class ReactionWheel : public Actuator
{
public:
  /**
   * Constructor
   * @param name Component name (e.g., "RW+X", "RW-Y")
   * @param maxTorque Maximum torque capability (N·m)
   * @param maxMomentum Maximum momentum storage (N·m·s)
   * @param spinAxis Wheel spin axis in body frame (normalized)
   */
  ReactionWheel(const std::string &name,
                double maxTorque,
                double maxMomentum,
                const glm::dvec3 &spinAxis = glm::dvec3(1.0, 0.0, 0.0));

  // Component interface overrides
  std::string getTypeName() const override { return "ReactionWheel"; }
  void update(double deltaTime) override;
  void reset() override;

  // Actuator interface overrides
  glm::dvec3 getForce() const override { return glm::dvec3(0.0); } // Wheels produce no force
  glm::dvec3 getTorque() const override;

  /**
   * Command desired torque
   * Torque will be limited to maxTorque and momentum constraints
   * @param torque Desired torque vector in body frame (N·m)
   */
  void commandTorque(const glm::dvec3 &torque);

  /**
   * Command desired torque along wheel spin axis
   * @param torque Desired torque magnitude (N·m), positive = accelerate wheel
   */
  void commandTorque(double torque);

  /**
   * Get current momentum saturation level
   * @return Fraction of maximum momentum (0.0 = empty, 1.0 = saturated)
   */
  double getSaturation() const;

  /**
   * Check if wheel is saturated (needs desaturation)
   * @return True if wheel has reached momentum limit
   */
  bool isSaturated() const;

  /**
   * Desaturate wheel (external torque applied, e.g., magnetorquers)
   * @param externalTorque External torque to reduce momentum (N·m)
   * @param deltaTime Time step (seconds)
   */
  void desaturate(double externalTorque, double deltaTime);

  // Getters
  double getMaxTorque() const { return maxTorque; }
  double getMaxMomentum() const { return maxMomentum; }
  double getCurrentMomentum() const { return currentMomentum; }
  glm::dvec3 getSpinAxis() const { return spinAxis; }
  double getCommandedTorque() const { return commandedTorque; }
  double getActualTorque() const { return actualTorque; }
  double getPowerConsumption() const { return powerConsumption; }

  // Setters
  void setMaxTorque(double torque) { maxTorque = torque; }
  void setMaxMomentum(double momentum) { maxMomentum = momentum; }

private:
  // Physical properties
  double maxTorque;          // Maximum output torque (N·m)
  double maxMomentum;        // Maximum momentum storage (N·m·s)
  glm::dvec3 spinAxis;       // Wheel spin axis in body frame (normalized)

  // Current state
  double currentMomentum;    // Current stored momentum (N·m·s)
  double commandedTorque;    // Commanded torque (N·m)
  double actualTorque;       // Actual torque after limits (N·m)
  double wheelSpeed;         // Wheel angular velocity (rad/s) - for telemetry

  // Power model (simple)
  double powerConsumption;   // Current power draw (Watts)
  double idlePower;          // Power when idle (W)
  double torquePowerCoeff;   // Power per torque (W/(N·m))

  /**
   * Apply momentum and torque limits
   * Ensures wheel doesn't exceed physical constraints
   */
  void applyLimits(double deltaTime);

  /**
   * Calculate power consumption based on torque
   */
  void updatePower();
};

#endif // REACTION_WHEEL_H
