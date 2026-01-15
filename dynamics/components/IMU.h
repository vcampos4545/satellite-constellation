#ifndef IMU_H
#define IMU_H

#include <glm/glm.hpp>
#include <random>
#include "Component.h"

/**
 * Inertial Measurement Unit (IMU) Sensor Model
 *
 * Simulates a realistic IMU with:
 * - Angular velocity measurements (gyroscope)
 * - Noise (white noise + bias)
 * - Configurable error characteristics
 *
 * Component Integration:
 * - Inherits from Sensor component
 * - Automatically updates bias drift each frame
 * - Can be accessed via spacecraft component system
 */
class IMU : public Sensor
{
public:
  IMU();
  explicit IMU(const std::string &name);

  // Component interface overrides
  std::string getTypeName() const override { return "IMU"; }
  void update(double deltaTime) override { updateBias(deltaTime); }

  // Sensor interface override
  void measure() override {} // Not used for IMU (measurement is on-demand)

  /**
   * Get simulated IMU measurement from true angular velocity
   * Adds noise and bias to simulate real sensor
   * @param trueAngularVelocity True angular velocity in body frame (rad/s)
   * @return Noisy measurement with bias (rad/s)
   */
  glm::dvec3 measureAngularVelocity(const glm::dvec3 &trueAngularVelocity);

  /**
   * Update bias (random walk over time)
   * Called automatically by Component::update()
   */
  void updateBias(double deltaTime);

  // Setters for noise characteristics
  void setWhiteNoise(double noise) { whiteNoiseStdDev = noise; }
  void setBiasStability(double bias) { biasStability = bias; }
  void setBiasRandomWalk(double walk) { biasRandomWalk = walk; }

  // Getters
  glm::dvec3 getCurrentBias() const { return bias; }
  glm::dvec3 getLastMeasurement() const { return lastMeasurement; }

private:
  // Noise characteristics (typical for MEMS gyro)
  double whiteNoiseStdDev;  // White noise standard deviation (rad/s)
  double biasStability;     // Constant bias offset (rad/s)
  double biasRandomWalk;    // Bias drift rate (rad/s/sqrt(s))

  // Current state
  glm::dvec3 bias;           // Current bias (drifts over time)
  glm::dvec3 lastMeasurement; // Last measurement returned

  // Random number generation
  std::mt19937 rng;
  std::normal_distribution<double> whiteNoiseDist;
};

#endif // IMU_H
