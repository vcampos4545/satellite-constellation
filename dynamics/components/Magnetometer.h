#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include "Component.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <random>

/**
 * Magnetometer Sensor Component
 *
 * Simulates a three-axis magnetometer that measures Earth's magnetic field
 * in the spacecraft body frame. Magnetometers are used for:
 * - Attitude determination (with magnetic field models like IGRF)
 * - Magnetorquer control feedback
 * - Orbit determination assistance
 *
 * Sensor Characteristics:
 * - Measurement range: typically ±60,000 nT to ±100,000 nT
 * - Resolution: 1-10 nT for spacecraft-grade sensors
 * - Noise: 5-50 nT RMS typical
 * - Bias: 10-100 nT typical (can be calibrated)
 *
 * Features:
 * - Three-axis magnetic field measurement
 * - Configurable noise and bias
 * - Scale factor errors
 * - Misalignment errors (orthogonality)
 *
 * Usage:
 *   auto mag = spacecraft->addComponent<Magnetometer>("MAG");
 *   mag->setNoise(10.0);  // 10 nT RMS noise
 *   mag->measure(magneticFieldECI, attitude);
 *   glm::dvec3 measuredB = mag->getMagneticField();
 */
class Magnetometer : public Sensor
{
public:
  /**
   * Default constructor - typical spacecraft magnetometer
   */
  Magnetometer();

  /**
   * Named constructor
   * @param name Component name
   */
  explicit Magnetometer(const std::string &name);

  // ========== SENSOR INTERFACE ==========

  /**
   * Get component type name
   */
  std::string getTypeName() const override { return "Magnetometer"; }

  /**
   * Dummy measure() override for Sensor interface
   */
  void measure() override {}

  /**
   * Update magnetometer (bias drift)
   * @param deltaTime Time step in seconds
   */
  void update(double deltaTime) override;

  /**
   * Take magnetometer measurement
   *
   * @param magneticFieldECI Magnetic field vector in ECI frame (Tesla)
   * @param attitude Spacecraft attitude quaternion (body to ECI)
   */
  void measure(const glm::dvec3 &magneticFieldECI, const glm::dquat &attitude);

  // ========== MEASUREMENT GETTERS ==========

  /**
   * Get measured magnetic field in body frame
   * @return Magnetic field vector (Tesla)
   */
  glm::dvec3 getMagneticField() const { return measuredField; }

  /**
   * Get measured magnetic field magnitude
   * @return Field magnitude (Tesla)
   */
  double getFieldMagnitude() const { return glm::length(measuredField); }

  /**
   * Get true magnetic field in body frame (without noise)
   * @return True magnetic field vector (Tesla)
   */
  glm::dvec3 getTrueField() const { return trueField; }

  /**
   * Get current sensor bias
   * @return Bias vector (Tesla)
   */
  glm::dvec3 getBias() const { return bias; }

  /**
   * Get measurement noise level
   * @return Noise standard deviation (Tesla)
   */
  double getNoise() const { return noiseStdDev; }

  // ========== CONFIGURATION SETTERS ==========

  /**
   * Set measurement noise level
   * @param noise Noise standard deviation (Tesla)
   */
  void setNoise(double noise) { noiseStdDev = noise; }

  /**
   * Set sensor bias
   * @param biasVec Bias vector (Tesla)
   */
  void setBias(const glm::dvec3 &biasVec) { bias = biasVec; }

  /**
   * Set bias random walk rate
   * @param walkRate Bias drift rate (Tesla/sqrt(s))
   */
  void setBiasRandomWalk(double walkRate) { biasRandomWalk = walkRate; }

  /**
   * Set scale factor error (multiplicative error)
   * @param scaleFactor Scale factors for each axis (1.0 = perfect)
   */
  void setScaleFactor(const glm::dvec3 &scaleFactor) { scaleFactors = scaleFactor; }

  /**
   * Set sensor measurement range
   * @param range Maximum measurable field magnitude (Tesla)
   */
  void setRange(double range) { measurementRange = range; }

  /**
   * Check if measurement is saturated (outside range)
   * @return True if any axis is saturated
   */
  bool isSaturated() const { return saturated; }

private:
  // ========== SENSOR CHARACTERISTICS ==========
  double noiseStdDev;        // White noise standard deviation (Tesla)
  glm::dvec3 bias;           // Constant bias offset (Tesla)
  double biasRandomWalk;     // Bias drift rate (Tesla/sqrt(s))
  glm::dvec3 scaleFactors;   // Scale factor errors per axis (1.0 = perfect)
  double measurementRange;   // Maximum measurable field (Tesla)

  // ========== MEASUREMENT STATE ==========
  glm::dvec3 measuredField;  // Measured magnetic field (body frame, Tesla)
  glm::dvec3 trueField;      // True magnetic field (body frame, Tesla)
  bool saturated;            // True if measurement is saturated

  // ========== RANDOM NUMBER GENERATION ==========
  std::mt19937 rng;
  std::normal_distribution<double> noiseDist;

  // ========== HELPER METHODS ==========

  /**
   * Apply sensor errors (scale factor, misalignment, bias, noise)
   * @param trueValue True magnetic field in body frame
   * @return Measured field with errors
   */
  glm::dvec3 applyErrors(const glm::dvec3 &trueValue);
};

#endif // MAGNETOMETER_H
