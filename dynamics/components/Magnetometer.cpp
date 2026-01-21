#include "Magnetometer.h"
#include <cmath>

Magnetometer::Magnetometer()
    : Sensor(),
      noiseStdDev(10e-9),           // 10 nT noise (typical spacecraft magnetometer)
      bias(0.0),
      biasRandomWalk(1e-9),         // 1 nT/sqrt(s) bias drift
      scaleFactors(1.0, 1.0, 1.0),  // Perfect scale factors
      measurementRange(100e-6),     // 100 μT range (covers Earth's field)
      measuredField(0.0),
      trueField(0.0),
      saturated(false),
      rng(std::random_device{}()),
      noiseDist(0.0, 1.0)
{
  // Initialize bias with small random offset
  bias.x = noiseDist(rng) * 50e-9;  // ±50 nT initial bias
  bias.y = noiseDist(rng) * 50e-9;
  bias.z = noiseDist(rng) * 50e-9;
}

Magnetometer::Magnetometer(const std::string &name)
    : Sensor(name),
      noiseStdDev(10e-9),           // 10 nT noise
      bias(0.0),
      biasRandomWalk(1e-9),         // 1 nT/sqrt(s) bias drift
      scaleFactors(1.0, 1.0, 1.0),  // Perfect scale factors
      measurementRange(100e-6),     // 100 μT range
      measuredField(0.0),
      trueField(0.0),
      saturated(false),
      rng(std::random_device{}()),
      noiseDist(0.0, 1.0)
{
  // Initialize bias with small random offset
  bias.x = noiseDist(rng) * 50e-9;
  bias.y = noiseDist(rng) * 50e-9;
  bias.z = noiseDist(rng) * 50e-9;
}

void Magnetometer::update(double deltaTime)
{
  // Bias random walk (Brownian motion)
  double driftStdDev = biasRandomWalk * std::sqrt(deltaTime);

  bias.x += noiseDist(rng) * driftStdDev;
  bias.y += noiseDist(rng) * driftStdDev;
  bias.z += noiseDist(rng) * driftStdDev;

  // Limit bias drift to reasonable bounds
  double maxBias = 500e-9;  // 500 nT max bias
  bias.x = glm::clamp(bias.x, -maxBias, maxBias);
  bias.y = glm::clamp(bias.y, -maxBias, maxBias);
  bias.z = glm::clamp(bias.z, -maxBias, maxBias);
}

void Magnetometer::measure(const glm::dvec3 &magneticFieldECI, const glm::dquat &attitude)
{
  // Transform magnetic field from ECI to body frame
  // attitude is body-to-ECI, so we need the inverse (ECI-to-body)
  glm::dmat3 R_eci_to_body = glm::mat3_cast(glm::inverse(attitude));
  trueField = R_eci_to_body * magneticFieldECI;

  // Apply sensor errors
  measuredField = applyErrors(trueField);

  // Check for saturation
  saturated = false;
  for (int i = 0; i < 3; ++i)
  {
    if (std::abs(measuredField[i]) > measurementRange)
    {
      measuredField[i] = glm::sign(measuredField[i]) * measurementRange;
      saturated = true;
    }
  }
}

glm::dvec3 Magnetometer::applyErrors(const glm::dvec3 &trueValue)
{
  /**
   * Apply magnetometer error model:
   * measured = scale * true + bias + noise
   *
   * More sophisticated models could include:
   * - Misalignment matrix (non-orthogonality)
   * - Temperature effects
   * - Hysteresis
   * - Hard/soft iron disturbances
   */

  glm::dvec3 measured;

  // Apply scale factor errors
  measured.x = scaleFactors.x * trueValue.x;
  measured.y = scaleFactors.y * trueValue.y;
  measured.z = scaleFactors.z * trueValue.z;

  // Add bias
  measured += bias;

  // Add white noise
  measured.x += noiseDist(rng) * noiseStdDev;
  measured.y += noiseDist(rng) * noiseStdDev;
  measured.z += noiseDist(rng) * noiseStdDev;

  return measured;
}
