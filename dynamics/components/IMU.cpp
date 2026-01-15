#include "IMU.h"
#include <cmath>

IMU::IMU()
    : Sensor(), // Initialize base class with no name
      whiteNoiseStdDev(0.001),    // 0.001 rad/s = ~0.06 deg/s (typical MEMS gyro)
      biasStability(0.0005),       // 0.0005 rad/s = ~0.03 deg/s constant bias
      biasRandomWalk(0.0001),      // Bias drift
      bias(0.0, 0.0, 0.0),
      lastMeasurement(0.0, 0.0, 0.0),
      rng(std::random_device{}()),
      whiteNoiseDist(0.0, 1.0)
{
  // Initialize bias with random offset (within bias stability)
  bias.x = whiteNoiseDist(rng) * biasStability;
  bias.y = whiteNoiseDist(rng) * biasStability;
  bias.z = whiteNoiseDist(rng) * biasStability;
}

IMU::IMU(const std::string &name)
    : Sensor(name), // Initialize base class with name
      whiteNoiseStdDev(0.001),    // 0.001 rad/s = ~0.06 deg/s (typical MEMS gyro)
      biasStability(0.0005),       // 0.0005 rad/s = ~0.03 deg/s constant bias
      biasRandomWalk(0.0001),      // Bias drift
      bias(0.0, 0.0, 0.0),
      lastMeasurement(0.0, 0.0, 0.0),
      rng(std::random_device{}()),
      whiteNoiseDist(0.0, 1.0)
{
  // Initialize bias with random offset (within bias stability)
  bias.x = whiteNoiseDist(rng) * biasStability;
  bias.y = whiteNoiseDist(rng) * biasStability;
  bias.z = whiteNoiseDist(rng) * biasStability;
}

glm::dvec3 IMU::measureAngularVelocity(const glm::dvec3 &trueAngularVelocity)
{
  // Add white noise (independent on each axis)
  glm::dvec3 whiteNoise(
      whiteNoiseDist(rng) * whiteNoiseStdDev,
      whiteNoiseDist(rng) * whiteNoiseStdDev,
      whiteNoiseDist(rng) * whiteNoiseStdDev);

  // Measurement = true value + bias + white noise
  lastMeasurement = trueAngularVelocity + bias + whiteNoise;

  return lastMeasurement;
}

void IMU::updateBias(double deltaTime)
{
  // Bias random walk (Brownian motion)
  // Each axis drifts independently
  double driftStdDev = biasRandomWalk * std::sqrt(deltaTime);

  bias.x += whiteNoiseDist(rng) * driftStdDev;
  bias.y += whiteNoiseDist(rng) * driftStdDev;
  bias.z += whiteNoiseDist(rng) * driftStdDev;

  // Limit bias drift to reasonable bounds (prevent unbounded growth)
  double maxBias = biasStability * 10.0;
  bias.x = glm::clamp(bias.x, -maxBias, maxBias);
  bias.y = glm::clamp(bias.y, -maxBias, maxBias);
  bias.z = glm::clamp(bias.z, -maxBias, maxBias);
}
