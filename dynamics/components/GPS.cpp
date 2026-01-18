#include "GPS.h"
#include <random>
#include <cmath>

GPS::GPS()
    : Sensor(),
      measuredPosition(0.0),
      measuredVelocity(0.0),
      positionAccuracy(5.0),    // Civilian GPS: ~5m accuracy
      velocityAccuracy(0.2),    // Civilian GPS: ~0.2 m/s accuracy
      updateRate(1.0),          // 1 Hz update rate
      timeSinceUpdate(0.0),
      validFix(true),
      numSatellites(8),         // Typical number of satellites in view
      gdop(2.0),                // Good GDOP
      simulateSignalLoss(false)
{
  updateGDOP();
}

GPS::GPS(const std::string &name)
    : Sensor(name),
      measuredPosition(0.0),
      measuredVelocity(0.0),
      positionAccuracy(5.0),    // Civilian GPS: ~5m accuracy
      velocityAccuracy(0.2),    // Civilian GPS: ~0.2 m/s accuracy
      updateRate(1.0),          // 1 Hz update rate
      timeSinceUpdate(0.0),
      validFix(true),
      numSatellites(8),         // Typical number of satellites in view
      gdop(2.0),                // Good GDOP
      simulateSignalLoss(false)
{
  updateGDOP();
}

GPS::GPS(double posAccuracy, double velAccuracy, double rate)
    : Sensor(),
      measuredPosition(0.0),
      measuredVelocity(0.0),
      positionAccuracy(posAccuracy),
      velocityAccuracy(velAccuracy),
      updateRate(rate),
      timeSinceUpdate(0.0),
      validFix(true),
      numSatellites(8),
      gdop(2.0),
      simulateSignalLoss(false)
{
  updateGDOP();
}

void GPS::update(double deltaTime)
{
  // Accumulate time for rate limiting
  timeSinceUpdate += deltaTime;

  // Check signal availability if simulation is enabled
  if (simulateSignalLoss)
  {
    validFix = checkSignalAvailability();
  }
}

void GPS::measure(const glm::dvec3 &truthPosition, const glm::dvec3 &truthVelocity)
{
  // Only take new measurement if enough time has passed (rate limiting)
  double updateInterval = 1.0 / updateRate;
  if (timeSinceUpdate < updateInterval)
  {
    return; // Not time for new measurement yet
  }

  // Reset time accumulator
  timeSinceUpdate = 0.0;

  // Check if we have valid signal
  if (!validFix)
  {
    // No valid fix - measurements are stale/invalid
    return;
  }

  // Add noise to position measurement
  // GDOP affects accuracy (higher GDOP = worse accuracy)
  double effectivePositionAccuracy = positionAccuracy * gdop;
  measuredPosition = addNoise(truthPosition, effectivePositionAccuracy);

  // Add noise to velocity measurement
  double effectiveVelocityAccuracy = velocityAccuracy * gdop;
  measuredVelocity = addNoise(truthVelocity, effectiveVelocityAccuracy);
}

double GPS::addNoise(double value, double stdDev)
{
  // Box-Muller transform for Gaussian noise
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::normal_distribution<double> dist(0.0, stdDev);

  return value + dist(gen);
}

glm::dvec3 GPS::addNoise(const glm::dvec3 &value, double stdDev)
{
  // Add independent Gaussian noise to each component
  return glm::dvec3(
      addNoise(value.x, stdDev),
      addNoise(value.y, stdDev),
      addNoise(value.z, stdDev));
}

void GPS::updateGDOP()
{
  // Simplified GDOP model based on number of satellites
  // Real GDOP depends on satellite geometry, but this is a reasonable approximation

  if (numSatellites < 4)
  {
    // Need at least 4 satellites for 3D position fix
    gdop = 999.0;  // Invalid
    validFix = false;
  }
  else if (numSatellites == 4)
  {
    gdop = 5.0;  // Poor geometry (minimum satellites)
  }
  else if (numSatellites <= 6)
  {
    gdop = 3.0;  // Moderate geometry
  }
  else if (numSatellites <= 8)
  {
    gdop = 2.0;  // Good geometry
  }
  else
  {
    gdop = 1.5;  // Excellent geometry (many satellites)
  }
}

bool GPS::checkSignalAvailability()
{
  // Simplified signal availability model
  // In reality, this would depend on:
  // - Satellite position above horizon
  // - Atmospheric conditions
  // - Spacecraft orientation (antenna pointing)
  // - Jamming/interference

  // For now, simulate occasional signal loss (5% probability per second)
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist(0.0, 1.0);

  // If we already lost signal, 50% chance to regain it each second
  if (!validFix)
  {
    return dist(gen) > 0.5;
  }

  // If we have signal, small chance to lose it
  return dist(gen) > 0.05;
}
