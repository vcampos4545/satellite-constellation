#ifndef GPS_H
#define GPS_H

#include "Component.h"
#include <glm/glm.hpp>
#include <string>

/**
 * GPS Sensor Component
 *
 * Simulates a GPS/GNSS receiver providing position and velocity measurements.
 * Models realistic sensor characteristics including noise, accuracy, and signal availability.
 *
 * Typical GPS Performance:
 * - Civilian GPS (L1 C/A): ~5-10m position accuracy, ~0.1-0.5 m/s velocity accuracy
 * - Dual-frequency GPS: ~1-3m position accuracy, ~0.05 m/s velocity accuracy
 * - Update rate: 1-10 Hz
 *
 * Features:
 * - Gaussian noise on position and velocity
 * - Configurable accuracy and update rate
 * - Signal availability simulation
 * - Number of satellites tracked
 * - Dilution of Precision (DOP) effects
 *
 * Usage:
 *   auto gps = spacecraft->addComponent<GPS>("gps");
 *   gps->setPositionAccuracy(5.0);  // 5m standard deviation
 *   gps->measure(truthPosition, truthVelocity);
 *   glm::dvec3 measuredPos = gps->getPosition();
 */
class GPS : public Sensor
{
public:
  /**
   * Default constructor - civilian GPS accuracy
   */
  GPS();

  /**
   * Named constructor
   * @param name Component name
   */
  GPS(const std::string &name);

  /**
   * Constructor with custom accuracy
   * @param posAccuracy Position standard deviation (meters)
   * @param velAccuracy Velocity standard deviation (m/s)
   * @param updateRate Measurement update rate (Hz)
   */
  GPS(double posAccuracy, double velAccuracy, double updateRate = 1.0);

  // ========== SENSOR INTERFACE ==========

  /**
   * Update sensor (accumulates time for update rate limiting)
   * @param deltaTime Time step (seconds)
   */
  void update(double deltaTime) override;

  /**
   * Take measurement with current truth state
   * Adds noise to truth position and velocity
   * @param truthPosition True position (ECI frame, meters)
   * @param truthVelocity True velocity (ECI frame, m/s)
   */
  void measure(const glm::dvec3 &truthPosition, const glm::dvec3 &truthVelocity);

  /**
   * Dummy measure() override for Sensor interface
   */
  void measure() override {}

  /**
   * Get component type name
   */
  std::string getTypeName() const override { return "GPS"; }

  // ========== MEASUREMENT GETTERS ==========

  /**
   * Get measured position (with noise)
   * @return Position measurement in ECI frame (meters)
   */
  glm::dvec3 getPosition() const { return measuredPosition; }

  /**
   * Get measured velocity (with noise)
   * @return Velocity measurement in ECI frame (m/s)
   */
  glm::dvec3 getVelocity() const { return measuredVelocity; }

  /**
   * Get position accuracy (1-sigma standard deviation)
   * @return Position accuracy (meters)
   */
  double getPositionAccuracy() const { return positionAccuracy; }

  /**
   * Get velocity accuracy (1-sigma standard deviation)
   * @return Velocity accuracy (m/s)
   */
  double getVelocityAccuracy() const { return velocityAccuracy; }

  /**
   * Check if GPS has valid fix
   * @return True if GPS is locked and providing valid measurements
   */
  bool hasValidFix() const { return validFix; }

  /**
   * Get number of satellites currently tracked
   * @return Number of GPS satellites in view
   */
  int getNumSatellites() const { return numSatellites; }

  /**
   * Get Geometric Dilution of Precision
   * Lower is better (1.0 = ideal, >5.0 = poor)
   * @return GDOP value
   */
  double getGDOP() const { return gdop; }

  /**
   * Get time since last measurement update
   * @return Time since last update (seconds)
   */
  double getTimeSinceUpdate() const { return timeSinceUpdate; }

  /**
   * Get configured update rate
   * @return Update rate (Hz)
   */
  double getUpdateRate() const { return updateRate; }

  // ========== CONFIGURATION SETTERS ==========

  /**
   * Set position accuracy
   * @param accuracy Position standard deviation (meters)
   */
  void setPositionAccuracy(double accuracy) { positionAccuracy = accuracy; }

  /**
   * Set velocity accuracy
   * @param accuracy Velocity standard deviation (m/s)
   */
  void setVelocityAccuracy(double accuracy) { velocityAccuracy = accuracy; }

  /**
   * Set measurement update rate
   * @param rate Update rate (Hz)
   */
  void setUpdateRate(double rate) { updateRate = rate; }

  /**
   * Enable/disable signal availability simulation
   * When enabled, GPS may lose signal based on geometry
   * @param enable True to enable signal loss simulation
   */
  void setSignalLossSimulation(bool enable) { simulateSignalLoss = enable; }

  /**
   * Set number of satellites tracked
   * Affects DOP and measurement quality
   * @param numSats Number of satellites (4-12 typical)
   */
  void setNumSatellites(int numSats) { numSatellites = numSats; updateGDOP(); }

private:
  // ========== MEASUREMENT DATA ==========
  glm::dvec3 measuredPosition; // Measured position with noise (ECI frame, meters)
  glm::dvec3 measuredVelocity; // Measured velocity with noise (ECI frame, m/s)

  // ========== SENSOR CHARACTERISTICS ==========
  double positionAccuracy;  // Position 1-sigma standard deviation (meters)
  double velocityAccuracy;  // Velocity 1-sigma standard deviation (m/s)
  double updateRate;        // Measurement update rate (Hz)
  double timeSinceUpdate;   // Time accumulator for rate limiting (seconds)

  // ========== SIGNAL QUALITY ==========
  bool validFix;            // True if GPS has valid position fix
  int numSatellites;        // Number of GPS satellites tracked (4-12 typical)
  double gdop;              // Geometric Dilution of Precision (1.0-10.0)
  bool simulateSignalLoss;  // Enable signal loss simulation

  // ========== HELPER METHODS ==========

  /**
   * Add Gaussian noise to measurement
   * @param value True value
   * @param stdDev Standard deviation of noise
   * @return Noisy measurement
   */
  double addNoise(double value, double stdDev);

  /**
   * Add Gaussian noise to vector measurement
   * @param value True vector value
   * @param stdDev Standard deviation of noise (applied per component)
   * @return Noisy measurement vector
   */
  glm::dvec3 addNoise(const glm::dvec3 &value, double stdDev);

  /**
   * Update GDOP based on number of satellites
   * More satellites = better geometry = lower GDOP
   */
  void updateGDOP();

  /**
   * Simulate signal availability
   * GPS may lose signal in certain conditions
   * @return True if signal is available
   */
  bool checkSignalAvailability();
};

#endif // GPS_H
