#ifndef SUN_SENSOR_H
#define SUN_SENSOR_H

#include "Component.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <random>

/**
 * Sun Sensor Component
 *
 * Simulates a sun sensor that measures the direction to the sun in the
 * spacecraft body frame. Sun sensors are critical for attitude determination
 * and solar panel pointing.
 *
 * Sensor Types Modeled:
 * - Coarse Sun Sensor (CSS): ~1-5 degree accuracy, wide FOV (~120 degrees)
 * - Fine Sun Sensor (FSS): ~0.01-0.1 degree accuracy, narrow FOV (~60 degrees)
 * - Digital Sun Sensor (DSS): ~0.1-0.5 degree accuracy, moderate FOV
 *
 * Features:
 * - Field of view (FOV) limiting - only detects sun within FOV cone
 * - Angular measurement noise (Gaussian)
 * - Eclipse detection (sun blocked by Earth)
 * - Intensity measurement (useful for eclipse entry/exit detection)
 * - Configurable mounting orientation (boresight axis)
 *
 * Output:
 * - Sun vector in body frame (unit vector)
 * - Sun intensity (0-1, accounts for eclipse)
 * - Validity flag (sun in FOV and not eclipsed)
 *
 * Usage:
 *   auto sunSensor = spacecraft->addComponent<SunSensor>("CSS");
 *   sunSensor->setFieldOfView(120.0);  // 120 degree full cone angle
 *   sunSensor->setAccuracy(2.0);       // 2 degree 1-sigma noise
 *   sunSensor->measure(sunPosECI, spacecraftPosECI, attitude, earthPosECI, earthRadius);
 *   if (sunSensor->isSunVisible()) {
 *     glm::dvec3 sunDir = sunSensor->getSunVector();
 *   }
 */
class SunSensor : public Sensor
{
public:
  /**
   * Sensor type presets
   */
  enum class Type
  {
    COARSE,  // Wide FOV, low accuracy (CSS)
    FINE,    // Narrow FOV, high accuracy (FSS)
    DIGITAL  // Moderate FOV and accuracy (DSS)
  };

  /**
   * Default constructor - creates a coarse sun sensor
   */
  SunSensor();

  /**
   * Named constructor
   * @param name Component name
   */
  explicit SunSensor(const std::string &name);

  /**
   * Constructor with sensor type preset
   * @param type Sensor type (COARSE, FINE, DIGITAL)
   * @param name Component name
   */
  SunSensor(Type type, const std::string &name = "SunSensor");

  // ========== SENSOR INTERFACE ==========

  /**
   * Get component type name
   */
  std::string getTypeName() const override { return "SunSensor"; }

  /**
   * Dummy measure() override for Sensor interface
   * Use measure() with parameters instead
   */
  void measure() override {}

  /**
   * Take sun sensor measurement
   *
   * @param sunPositionECI Sun position in ECI frame (meters)
   * @param spacecraftPositionECI Spacecraft position in ECI frame (meters)
   * @param attitude Spacecraft attitude quaternion (body to ECI)
   * @param earthPositionECI Earth center position in ECI frame (meters)
   * @param earthRadius Earth radius for eclipse calculation (meters)
   */
  void measure(
      const glm::dvec3 &sunPositionECI,
      const glm::dvec3 &spacecraftPositionECI,
      const glm::dquat &attitude,
      const glm::dvec3 &earthPositionECI,
      double earthRadius);

  // ========== MEASUREMENT GETTERS ==========

  /**
   * Get measured sun vector in body frame
   * Only valid if isSunVisible() returns true
   * @return Unit vector pointing toward sun in body frame
   */
  glm::dvec3 getSunVector() const { return measuredSunVector; }

  /**
   * Get true sun vector in body frame (without noise)
   * For debugging/comparison purposes
   * @return True unit vector pointing toward sun in body frame
   */
  glm::dvec3 getTrueSunVector() const { return trueSunVector; }

  /**
   * Check if sun is currently visible to sensor
   * False if sun is outside FOV or spacecraft is in eclipse
   * @return True if valid sun measurement available
   */
  bool isSunVisible() const { return sunVisible; }

  /**
   * Check if spacecraft is in Earth's shadow (eclipse)
   * @return True if spacecraft is eclipsed
   */
  bool isInEclipse() const { return inEclipse; }

  /**
   * Get sun intensity (0-1)
   * Accounts for partial eclipse (penumbra)
   * @return Sun intensity factor (0 = full eclipse, 1 = full sun)
   */
  double getSunIntensity() const { return sunIntensity; }

  /**
   * Get angle between sun and sensor boresight
   * @return Angle in degrees (0 = sun on boresight)
   */
  double getSunAngle() const { return sunAngle; }

  /**
   * Get measurement noise (angular accuracy)
   * @return 1-sigma accuracy in degrees
   */
  double getAccuracy() const { return angularAccuracy; }

  /**
   * Get field of view (full cone angle)
   * @return FOV in degrees
   */
  double getFieldOfView() const { return fieldOfView; }

  // ========== CONFIGURATION SETTERS ==========

  /**
   * Set field of view (full cone angle)
   * @param fovDegrees FOV in degrees (e.g., 120 for ±60 degree half-angle)
   */
  void setFieldOfView(double fovDegrees) { fieldOfView = fovDegrees; }

  /**
   * Set angular measurement accuracy
   * @param accuracyDegrees 1-sigma accuracy in degrees
   */
  void setAccuracy(double accuracyDegrees) { angularAccuracy = accuracyDegrees; }

  /**
   * Set sensor boresight axis in body frame
   * Default is +X axis (1,0,0)
   * @param axis Unit vector defining sensor pointing direction
   */
  void setBoresightAxis(const glm::dvec3 &axis) { boresightAxis = glm::normalize(axis); }

  /**
   * Get sensor boresight axis in body frame
   * @return Unit vector defining sensor pointing direction
   */
  glm::dvec3 getBoresightAxis() const { return boresightAxis; }

  /**
   * Enable/disable eclipse calculation
   * @param enable True to calculate eclipse, false to assume always sunlit
   */
  void setEclipseCalculation(bool enable) { calculateEclipse = enable; }

private:
  // ========== SENSOR CHARACTERISTICS ==========
  double fieldOfView;      // Full cone angle in degrees
  double angularAccuracy;  // 1-sigma measurement noise in degrees
  glm::dvec3 boresightAxis; // Sensor pointing direction in body frame

  // ========== MEASUREMENT STATE ==========
  glm::dvec3 measuredSunVector;  // Measured sun direction (body frame, unit vector)
  glm::dvec3 trueSunVector;      // True sun direction (body frame, unit vector)
  bool sunVisible;               // True if sun is in FOV and not eclipsed
  bool inEclipse;                // True if spacecraft is in Earth's shadow
  double sunIntensity;           // Sun intensity factor (0-1)
  double sunAngle;               // Angle from boresight to sun (degrees)

  // ========== CONFIGURATION ==========
  bool calculateEclipse;  // Whether to calculate eclipse

  // ========== RANDOM NUMBER GENERATION ==========
  std::mt19937 rng;
  std::normal_distribution<double> noiseDist;

  // ========== HELPER METHODS ==========

  /**
   * Check if spacecraft is in Earth's shadow
   * Uses cylindrical shadow model (umbra only, no penumbra for simplicity)
   *
   * @param sunPos Sun position (ECI)
   * @param scPos Spacecraft position (ECI)
   * @param earthPos Earth position (ECI)
   * @param earthRadius Earth radius (meters)
   * @return True if spacecraft is eclipsed
   */
  bool checkEclipse(
      const glm::dvec3 &sunPos,
      const glm::dvec3 &scPos,
      const glm::dvec3 &earthPos,
      double earthRadius);

  /**
   * Add angular noise to sun vector measurement
   * Rotates the vector by a small random angle
   *
   * @param trueVector True sun vector (unit)
   * @return Noisy sun vector (unit)
   */
  glm::dvec3 addAngularNoise(const glm::dvec3 &trueVector);
};

#endif // SUN_SENSOR_H
