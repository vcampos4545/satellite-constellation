#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

/**
 * AttitudeEstimator
 *
 * Implements attitude determination using:
 * 1. TRIAD Algorithm - Deterministic attitude from two vector measurements
 * 2. Multiplicative Extended Kalman Filter (MEKF) - Fuses gyro with TRIAD
 *
 * TRIAD Algorithm:
 * Given two reference vectors (r1, r2) in inertial frame and their
 * measurements (b1, b2) in body frame, TRIAD constructs orthonormal
 * triads in both frames and computes the rotation matrix between them.
 *
 * MEKF:
 * State vector: [attitude_error (3), gyro_bias (3)]
 * - Propagation: Integrates gyro measurements (corrected for bias)
 * - Update: Uses TRIAD attitude measurement
 *
 * The MEKF uses the multiplicative formulation where the attitude error
 * is represented as a small rotation from the estimated attitude.
 *
 * References:
 * - Markley & Crassidis, "Fundamentals of Spacecraft Attitude Determination and Control"
 * - Shuster & Oh, "Three-Axis Attitude Determination from Vector Observations" (TRIAD)
 */
class AttitudeEstimator
{
public:
  AttitudeEstimator();

  /**
   * Initialize the estimator with known attitude
   * @param initialAttitude Initial attitude quaternion (body to inertial)
   */
  void initialize(const glm::dquat &initialAttitude);

  /**
   * Reset the estimator state
   */
  void reset();

  /**
   * Propagate attitude estimate using gyroscope measurement
   * Call this at the gyro update rate (typically 10-100 Hz)
   *
   * @param gyroMeasurement Angular velocity measurement in body frame (rad/s)
   * @param deltaTime Time step (seconds)
   */
  void propagate(const glm::dvec3 &gyroMeasurement, double deltaTime);

  /**
   * Update attitude estimate with vector measurements using TRIAD + Kalman
   * Call this at the sensor update rate (typically 1-10 Hz)
   *
   * @param refVector1_inertial First reference vector in inertial frame (normalized)
   * @param refVector2_inertial Second reference vector in inertial frame (normalized)
   * @param measVector1_body First measured vector in body frame (normalized)
   * @param measVector2_body Second measured vector in body frame (normalized)
   */
  void update(const glm::dvec3 &refVector1_inertial,
              const glm::dvec3 &refVector2_inertial,
              const glm::dvec3 &measVector1_body,
              const glm::dvec3 &measVector2_body);

  /**
   * Get current attitude estimate
   * @return Estimated attitude quaternion (body to inertial)
   */
  glm::dquat getAttitude() const { return attitude; }

  /**
   * Get current angular velocity estimate (bias-corrected)
   * @return Estimated angular velocity in body frame (rad/s)
   */
  glm::dvec3 getAngularVelocity() const { return angularVelocity; }

  /**
   * Get estimated gyroscope bias
   * @return Estimated gyro bias in body frame (rad/s)
   */
  glm::dvec3 getGyroBias() const { return gyroBias; }

  /**
   * Get attitude covariance (diagonal elements)
   * @return 3x1 vector of attitude variance (rad^2)
   */
  glm::dvec3 getAttitudeCovariance() const;

  /**
   * TRIAD Algorithm - Static method for standalone use
   *
   * Computes attitude from two vector measurements.
   * The first vector is used as the primary reference.
   *
   * @param r1 First reference vector in inertial frame (normalized)
   * @param r2 Second reference vector in inertial frame (normalized)
   * @param b1 First measured vector in body frame (normalized)
   * @param b2 Second measured vector in body frame (normalized)
   * @return Attitude quaternion (body to inertial)
   */
  static glm::dquat triad(const glm::dvec3 &r1, const glm::dvec3 &r2,
                          const glm::dvec3 &b1, const glm::dvec3 &b2);

  // Configuration
  void setGyroNoiseStdDev(double sigma) { gyroNoiseStdDev = sigma; }
  void setGyroBiasStdDev(double sigma) { gyroBiasStdDev = sigma; }
  void setAttitudeMeasurementStdDev(double sigma) { attitudeMeasStdDev = sigma; }

  // Status
  bool isInitialized() const { return initialized; }

private:
  // State estimates
  glm::dquat attitude;        // Estimated attitude (body to inertial)
  glm::dvec3 angularVelocity; // Estimated angular velocity (body frame)
  glm::dvec3 gyroBias;        // Estimated gyroscope bias (body frame)

  // Covariance matrix (6x6: [attitude_error(3), gyro_bias(3)])
  // Stored as array for simplicity (symmetric matrix)
  double P[6][6];

  // Process noise parameters
  double gyroNoiseStdDev;  // Gyro measurement noise (rad/s/sqrt(Hz))
  double gyroBiasStdDev;   // Gyro bias random walk (rad/s^2/sqrt(Hz))

  // Measurement noise parameters
  double attitudeMeasStdDev; // Attitude measurement noise (rad)

  // Status
  bool initialized;

  /**
   * Compute state transition matrix for attitude error dynamics
   * @param omega Angular velocity (rad/s)
   * @param dt Time step (seconds)
   * @param F Output 6x6 state transition matrix
   */
  void computeStateTransition(const glm::dvec3 &omega, double dt, double F[6][6]);

  /**
   * Compute process noise covariance matrix
   * @param dt Time step (seconds)
   * @param Q Output 6x6 process noise covariance
   */
  void computeProcessNoise(double dt, double Q[6][6]);

  /**
   * Extract rotation vector (axis-angle) from quaternion error
   * @param q_error Error quaternion
   * @return Rotation vector (angle * axis)
   */
  static glm::dvec3 quaternionToRotationVector(const glm::dquat &q_error);

  /**
   * Create quaternion from small rotation vector
   * @param rv Rotation vector (angle * axis), assumed small
   * @return Quaternion representing the rotation
   */
  static glm::dquat rotationVectorToQuaternion(const glm::dvec3 &rv);
};

#endif // ATTITUDE_ESTIMATOR_H
