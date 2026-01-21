#include "AttitudeEstimator.h"
#include <cmath>
#include <cstring>

AttitudeEstimator::AttitudeEstimator()
    : attitude(1.0, 0.0, 0.0, 0.0),
      angularVelocity(0.0),
      gyroBias(0.0),
      gyroNoiseStdDev(0.001),     // 0.001 rad/s/sqrt(Hz) typical MEMS gyro
      gyroBiasStdDev(0.0001),     // 0.0001 rad/s^2/sqrt(Hz) typical bias drift
      attitudeMeasStdDev(0.01),   // 0.01 rad (~0.6 deg) typical TRIAD accuracy
      initialized(false)
{
  // Initialize covariance to identity (high initial uncertainty)
  std::memset(P, 0, sizeof(P));
  for (int i = 0; i < 6; i++)
  {
    P[i][i] = 0.1; // Initial uncertainty: ~18 degrees attitude, ~0.3 rad/s bias
  }
}

void AttitudeEstimator::initialize(const glm::dquat &initialAttitude)
{
  attitude = glm::normalize(initialAttitude);
  angularVelocity = glm::dvec3(0.0);
  gyroBias = glm::dvec3(0.0);

  // Reset covariance to moderate values
  std::memset(P, 0, sizeof(P));
  for (int i = 0; i < 3; i++)
  {
    P[i][i] = 0.01;       // Attitude uncertainty: ~6 degrees
    P[i + 3][i + 3] = 0.001; // Bias uncertainty: ~0.03 rad/s
  }

  initialized = true;
}

void AttitudeEstimator::reset()
{
  attitude = glm::dquat(1.0, 0.0, 0.0, 0.0);
  angularVelocity = glm::dvec3(0.0);
  gyroBias = glm::dvec3(0.0);

  std::memset(P, 0, sizeof(P));
  for (int i = 0; i < 6; i++)
  {
    P[i][i] = 0.1;
  }

  initialized = false;
}

void AttitudeEstimator::propagate(const glm::dvec3 &gyroMeasurement, double deltaTime)
{
  if (!initialized)
  {
    return;
  }

  // Correct gyro measurement for estimated bias
  glm::dvec3 omega = gyroMeasurement - gyroBias;
  angularVelocity = omega;

  // Propagate attitude using quaternion kinematics
  // dq/dt = 0.5 * omega_quat * q
  glm::dquat omegaQuat(0.0, omega.x, omega.y, omega.z);
  glm::dquat qdot = 0.5 * omegaQuat * attitude;

  // First-order integration (could use RK4 for better accuracy)
  attitude = attitude + qdot * deltaTime;
  attitude = glm::normalize(attitude);

  // Propagate covariance: P = F * P * F^T + Q
  double F[6][6];
  double Q[6][6];
  computeStateTransition(omega, deltaTime, F);
  computeProcessNoise(deltaTime, Q);

  // P_new = F * P * F^T + Q
  double temp[6][6];
  double P_new[6][6];

  // temp = F * P
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      temp[i][j] = 0.0;
      for (int k = 0; k < 6; k++)
      {
        temp[i][j] += F[i][k] * P[k][j];
      }
    }
  }

  // P_new = temp * F^T + Q
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      P_new[i][j] = Q[i][j];
      for (int k = 0; k < 6; k++)
      {
        P_new[i][j] += temp[i][k] * F[j][k]; // F^T[k][j] = F[j][k]
      }
    }
  }

  std::memcpy(P, P_new, sizeof(P));
}

void AttitudeEstimator::update(const glm::dvec3 &refVector1_inertial,
                                const glm::dvec3 &refVector2_inertial,
                                const glm::dvec3 &measVector1_body,
                                const glm::dvec3 &measVector2_body)
{
  // Compute TRIAD attitude measurement
  glm::dquat measuredAttitude = triad(refVector1_inertial, refVector2_inertial,
                                       measVector1_body, measVector2_body);

  if (!initialized)
  {
    // First measurement - initialize directly
    initialize(measuredAttitude);
    return;
  }

  // Compute attitude error (innovation)
  // error_quat = q_measured * q_estimated^-1
  glm::dquat errorQuat = measuredAttitude * glm::inverse(attitude);
  errorQuat = glm::normalize(errorQuat);

  // Ensure shortest path (scalar part positive)
  if (errorQuat.w < 0.0)
  {
    errorQuat = -errorQuat;
  }

  // Convert to rotation vector (small angle approximation valid for small errors)
  glm::dvec3 innovation = quaternionToRotationVector(errorQuat);

  // Kalman update
  // Measurement matrix H = [I(3x3), 0(3x3)] (attitude measurement, no direct bias observation)
  // Innovation covariance: S = H * P * H^T + R
  // For attitude-only measurement: S = P[0:3, 0:3] + R

  double R = attitudeMeasStdDev * attitudeMeasStdDev; // Measurement variance

  // S (3x3) = P[0:3, 0:3] + R*I
  double S[3][3];
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      S[i][j] = P[i][j];
      if (i == j)
      {
        S[i][j] += R;
      }
    }
  }

  // Compute S inverse (3x3 matrix inversion)
  double detS = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1]) -
                S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0]) +
                S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);

  if (std::abs(detS) < 1e-12)
  {
    // Singular matrix - skip update
    return;
  }

  double S_inv[3][3];
  S_inv[0][0] = (S[1][1] * S[2][2] - S[1][2] * S[2][1]) / detS;
  S_inv[0][1] = (S[0][2] * S[2][1] - S[0][1] * S[2][2]) / detS;
  S_inv[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) / detS;
  S_inv[1][0] = (S[1][2] * S[2][0] - S[1][0] * S[2][2]) / detS;
  S_inv[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) / detS;
  S_inv[1][2] = (S[0][2] * S[1][0] - S[0][0] * S[1][2]) / detS;
  S_inv[2][0] = (S[1][0] * S[2][1] - S[1][1] * S[2][0]) / detS;
  S_inv[2][1] = (S[0][1] * S[2][0] - S[0][0] * S[2][1]) / detS;
  S_inv[2][2] = (S[0][0] * S[1][1] - S[0][1] * S[1][0]) / detS;

  // Kalman gain: K = P * H^T * S^-1
  // K (6x3) = P[6x6] * H^T[6x3] * S_inv[3x3]
  // H^T = [I(3x3); 0(3x3)]
  double K[6][3];
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      K[i][j] = 0.0;
      for (int k = 0; k < 3; k++)
      {
        K[i][j] += P[i][k] * S_inv[k][j];
      }
    }
  }

  // State correction: dx = K * innovation
  glm::dvec3 attitudeCorrection(0.0);
  glm::dvec3 biasCorrection(0.0);

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      attitudeCorrection[i] += K[i][j] * innovation[j];
      biasCorrection[i] += K[i + 3][j] * innovation[j];
    }
  }

  // Apply attitude correction (multiplicative)
  glm::dquat correctionQuat = rotationVectorToQuaternion(attitudeCorrection);
  attitude = correctionQuat * attitude;
  attitude = glm::normalize(attitude);

  // Apply bias correction (additive)
  gyroBias += biasCorrection;

  // Update covariance: P = (I - K*H) * P
  // Simplified: P_new[i][j] = P[i][j] - sum_k(K[i][k] * P[k][j]) for k < 3
  double P_new[6][6];
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      P_new[i][j] = P[i][j];
      for (int k = 0; k < 3; k++)
      {
        P_new[i][j] -= K[i][k] * P[k][j];
      }
    }
  }

  std::memcpy(P, P_new, sizeof(P));
}

glm::dquat AttitudeEstimator::triad(const glm::dvec3 &r1, const glm::dvec3 &r2,
                                     const glm::dvec3 &b1, const glm::dvec3 &b2)
{
  /**
   * TRIAD Algorithm
   *
   * Constructs orthonormal triads in both reference and body frames,
   * then computes the rotation matrix from body to inertial.
   *
   * Reference triad (inertial frame):
   *   t1_r = r1
   *   t2_r = (r1 x r2) / |r1 x r2|
   *   t3_r = t1_r x t2_r
   *
   * Body triad (body frame):
   *   t1_b = b1
   *   t2_b = (b1 x b2) / |b1 x b2|
   *   t3_b = t1_b x t2_b
   *
   * Rotation matrix: R = [t1_r t2_r t3_r] * [t1_b t2_b t3_b]^T
   */

  // Normalize input vectors
  glm::dvec3 r1n = glm::normalize(r1);
  glm::dvec3 r2n = glm::normalize(r2);
  glm::dvec3 b1n = glm::normalize(b1);
  glm::dvec3 b2n = glm::normalize(b2);

  // Build reference triad (inertial frame)
  glm::dvec3 t1_r = r1n;
  glm::dvec3 t2_r = glm::normalize(glm::cross(r1n, r2n));
  glm::dvec3 t3_r = glm::cross(t1_r, t2_r);

  // Build body triad (body frame)
  glm::dvec3 t1_b = b1n;
  glm::dvec3 t2_b = glm::normalize(glm::cross(b1n, b2n));
  glm::dvec3 t3_b = glm::cross(t1_b, t2_b);

  // Construct rotation matrix R_body_to_inertial
  // R = M_r * M_b^T where M_r = [t1_r | t2_r | t3_r], M_b = [t1_b | t2_b | t3_b]
  glm::dmat3 M_r(t1_r, t2_r, t3_r);       // Columns are the triad vectors
  glm::dmat3 M_b(t1_b, t2_b, t3_b);
  glm::dmat3 R = M_r * glm::transpose(M_b);

  // Convert rotation matrix to quaternion
  return glm::quat_cast(R);
}

glm::dvec3 AttitudeEstimator::getAttitudeCovariance() const
{
  return glm::dvec3(P[0][0], P[1][1], P[2][2]);
}

void AttitudeEstimator::computeStateTransition(const glm::dvec3 &omega, double dt, double F[6][6])
{
  /**
   * State transition matrix for MEKF
   *
   * State: [attitude_error (3), gyro_bias (3)]
   *
   * Attitude error dynamics: d(delta_theta)/dt = -[omega x] * delta_theta - delta_bias
   * Bias dynamics: d(delta_bias)/dt = 0 (random walk modeled in process noise)
   *
   * Continuous: A = [-[omega x], -I; 0, 0]
   * Discrete: F ≈ I + A*dt (first-order approximation)
   */

  // Initialize to identity
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      F[i][j] = (i == j) ? 1.0 : 0.0;
    }
  }

  // Attitude error block: I - [omega x] * dt
  // [omega x] = [  0   -wz   wy ]
  //             [  wz   0   -wx ]
  //             [ -wy   wx   0  ]
  F[0][1] = omega.z * dt;
  F[0][2] = -omega.y * dt;
  F[1][0] = -omega.z * dt;
  F[1][2] = omega.x * dt;
  F[2][0] = omega.y * dt;
  F[2][1] = -omega.x * dt;

  // Coupling: attitude_error affected by bias_error
  // -I * dt
  F[0][3] = -dt;
  F[1][4] = -dt;
  F[2][5] = -dt;
}

void AttitudeEstimator::computeProcessNoise(double dt, double Q[6][6])
{
  /**
   * Process noise covariance
   *
   * Attitude noise from gyro: sigma_gyro^2 * dt
   * Bias noise (random walk): sigma_bias^2 * dt
   */

  std::memset(Q, 0, 6 * 6 * sizeof(double));

  double gyroVar = gyroNoiseStdDev * gyroNoiseStdDev * dt;
  double biasVar = gyroBiasStdDev * gyroBiasStdDev * dt;

  // Attitude noise
  Q[0][0] = gyroVar;
  Q[1][1] = gyroVar;
  Q[2][2] = gyroVar;

  // Bias noise
  Q[3][3] = biasVar;
  Q[4][4] = biasVar;
  Q[5][5] = biasVar;
}

glm::dvec3 AttitudeEstimator::quaternionToRotationVector(const glm::dquat &q)
{
  // Convert quaternion to rotation vector (axis * angle)
  // For unit quaternion: q = [cos(theta/2), sin(theta/2)*axis]

  double sinHalfAngle = glm::length(glm::dvec3(q.x, q.y, q.z));

  if (sinHalfAngle < 1e-10)
  {
    // Small angle: rotation vector ≈ 2 * vector_part
    return 2.0 * glm::dvec3(q.x, q.y, q.z);
  }

  double halfAngle = std::atan2(sinHalfAngle, q.w);
  double angle = 2.0 * halfAngle;
  glm::dvec3 axis = glm::dvec3(q.x, q.y, q.z) / sinHalfAngle;

  return angle * axis;
}

glm::dquat AttitudeEstimator::rotationVectorToQuaternion(const glm::dvec3 &rv)
{
  // Convert rotation vector to quaternion
  double angle = glm::length(rv);

  if (angle < 1e-10)
  {
    // Small rotation: q ≈ [1, rv/2]
    return glm::normalize(glm::dquat(1.0, rv.x / 2.0, rv.y / 2.0, rv.z / 2.0));
  }

  glm::dvec3 axis = rv / angle;
  double halfAngle = angle / 2.0;

  return glm::dquat(std::cos(halfAngle),
                    std::sin(halfAngle) * axis.x,
                    std::sin(halfAngle) * axis.y,
                    std::sin(halfAngle) * axis.z);
}
