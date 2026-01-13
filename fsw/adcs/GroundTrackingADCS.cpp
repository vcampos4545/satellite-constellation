#include "GroundTrackingADCS.h"
#include "Satellite.h"
#include "Constants.h"
#include <iostream>
#include <cmath>
#include <limits>

GroundTrackingADCS::GroundTrackingADCS()
    : integralError(0.0), previousError(0.0), currentTargetStation(-1)
{
  // Initialize ground station database
  // For now, hardcoded major ground stations
  // TODO: Load from universe or configuration file

  // Major global ground stations (approximate locations)
  groundStations = {
      {"Svalbard", glm::dvec3(0.0), 78.23, 15.39},       // Norway
      {"Alaska", glm::dvec3(0.0), 64.86, -147.85},       // USA
      {"Wallops", glm::dvec3(0.0), 37.94, -75.46},       // USA
      {"Kourou", glm::dvec3(0.0), 5.25, -52.77},         // French Guiana
      {"McMurdo", glm::dvec3(0.0), -77.85, 166.67},      // Antarctica
      {"Perth", glm::dvec3(0.0), -31.95, 115.86},        // Australia
      {"Santiago", glm::dvec3(0.0), -33.45, -70.67},     // Chile
      {"Kiruna", glm::dvec3(0.0), 67.86, 21.06},         // Sweden
      {"Hartebeesthoek", glm::dvec3(0.0), -25.89, 27.69} // South Africa
  };
}

void GroundTrackingADCS::reset()
{
  integralError = glm::dvec3(0.0);
  previousError = glm::dvec3(0.0);
  currentTargetStation = -1;
  currentTargetPosition = glm::dvec3(0.0);
  gainsAutoTuned = false;
}

void GroundTrackingADCS::execute(Satellite *satellite, double deltaTime)
{
  /**
   * Ground Tracking ADCS Main Loop
   *
   * Algorithm:
   * 0. Auto-tune PID gains (first time only)
   * 1. Estimate attitude from IMU (with shortcuts until full sensors implemented)
   * 2. Find nearest visible ground station
   * 3. Compute pointing quaternion toward that station
   * 4. Use PID control to command reaction wheels
   */

  // ========== STEP 0: AUTO-TUNE PID GAINS (FIRST TIME ONLY) ==========
  if (!gainsAutoTuned)
  {
    autoTunePIDGains(satellite);
    gainsAutoTuned = true;
    std::cout << "[ADCS] Auto-tuned PID gains: Kp=" << Kp << ", Ki=" << Ki << ", Kd=" << Kd << std::endl;
  }

  // ========== STEP 1: ATTITUDE ESTIMATION ==========
  glm::dquat estimatedAttitude = estimateAttitude(satellite, deltaTime);

  // ========== STEP 2: FIND NEAREST GROUND STATION ==========
  int nearestStation = findNearestGroundStation(satellite);

  if (nearestStation < 0)
  {
    // No ground station in view - enter safe mode (point nadir or sun)
    // For now, just maintain current attitude
    currentTargetStation = -1;
    currentTargetPosition = glm::dvec3(0.0);
    return;
  }

  // Update target if changed
  if (nearestStation != currentTargetStation)
  {
    currentTargetStation = nearestStation;
    currentTargetPosition = groundStations[nearestStation].position;
    std::cout << "[ADCS] Now tracking ground station: " << groundStations[nearestStation].name << std::endl;

    // Reset integral error when changing targets
    integralError = glm::dvec3(0.0);
  }
  else
  {
    // Update position even for same target (Earth rotates)
    currentTargetPosition = groundStations[nearestStation].position;
  }

  // ========== STEP 3: COMPUTE POINTING QUATERNION ==========
  glm::dquat targetAttitude = computePointingQuaternion(satellite, currentTargetPosition);

  // ========== STEP 4: PID CONTROL ==========
  glm::dvec3 controlTorque = computePIDTorque(satellite, estimatedAttitude, targetAttitude, deltaTime);

  // ========== STEP 5: COMMAND ACTUATORS ==========
  // Command reaction wheels to apply the desired torque
  satellite->commandReactionWheels(controlTorque, deltaTime);

  // Set commanded torque for attitude propagation (handled in Satellite::update())
  satellite->setCommandedTorque(controlTorque);
}

glm::dquat GroundTrackingADCS::estimateAttitude(Satellite *satellite, double deltaTime)
{
  /**
   * Attitude Estimation
   *
   * SHORTCUT VERSION (until sun sensor and star tracker implemented):
   * - Use truth attitude from satellite
   * - Add IMU measurement to validate sensor pipeline
   *
   * FUTURE IMPLEMENTATION:
   * - Sun sensor: determine body frame Sun direction
   * - Star tracker: determine inertial attitude
   * - Magnetometer: determine magnetic field direction
   * - Extended Kalman Filter (EKF) to fuse all sensors
   */

  // Read IMU (validates sensor is working and provides angular velocity)
  const IMU &imu = satellite->getIMU();
  glm::dvec3 measuredAngularVelocity = imu.getLastMeasurement();

  // For now, use truth attitude as shortcut
  // TODO: Implement full attitude determination from sun sensor + star tracker
  glm::dquat truthAttitude = satellite->getQuaternion();

  // In a real implementation, we would:
  // 1. Get sun vector in body frame from sun sensor
  // 2. Get star positions from star tracker
  // 3. Run TRIAD or QUEST algorithm to get attitude
  // 4. Fuse with IMU angular rates using EKF
  //
  // For now, return truth attitude
  return truthAttitude;
}

int GroundTrackingADCS::findNearestGroundStation(Satellite *satellite)
{
  /**
   * Find Nearest Visible Ground Station
   *
   * Algorithm:
   * 1. Get current Earth rotation angle
   * 2. Compute ECEF position of each ground station
   * 3. Check if station is above horizon (visible)
   * 4. Find closest visible station
   */

  glm::dvec3 satellitePos = satellite->getPosition();
  double earthRotation = 0.0; // TODO: Get from universe or compute from simulation time

  double minDistance = std::numeric_limits<double>::max();
  int nearestIndex = -1;

  for (size_t i = 0; i < groundStations.size(); i++)
  {
    // Convert lat/lon to ECEF position
    glm::dvec3 stationECEF = latLonToECEF(groundStations[i].latitude, groundStations[i].longitude, earthRotation);
    groundStations[i].position = stationECEF;

    // Check if ground station is visible (above horizon)
    glm::dvec3 satToStation = stationECEF - satellitePos;
    double distance = glm::length(satToStation);

    // Vector from Earth center to satellite
    glm::dvec3 earthToSat = satellitePos; // Earth at origin
    double satAltitude = glm::length(earthToSat);

    // Horizon angle: stations below horizon are not visible
    // cos(horizon_angle) = R_earth / r_satellite
    double cosHorizonAngle = EARTH_RADIUS / satAltitude;

    // Angle between nadir (Earth center direction) and station direction
    glm::dvec3 nadirDir = -glm::normalize(earthToSat);
    glm::dvec3 stationDir = glm::normalize(satToStation);
    double cosAngle = glm::dot(nadirDir, stationDir);

    // Check if station is above horizon
    if (cosAngle > cosHorizonAngle)
    {
      // Station is visible
      if (distance < minDistance)
      {
        minDistance = distance;
        nearestIndex = (int)i;
      }
    }
  }

  return nearestIndex;
}

glm::dquat GroundTrackingADCS::computePointingQuaternion(Satellite *satellite, const glm::dvec3 &groundStationPos)
{
  /**
   * Compute Target Pointing Quaternion
   *
   * Goal: Point satellite body +Z axis toward ground station
   *
   * Algorithm:
   * 1. Compute direction vector from satellite to ground station (in inertial frame)
   * 2. Compute quaternion that rotates body +Z to align with this direction
   * 3. Constrain rotation around pointing axis (optional - use velocity vector for roll)
   */

  glm::dvec3 satellitePos = satellite->getPosition();
  glm::dvec3 satelliteVel = satellite->getVelocity();

  // Direction to ground station (inertial frame)
  glm::dvec3 targetDir = glm::normalize(groundStationPos - satellitePos);

  // Body frame: +Z should point at ground station
  glm::dvec3 bodyZ(0.0, 0.0, 1.0);

  // Compute rotation axis (perpendicular to both vectors)
  glm::dvec3 rotationAxis = glm::cross(bodyZ, targetDir);
  double sinAngle = glm::length(rotationAxis);

  // Check if already aligned or anti-aligned
  if (sinAngle < 1e-6)
  {
    // Already aligned or need 180° rotation
    if (glm::dot(bodyZ, targetDir) > 0)
    {
      // Already aligned
      return glm::dquat(1.0, 0.0, 0.0, 0.0);
    }
    else
    {
      // Need 180° rotation - pick arbitrary perpendicular axis
      rotationAxis = glm::dvec3(1.0, 0.0, 0.0);
      return glm::angleAxis(PI, glm::normalize(rotationAxis));
    }
  }

  // Normalize rotation axis
  rotationAxis = glm::normalize(rotationAxis);

  // Compute rotation angle
  double cosAngle = glm::dot(bodyZ, targetDir);
  double angle = std::acos(glm::clamp(cosAngle, -1.0, 1.0));

  // Create quaternion for this rotation
  glm::dquat targetQuat = glm::angleAxis(angle, rotationAxis);

  // Optional: Constrain roll using velocity vector
  // For Earth observation, often want +X along velocity (or perpendicular to velocity)
  // This ensures consistent image orientation
  // TODO: Implement roll constraint if needed

  return targetQuat;
}

glm::dvec3 GroundTrackingADCS::computePIDTorque(Satellite *satellite,
                                                const glm::dquat &currentAttitude,
                                                const glm::dquat &targetAttitude,
                                                double deltaTime)
{
  /**
   * PID Control Law
   *
   * Computes control torque to null attitude error using PID controller.
   *
   * τ = Kp * error + Ki * ∫error dt + Kd * d(error)/dt
   */

  // Compute attitude error
  glm::dvec3 error = computeAttitudeError(currentAttitude, targetAttitude);

  // Proportional term
  glm::dvec3 proportional = Kp * error;

  // Integral term (with anti-windup)
  integralError += error * deltaTime;

  // Anti-windup: clamp integral error to prevent saturation
  const double maxIntegral = 1.0; // radians
  for (int i = 0; i < 3; i++)
  {
    if (integralError[i] > maxIntegral)
      integralError[i] = maxIntegral;
    if (integralError[i] < -maxIntegral)
      integralError[i] = -maxIntegral;
  }

  glm::dvec3 integral = Ki * integralError;

  // Derivative term (use angular velocity for better performance)
  // d(error)/dt ≈ -ω for small errors
  glm::dvec3 angularVelocity = satellite->getAngularVelocity();
  glm::dvec3 derivative = -Kd * angularVelocity;

  // Alternative: numerical derivative of error
  // glm::dvec3 errorDerivative = (error - previousError) / deltaTime;
  // glm::dvec3 derivative = Kd * errorDerivative;
  // previousError = error;

  // Total control torque
  glm::dvec3 controlTorque = proportional + integral + derivative;

  // Apply torque limits (based on reaction wheel capabilities)
  const double maxTorque = 0.01; // N·m (typical for cubesat reaction wheels)
  for (int i = 0; i < 3; i++)
  {
    if (controlTorque[i] > maxTorque)
      controlTorque[i] = maxTorque;
    if (controlTorque[i] < -maxTorque)
      controlTorque[i] = -maxTorque;
  }

  return controlTorque;
}

glm::dvec3 GroundTrackingADCS::computeAttitudeError(const glm::dquat &current, const glm::dquat &target)
{
  /**
   * Compute Attitude Error Vector
   *
   * Error quaternion: q_error = q_target * q_current^-1
   *
   * Convert to error vector (small angle approximation):
   * error_vector ≈ 2 * [x, y, z] components of q_error
   */

  // Compute error quaternion
  glm::dquat qError = target * glm::inverse(current);

  // Normalize to ensure unit quaternion
  qError = glm::normalize(qError);

  // Ensure shortest path (quaternion double cover)
  if (qError.w < 0.0)
  {
    qError = -qError;
  }

  // Extract error vector from quaternion
  // For small angles: error ≈ 2 * vector_part
  glm::dvec3 errorVector(qError.x, qError.y, qError.z);
  errorVector *= 2.0;

  return errorVector;
}

glm::dvec3 GroundTrackingADCS::latLonToECEF(double latitude, double longitude, double earthRotation)
{
  /**
   * Convert Geodetic Coordinates to ECEF
   *
   * @param latitude Latitude in degrees
   * @param longitude Longitude in degrees
   * @param earthRotation Earth rotation angle in radians
   * @return ECEF position (meters)
   */

  // Convert to radians
  double lat = glm::radians(latitude);
  double lon = glm::radians(longitude) + earthRotation;

  // ECEF coordinates (assuming spherical Earth for simplicity)
  double x = EARTH_RADIUS * cos(lat) * cos(lon);
  double y = EARTH_RADIUS * cos(lat) * sin(lon);
  double z = EARTH_RADIUS * sin(lat);

  return glm::dvec3(x, y, z);
}

void GroundTrackingADCS::autoTunePIDGains(Satellite *satellite)
{
  /**
   * Auto-Tune PID Gains for Spacecraft Attitude Control
   *
   * Method: Modified Ziegler-Nichols for spacecraft
   *
   * Theory:
   * - Spacecraft attitude dynamics: I*ω_dot = τ
   * - Natural frequency: ω_n ≈ sqrt(τ_max / I)
   * - For critically damped: ζ = 1.0
   * - For slightly underdamped: ζ = 0.7 (faster response, slight overshoot)
   *
   * PID Gains:
   * - Kp = 2*ζ*ω_n*I  (proportional to stiffness)
   * - Kd = 2*ζ*I       (damping)
   * - Ki = ω_n²*I      (removes steady-state error)
   *
   * For spacecraft, we want:
   * - Fast response (high ω_n)
   * - Good damping (ζ ≈ 0.7-1.0)
   * - Minimal overshoot
   */

  // Get satellite inertia tensor (diagonal elements)
  glm::dvec3 inertia = satellite->getInertiaTensor();

  // Use average inertia for tuning
  double I_avg = (inertia.x + inertia.y + inertia.z) / 3.0;

  // Estimate maximum available torque from reaction wheels
  double tau_max = satellite->getReactionWheelMaxTorque();

  // Compute natural frequency
  // ω_n = sqrt(τ_max / I)
  double omega_n = sqrt(tau_max / I_avg);

  // Choose damping ratio
  // ζ = 0.7 gives good balance between speed and overshoot (slightly underdamped)
  // ζ = 1.0 would be critically damped (no overshoot, but slower)
  double zeta = 0.7;

  // Compute PID gains
  // Kp: Proportional gain (stiffness of control)
  Kp = 2.0 * zeta * omega_n * I_avg;

  // Kd: Derivative gain (damping)
  Kd = 2.0 * zeta * I_avg;

  // Ki: Integral gain (removes steady-state error)
  // Use smaller gain to prevent integral windup
  Ki = 0.1 * omega_n * omega_n * I_avg;

  // Apply safety bounds to prevent instability
  // These bounds are based on typical spacecraft values
  const double Kp_min = 0.01, Kp_max = 10.0;
  const double Ki_min = 0.001, Ki_max = 1.0;
  const double Kd_min = 0.01, Kd_max = 10.0;

  Kp = glm::clamp(Kp, Kp_min, Kp_max);
  Ki = glm::clamp(Ki, Ki_min, Ki_max);
  Kd = glm::clamp(Kd, Kd_min, Kd_max);
}
