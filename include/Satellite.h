#ifndef SATELLITE_H
#define SATELLITE_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include "Config.h"

// Attitude control modes
enum class AttitudeControlMode
{
  NONE,              // No active control (tumbling)
  DETUMBLE,          // Reduce angular velocity to zero using magnetorquers
  NADIR_POINTING,    // Point one axis toward Earth center
  SUN_POINTING,      // Point one axis toward Sun
  VELOCITY_POINTING, // Point one axis along velocity vector
  INERTIAL_HOLD,     // Maintain fixed orientation in inertial space
  TARGET_TRACKING    // Point toward a specific target
};

// Control algorithm types for reaction wheels
enum class ControlAlgorithm
{
  PID, // PID controller (default)
  LQR, // Linear Quadratic Regulator
  MPC  // Model Predictive Control
};

class Satellite
{
public:
  Satellite(const Orbit orbit, const glm::dvec3 &position, const glm::dvec3 &velocity, const glm::vec3 &color, int planeId = 0, int indexInPlane = 0);

  // Update physics
  void update(double deltaTime, const glm::dvec3 &earthCenter, double earthMass, const glm::dvec3 &sunPosition, const glm::dvec3 &moonPosition);

  // Calculate complete orbital path (full orbit prediction)
  void calculateOrbitPath(int numPoints = 100);

  // ADCS Control Loop (mirrors flight software architecture)
  void adcsControlLoop(double deltaTime, const glm::dvec3 &earthCenter, const glm::dvec3 &sunPosition);

  // Update target point for tracking mode (e.g., ground station position)
  void updateTargetTracking(const glm::dvec3 &targetPos) { targetPoint = targetPos; }

  // Calculate footprint circle on Earth's surface
  void calculateFootprint(const glm::dvec3 &earthCenter, int numPoints = 100);

  // Getters
  glm::dvec3 getPosition() const { return position; }
  glm::dvec3 getVelocity() const { return velocity; }
  glm::vec3 getColor() const { return color; }
  const std::vector<glm::dvec3> &getOrbitPath() const { return orbitPath; }
  const std::vector<glm::dvec3> &getFootprintCircle() const { return footprintCircle; }
  int getPlaneId() const { return planeId; }
  int getIndexInPlane() const { return indexInPlane; }
  bool shouldDrawOrbit() const
  {
    // For Molniya satellites (planeId -2), draw all orbits since each is in a different plane
    // For other constellations, only draw orbit for first satellite in each plane
    return (planeId == -2) || (indexInPlane == 0);
  }

  // Setters for physical properties
  void setMass(double m) { mass = m; }
  void setDragCoefficient(double cd) { dragCoefficient = cd; }
  void setCrossSectionalArea(double area) { crossSectionalArea = area; }
  void setReflectivity(double cr) { reflectivity = cr; }
  void setInertiaTensor(const glm::dvec3 &inertia) { inertiaTensor = inertia; }

  // Attitude getters
  glm::dquat getQuaternion() const { return quaternion; }
  glm::dvec3 getAngularVelocity() const { return angularVelocity; }
  AttitudeControlMode getControlMode() const { return controlMode; }

  // Get body axes in inertial frame (rotated by quaternion)
  glm::dvec3 getBodyXAxis() const { return quaternion * glm::dvec3(1.0, 0.0, 0.0); }
  glm::dvec3 getBodyYAxis() const { return quaternion * glm::dvec3(0.0, 1.0, 0.0); }
  glm::dvec3 getBodyZAxis() const { return quaternion * glm::dvec3(0.0, 0.0, 1.0); }

  // Attitude setters
  void setQuaternion(const glm::dquat &q) { quaternion = glm::normalize(q); }
  void setAngularVelocity(const glm::dvec3 &omega) { angularVelocity = omega; }
  void setControlMode(AttitudeControlMode mode) { controlMode = mode; }
  void setTargetQuaternion(const glm::dquat &q) { targetQuaternion = glm::normalize(q); }
  void setTargetPoint(const glm::dvec3 &point) { targetPoint = point; }

  // Control algorithm selection
  void setControlAlgorithm(ControlAlgorithm algo) { controlAlgorithm = algo; }
  ControlAlgorithm getControlAlgorithm() const { return controlAlgorithm; }

  // Actuator configuration
  void enableReactionWheels(bool enable) { hasReactionWheels = enable; }
  void enableMagnetorquers(bool enable) { hasMagnetorquers = enable; }
  void enableCMGs(bool enable) { hasCMGs = enable; }

  // PID control gains
  void setProportionalGain(double kp) { proportionalGain = kp; }
  void setIntegralGain(double ki) { integralGain = ki; }
  void setDerivativeGain(double kd) { derivativeGain = kd; }
  void resetIntegralError() { integralError = glm::dvec3(0.0); }

  // Auto-tune PID gains based on inertia and desired performance
  void autoTunePID(double settlingTime = 20.0, double dampingRatio = 0.9)
  {
    // Calculate natural frequency for desired settling time (2% criterion)
    // For 2% settling: t_s ≈ 4/(ζ·ω_n), so ω_n = 4/(ζ·t_s)
    double omega_n = 4.0 / (dampingRatio * settlingTime);

    // Use average moment of inertia
    double I_avg = (inertiaTensor.x + inertiaTensor.y + inertiaTensor.z) / 3.0;

    // Calculate gains for desired damping response
    // Kp = I·ω_n² (stiffness)
    // Kd = 2·ζ·I·ω_n (damping)
    // Ki = small fraction of Kp (eliminate steady-state error)
    proportionalGain = I_avg * omega_n * omega_n;
    derivativeGain = 2.0 * dampingRatio * I_avg * omega_n;
    integralGain = 0.01 * proportionalGain; // Very conservative to avoid instability

    resetIntegralError();
  }

private:
  // ========== ADCS FLIGHT SOFTWARE METHODS ==========
  // These methods mirror the structure of actual satellite ADCS code

  // Step 1: Attitude Determination
  glm::dquat determineAttitude(const glm::dvec3 &earthCenter, const glm::dvec3 &sunPosition);

  // Step 2: Compute target attitude based on control mode
  glm::dquat computeTargetAttitude(const glm::dvec3 &earthCenter, const glm::dvec3 &sunPosition);

  // Step 3: Compute attitude error
  glm::dvec3 computeAttitudeError(const glm::dquat &currentAttitude, const glm::dquat &targetAttitude);

  // Step 4: Control Algorithms - compute desired control torque
  glm::dvec3 computeControlTorque(const glm::dvec3 &attitudeError, double deltaTime);
  glm::dvec3 computeControlTorquePID(const glm::dvec3 &attitudeError, double deltaTime);
  glm::dvec3 computeControlTorqueLQR(const glm::dvec3 &attitudeError, double deltaTime);
  glm::dvec3 computeControlTorqueMPC(const glm::dvec3 &attitudeError, double deltaTime);

  // Step 5: Actuator allocation and commanding
  void commandReactionWheels(const glm::dvec3 &desiredTorque, double deltaTime);
  void commandMagnetorquers(const glm::dvec3 &desiredTorque, const glm::dvec3 &magneticField);
  void commandCMGs(const glm::dvec3 &desiredTorque, double deltaTime);

  // Step 6: Attitude dynamics propagation
  void propagateAttitudeDynamics(double deltaTime, const glm::dvec3 &externalTorque);

  // ========== ORBITAL DYNAMICS METHODS ==========
  glm::dvec3 calculateAcceleration(const glm::dvec3 &pos, const glm::dvec3 &vel, const glm::dvec3 &earthCenter, double earthMass, const glm::dvec3 &sunPos, const glm::dvec3 &moonPos) const;

  glm::dvec3 calculateGravitationalAcceleration(const glm::dvec3 &pos, const glm::dvec3 &bodyPos, double bodyMass) const;
  glm::dvec3 calculateDragAcceleration(const glm::dvec3 &pos, const glm::dvec3 &vel, const glm::dvec3 &earthCenter) const;
  glm::dvec3 calculateSolarRadiationPressure(const glm::dvec3 &pos, const glm::dvec3 &sunPos, const glm::dvec3 &earthCenter) const;

  // Orbital state
  Orbit orbit;         // Orbit paramters
  glm::dvec3 position; // Position in meters (x, y, z)
  glm::dvec3 velocity; // Velocity in meters/second
  glm::vec3 color;     // RGB color for rendering

  // Attitude state (orientation in space)
  glm::dquat quaternion;      // Attitude quaternion (body frame to inertial frame)
  glm::dvec3 angularVelocity; // Angular velocity in rad/s (body frame)

  // Physical properties
  double mass = 260.0;              // Satellite mass in kg (typical for Starlink)
  double dragCoefficient = 2.2;     // Drag coefficient (dimensionless, ~2.2 for satellites)
  double crossSectionalArea = 10.0; // Cross-sectional area in m^2
  double reflectivity = 1.3;        // Reflectivity coefficient (1.0 = absorbing, 2.0 = perfect mirror)

  // Moments of inertia (kg·m²) - assuming box-shaped satellite
  glm::dvec3 inertiaTensor = glm::dvec3(50.0, 50.0, 20.0); // Ixx, Iyy, Izz

  // Reaction wheel properties
  bool hasReactionWheels = false;
  double reactionWheelMaxTorque = 0.5;                // Maximum torque per wheel (N·m)
  double reactionWheelMaxMomentum = 10.0;             // Maximum momentum per wheel (N·m·s)
  glm::dvec3 reactionWheelMomentum = glm::dvec3(0.0); // Current stored momentum (body frame)

  // Magnetorquer properties
  bool hasMagnetorquers = false;
  double magnetorquerMaxDipole = 10.0; // Maximum magnetic dipole moment (A·m²)

  // Control Moment Gyroscope (CMG) properties
  bool hasCMGs = false;
  int numCMGs = 4;               // Number of CMGs (typically 4 for pyramid config)
  double cmgMomentum = 50.0;     // Momentum per CMG (N·m·s)
  double cmgMaxGimbalRate = 0.1; // Maximum gimbal rate (rad/s)

  // Attitude control state
  AttitudeControlMode controlMode = AttitudeControlMode::NONE;
  ControlAlgorithm controlAlgorithm = ControlAlgorithm::PID;    // Default controller
  glm::dquat targetQuaternion = glm::dquat(1.0, 0.0, 0.0, 0.0); // Target orientation (identity)
  glm::dvec3 targetPoint = glm::dvec3(0.0);                     // Target point for tracking mode

  // PID controller gains (tuned for typical small satellite with I ~ 50 kg·m²)
  double proportionalGain = 0.07; // Proportional gain for attitude error (N·m/rad)
  double integralGain = 0.0;      // Integral gain for steady-state error elimination (N·m/(rad·s))
  double derivativeGain = 0.5;    // Derivative gain for damping (N·m·s/rad)

  // Integral error accumulation
  glm::dvec3 integralError = glm::dvec3(0.0); // Accumulated attitude error
  double integralErrorMax = 10.0;             // Anti-windup limit (rad)

  // LQR controller state weighting matrices
  // State: [attitude_error, angular_velocity]
  glm::dmat3 lqrQ = glm::dmat3(10.0); // State cost matrix (diagonal)
  glm::dmat3 lqrR = glm::dmat3(0.1);  // Control cost matrix (diagonal)
  glm::dmat3 lqrK = glm::dmat3(0.0);  // LQR gain matrix (computed)

  // MPC controller parameters
  int mpcHorizon = 10;                // Prediction horizon (timesteps)
  double mpcTimestep = 0.1;           // MPC internal timestep (s)
  glm::dmat3 mpcQ = glm::dmat3(10.0); // State cost matrix
  glm::dmat3 mpcR = glm::dmat3(0.1);  // Control cost matrix

  std::vector<glm::dvec3> footprintCircle; // Positions for drawing footprint circle
  std::vector<glm::dvec3> orbitPath;       // Historical positions for drawing orbit
  int orbitPathMaxSize = 1000;             // Maximum number of points to store

  int planeId;      // Orbital plane identifier
  int indexInPlane; // Index of this satellite within its plane
};

#endif // SATELLITE_H
