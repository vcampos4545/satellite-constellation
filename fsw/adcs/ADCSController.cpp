#include "ADCSController.h"
#include "Satellite.h"
#include "Constants.h"
#include <cmath>

ADCSController::ADCSController()
    : integralError(0.0), previousError(0.0)
{
}

void ADCSController::executeControlLoop(Satellite *satellite, double deltaTime,
                                        const glm::dvec3 &earthCenter, const glm::dvec3 &sunPosition)
{
  // Check if ADCS is active
  if (satellite->getControlMode() == AttitudeControlMode::NONE)
    return;

  // STEP 1: Attitude Determination
  // In real satellite: fuse gyros, star trackers, sun sensors, magnetometers
  glm::dquat currentAttitude = determineAttitude(satellite, earthCenter, sunPosition);

  // STEP 2: Compute Target Attitude
  // Determine where we want to point based on control mode
  glm::dquat targetAttitude = computeTargetAttitude(satellite, earthCenter, sunPosition);

  // STEP 3: Compute Attitude Error
  // Calculate rotation needed to go from current to target
  glm::dvec3 attitudeError = computeAttitudeError(currentAttitude, targetAttitude);

  // STEP 4: Compute Control Torque
  // Calculate desired torque to minimize error
  glm::dvec3 desiredTorque = computeControlTorque(satellite, attitudeError, deltaTime);

  // STEP 5: Command Actuators
  // Apply torque using available actuators
  commandActuators(satellite, desiredTorque, deltaTime);
}

glm::dquat ADCSController::determineAttitude(Satellite *satellite,
                                             const glm::dvec3 &earthCenter,
                                             const glm::dvec3 &sunPosition)
{
  // ===== ATTITUDE DETERMINATION SUBSYSTEM =====
  // In real satellite: This would run a Kalman filter combining:
  //   - Gyroscope measurements (high rate, drifts)
  //   - Star tracker (absolute, slow update)
  //   - Sun sensor (1-2 axis constraint)
  //   - Magnetometer (2 axis constraint)
  //
  // For now in simulation: Return perfect knowledge
  // Future: Add sensor models + noise + Kalman filter

  return satellite->getQuaternion(); // Perfect knowledge
}

glm::dquat ADCSController::computeTargetAttitude(Satellite *satellite,
                                                 const glm::dvec3 &earthCenter,
                                                 const glm::dvec3 &sunPosition)
{
  // ===== TARGET ATTITUDE CALCULATION =====
  // Compute desired pointing based on control mode

  AttitudeControlMode controlMode = satellite->getControlMode();
  glm::dvec3 position = satellite->getPosition();
  glm::dvec3 velocity = satellite->getVelocity();
  glm::dquat quaternion = satellite->getQuaternion();
  glm::dquat targetQuaternion = satellite->getTargetQuaternion();
  glm::dvec3 targetPoint = satellite->getTargetPoint();

  switch (controlMode)
  {
  case AttitudeControlMode::NADIR_POINTING:
  {
    // Point Z-axis toward Earth center (nadir)
    glm::dvec3 nadir = glm::normalize(earthCenter - position);
    glm::dvec3 zBody(0.0, 0.0, 1.0);

    // Create quaternion to rotate zBody to align with nadir
    glm::dvec3 axis = glm::cross(zBody, nadir);
    double sinHalfAngle = glm::length(axis);

    if (sinHalfAngle < 1e-6)
      return glm::dquat(1.0, 0.0, 0.0, 0.0); // Already aligned

    double cosHalfAngle = sqrt(0.5 * (1.0 + glm::dot(zBody, nadir)));
    sinHalfAngle = sqrt(0.5 * (1.0 - glm::dot(zBody, nadir)));
    glm::dvec3 axisNorm = glm::normalize(axis);

    return glm::dquat(cosHalfAngle, axisNorm.x * sinHalfAngle, axisNorm.y * sinHalfAngle, axisNorm.z * sinHalfAngle);
  }

  case AttitudeControlMode::SUN_POINTING:
  {
    // Point Z-axis toward Sun
    glm::dvec3 toSun = glm::normalize(sunPosition - position);
    glm::dvec3 zBody(0.0, 0.0, 1.0);

    glm::dvec3 axis = glm::cross(zBody, toSun);
    double sinHalfAngle = glm::length(axis);

    if (sinHalfAngle < 1e-6)
      return glm::dquat(1.0, 0.0, 0.0, 0.0);

    double cosHalfAngle = sqrt(0.5 * (1.0 + glm::dot(zBody, toSun)));
    sinHalfAngle = sqrt(0.5 * (1.0 - glm::dot(zBody, toSun)));
    glm::dvec3 axisNorm = glm::normalize(axis);

    return glm::dquat(cosHalfAngle, axisNorm.x * sinHalfAngle, axisNorm.y * sinHalfAngle, axisNorm.z * sinHalfAngle);
  }

  case AttitudeControlMode::VELOCITY_POINTING:
  {
    // Point X-axis along velocity (RAM direction)
    glm::dvec3 velDir = glm::normalize(velocity);
    glm::dvec3 xBody(1.0, 0.0, 0.0);

    glm::dvec3 axis = glm::cross(xBody, velDir);
    double sinHalfAngle = glm::length(axis);

    if (sinHalfAngle < 1e-6)
      return glm::dquat(1.0, 0.0, 0.0, 0.0);

    double cosHalfAngle = sqrt(0.5 * (1.0 + glm::dot(xBody, velDir)));
    sinHalfAngle = sqrt(0.5 * (1.0 - glm::dot(xBody, velDir)));
    glm::dvec3 axisNorm = glm::normalize(axis);

    return glm::dquat(cosHalfAngle, axisNorm.x * sinHalfAngle, axisNorm.y * sinHalfAngle, axisNorm.z * sinHalfAngle);
  }

  case AttitudeControlMode::INERTIAL_HOLD:
    // Maintain current target orientation
    return targetQuaternion;

  case AttitudeControlMode::DETUMBLE:
    // For detumble, target is low angular velocity (any orientation is fine)
    return quaternion; // Don't change orientation, just reduce rates

  case AttitudeControlMode::TARGET_TRACKING:
  {
    // Point Z-axis toward target point (e.g., ground station)
    glm::dvec3 toTarget = glm::normalize(targetPoint - position);
    glm::dvec3 zBody(0.0, 0.0, 1.0);

    glm::dvec3 axis = glm::cross(zBody, toTarget);
    double sinHalfAngle = glm::length(axis);

    if (sinHalfAngle < 1e-6)
      return glm::dquat(1.0, 0.0, 0.0, 0.0); // Already aligned

    double cosHalfAngle = sqrt(0.5 * (1.0 + glm::dot(zBody, toTarget)));
    sinHalfAngle = sqrt(0.5 * (1.0 - glm::dot(zBody, toTarget)));
    glm::dvec3 axisNorm = glm::normalize(axis);

    return glm::dquat(cosHalfAngle, axisNorm.x * sinHalfAngle, axisNorm.y * sinHalfAngle, axisNorm.z * sinHalfAngle);
  }

  default:
    return glm::dquat(1.0, 0.0, 0.0, 0.0);
  }
}

glm::dvec3 ADCSController::computeAttitudeError(const glm::dquat &currentAttitude, const glm::dquat &targetAttitude)
{
  // ===== ATTITUDE ERROR CALCULATION =====
  // Compute quaternion error: q_error = q_target * q_current^-1
  glm::dquat errorQuat = targetAttitude * glm::inverse(currentAttitude);

  // Normalize to ensure it's a unit quaternion
  errorQuat = glm::normalize(errorQuat);

  // Convert to axis-angle (small angle approximation for control)
  // For small angles: error ≈ 2 * [qx, qy, qz]
  // For large angles: use proper atan2 formula
  glm::dvec3 errorAxis(errorQuat.x, errorQuat.y, errorQuat.z);
  double errorAngle = 2.0 * atan2(glm::length(errorAxis), errorQuat.w);

  // Return error vector (axis * angle)
  if (glm::length(errorAxis) > 1e-10)
    return glm::normalize(errorAxis) * errorAngle;
  else
    return glm::dvec3(0.0);
}

glm::dvec3 ADCSController::computeControlTorque(Satellite *satellite,
                                                const glm::dvec3 &attitudeError,
                                                double deltaTime)
{
  // Dispatch to appropriate control algorithm
  ControlAlgorithm controlAlgorithm = satellite->getControlAlgorithm();

  switch (controlAlgorithm)
  {
  case ControlAlgorithm::PID:
    return computeControlTorquePID(satellite, attitudeError, deltaTime);
  case ControlAlgorithm::LQR:
    return computeControlTorqueLQR(satellite, attitudeError, deltaTime);
  case ControlAlgorithm::MPC:
    return computeControlTorqueMPC(satellite, attitudeError, deltaTime);
  default:
    return computeControlTorquePID(satellite, attitudeError, deltaTime);
  }
}

glm::dvec3 ADCSController::computeControlTorquePID(Satellite *satellite,
                                                   const glm::dvec3 &attitudeError,
                                                   double deltaTime)
{
  // ===== PID CONTROLLER =====
  // Compute control torque: τ = Kp*e + Ki*∫e + Kd*de/dt
  // For attitude control: de/dt ≈ -ω (in body frame)

  double proportionalGain = satellite->getProportionalGain();
  double integralGain = satellite->getIntegralGain();
  double derivativeGain = satellite->getDerivativeGain();
  double integralErrorMax = satellite->getIntegralErrorMax();
  glm::dvec3 angularVelocity = satellite->getAngularVelocity();
  AttitudeControlMode controlMode = satellite->getControlMode();
  double reactionWheelMaxTorque = satellite->getReactionWheelMaxTorque();

  // Proportional term - torque proportional to attitude error
  glm::dvec3 proportionalTerm = proportionalGain * attitudeError;

  // Integral term (with anti-windup)
  // Only accumulate if error is significant (avoid drift)
  if (glm::length(attitudeError) > 0.01) // 0.01 rad = ~0.6 degrees
  {
    integralError += attitudeError * deltaTime;

    // Anti-windup: clamp integral error magnitude
    double integralMag = glm::length(integralError);
    if (integralMag > integralErrorMax)
      integralError = (integralErrorMax / integralMag) * integralError;
  }

  glm::dvec3 integralTerm = integralGain * integralError;

  // Derivative term - provide damping by opposing angular velocity
  // For detumble mode, we want to drive angular velocity to zero
  glm::dvec3 derivativeTerm;
  if (controlMode == AttitudeControlMode::DETUMBLE)
  {
    // Detumble: drive angular velocity to zero (B-dot control)
    derivativeTerm = -derivativeGain * angularVelocity;
  }
  else
  {
    // Normal control: damping term opposes angular velocity
    // This provides critical damping to prevent overshoot
    derivativeTerm = -derivativeGain * angularVelocity;
  }

  // Total control torque
  glm::dvec3 controlTorque = proportionalTerm + integralTerm + derivativeTerm;

  // Apply torque limits to prevent excessive control authority
  double torqueMagnitude = glm::length(controlTorque);
  double maxTorque = reactionWheelMaxTorque * 3.0; // 3 wheels working together
  if (torqueMagnitude > maxTorque)
  {
    controlTorque = (maxTorque / torqueMagnitude) * controlTorque;
  }

  return controlTorque;
}

glm::dvec3 ADCSController::computeControlTorqueLQR(Satellite *satellite,
                                                   const glm::dvec3 &attitudeError,
                                                   double deltaTime)
{
  // ===== LQR CONTROLLER =====
  // Linear Quadratic Regulator for optimal control
  // Minimizes cost function: J = integral(x'Qx + u'Ru)dt
  // Control law: u = -K*x where K is LQR gain matrix

  glm::dmat3 lqrQ = satellite->getLQRQ();
  glm::dmat3 lqrR = satellite->getLQRR();
  glm::dvec3 inertiaTensor = satellite->getInertiaTensor();
  glm::dvec3 angularVelocity = satellite->getAngularVelocity();
  double reactionWheelMaxTorque = satellite->getReactionWheelMaxTorque();

  // Get average weighting values
  double Q_att = (lqrQ[0][0] + lqrQ[1][1] + lqrQ[2][2]) / 3.0;
  double Q_omega = Q_att * 0.1; // Angular velocity cost (typically lower)
  double R = (lqrR[0][0] + lqrR[1][1] + lqrR[2][2]) / 3.0;
  double I_avg = (inertiaTensor.x + inertiaTensor.y + inertiaTensor.z) / 3.0;

  // Compute simplified LQR gains
  double K_att = sqrt(Q_att * I_avg / R);     // Gain for attitude error
  double K_omega = sqrt(Q_omega * I_avg / R); // Gain for angular velocity

  // LQR control law: τ = -K_att * e_att - K_omega * ω
  glm::dvec3 controlTorque = -K_att * attitudeError - K_omega * angularVelocity;

  // Apply torque limits
  double torqueMagnitude = glm::length(controlTorque);
  double maxTorque = reactionWheelMaxTorque * 3.0;
  if (torqueMagnitude > maxTorque)
  {
    controlTorque = (maxTorque / torqueMagnitude) * controlTorque;
  }

  return controlTorque;
}

glm::dvec3 ADCSController::computeControlTorqueMPC(Satellite *satellite,
                                                   const glm::dvec3 &attitudeError,
                                                   double deltaTime)
{
  // ===== MODEL PREDICTIVE CONTROLLER =====
  // Simplified single-step MPC (horizon = 1) for computational efficiency

  glm::dmat3 mpcQ = satellite->getMPCQ();
  glm::dmat3 mpcR = satellite->getMPCR();
  double mpcTimestep = satellite->getMPCTimestep();
  glm::dvec3 inertiaTensor = satellite->getInertiaTensor();
  glm::dvec3 angularVelocity = satellite->getAngularVelocity();
  double reactionWheelMaxTorque = satellite->getReactionWheelMaxTorque();

  // Get average weighting values
  double Q_att = (mpcQ[0][0] + mpcQ[1][1] + mpcQ[2][2]) / 3.0;
  double Q_omega = Q_att * 0.1;
  double R = (mpcR[0][0] + mpcR[1][1] + mpcR[2][2]) / 3.0;
  double I_avg = (inertiaTensor.x + inertiaTensor.y + inertiaTensor.z) / 3.0;

  // Predict next state if no control is applied
  glm::dvec3 predictedError = attitudeError + angularVelocity * mpcTimestep;
  glm::dvec3 angularMom = inertiaTensor * angularVelocity;
  glm::dvec3 gyroTorque = glm::cross(angularVelocity, angularMom);
  glm::dvec3 predictedOmega = angularVelocity - (gyroTorque / inertiaTensor) * mpcTimestep;

  // Compute control that minimizes predicted cost
  double gain = 1.0 / (R / Q_att + mpcTimestep * mpcTimestep / I_avg);
  glm::dvec3 controlTorque = -gain * (Q_att * predictedError + Q_omega * predictedOmega);

  // Apply torque limits
  double torqueMagnitude = glm::length(controlTorque);
  double maxTorque = reactionWheelMaxTorque * 3.0;
  if (torqueMagnitude > maxTorque)
  {
    controlTorque = (maxTorque / torqueMagnitude) * controlTorque;
  }

  return controlTorque;
}

void ADCSController::commandActuators(Satellite *satellite,
                                      const glm::dvec3 &desiredTorque,
                                      double deltaTime)
{
  // Command actuators to apply desired torque
  glm::dvec3 appliedTorque(0.0);

  if (satellite->hasReactionWheelsAvailable())
  {
    satellite->commandReactionWheels(desiredTorque, deltaTime);
    appliedTorque = desiredTorque; // Ideal case: wheels provide exactly what we want
  }
  else if (satellite->hasMagnetorquersAvailable())
  {
    // TODO: Get magnetic field model
    glm::dvec3 magneticField(0.0, 0.0, 3e-5); // Placeholder: ~30 μT
    satellite->commandMagnetorquers(desiredTorque, magneticField);
    // Magnetorquers can't provide arbitrary torque, only perpendicular to B-field
  }
  else if (satellite->hasCMGsAvailable())
  {
    satellite->commandCMGs(desiredTorque, deltaTime);
    appliedTorque = desiredTorque;
  }

  // Propagate attitude dynamics with applied torque
  satellite->propagateAttitudeDynamics(deltaTime, appliedTorque);
}
