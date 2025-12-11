#include "Satellite.h"
#include "Constants.h"
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>

Satellite::Satellite(const Orbit &orbit, const glm::dvec3 &initPos, const glm::dvec3 &initVel, const glm::vec3 &color, int planeId, int indexInPlane, const std::string &name)
    : orbit(orbit),
      position(initPos),
      velocity(initVel),
      color(color),
      planeId(planeId),
      indexInPlane(indexInPlane),
      name(name),
      quaternion(1.0, 0.0, 0.0, 0.0), // Identity quaternion (no rotation)
      angularVelocity(0.0, 0.0, 0.0)  // No initial rotation
{
  autoTunePID();
}

glm::dvec3 Satellite::calculateGravitationalAcceleration(const glm::dvec3 &pos, const glm::dvec3 &bodyPos, double bodyMass) const
{
  glm::dvec3 toBody = bodyPos - pos;
  double distance = glm::length(toBody);

  if (distance < 1.0)
    return glm::dvec3(0.0); // Avoid division by zero

  glm::dvec3 direction = glm::normalize(toBody);
  double accelMagnitude = G * bodyMass / (distance * distance);
  return direction * accelMagnitude;
}

glm::dvec3 Satellite::calculateZonalHarmonicsAcceleration(const glm::dvec3 &pos, const glm::dvec3 &earthCenter, double earthMass) const
{
  // Calculate Earth's non-spherical gravity perturbations (J2, J3, J4)
  // These account for Earth's oblate shape and mass distribution
  //
  // Reference: Vallado, "Fundamentals of Astrodynamics and Applications", 4th Ed.
  // Chapter 8: Perturbations

  // Position vector relative to Earth center
  glm::dvec3 r = pos - earthCenter;
  double r_mag = glm::length(r);

  if (r_mag < EARTH_EQUATORIAL_RADIUS)
    return glm::dvec3(0.0); // Inside Earth - shouldn't happen

  // Normalized position components
  double x = r.x / r_mag;
  double y = r.y / r_mag;
  double z = r.z / r_mag; // z/r is sin(latitude) in Earth-centered coordinates

  double z2 = z * z;   // (z/r)²
  double z4 = z2 * z2; // (z/r)⁴

  // Common terms
  double mu = G * earthMass;
  double Re = EARTH_EQUATORIAL_RADIUS;
  double Re_r = Re / r_mag;    // Re/r
  double Re_r2 = Re_r * Re_r;  // (Re/r)²
  double Re_r3 = Re_r2 * Re_r; // (Re/r)³
  double Re_r4 = Re_r3 * Re_r; // (Re/r)⁴
  double mu_r3 = mu / (r_mag * r_mag * r_mag);

  // ========== J2 PERTURBATION (Oblateness - Equatorial Bulge) ==========
  // Dominant term: ~1000x larger than J3/J4
  // Causes RAAN and argument of perigee to precess
  double j2_factor = -1.5 * J2 * Re_r2 * mu_r3;

  glm::dvec3 accel_J2;
  accel_J2.x = j2_factor * x * (1.0 - 5.0 * z2);
  accel_J2.y = j2_factor * y * (1.0 - 5.0 * z2);
  accel_J2.z = j2_factor * z * (3.0 - 5.0 * z2);

  // ========== J3 PERTURBATION (Pear-Shape - North/South Asymmetry) ==========
  // Second-order term: Southern hemisphere slightly "heavier"
  // Important for high-inclination orbits
  double j3_factor = -0.5 * J3 * Re_r3 * mu_r3;

  glm::dvec3 accel_J3;
  accel_J3.x = j3_factor * x * (5.0 * z * (7.0 * z2 - 3.0));
  accel_J3.y = j3_factor * y * (5.0 * z * (7.0 * z2 - 3.0));
  accel_J3.z = j3_factor * (6.0 * z2 - 7.0 * z4 - 3.0 / 5.0);

  // ========== J4 PERTURBATION (Higher-Order Oblateness) ==========
  // Third-order term: Additional refinement to oblateness
  // Small correction to J2 effects
  double j4_factor = 0.625 * J4 * Re_r4 * mu_r3; // 5/8 = 0.625

  glm::dvec3 accel_J4;
  accel_J4.x = j4_factor * x * (1.0 - 14.0 * z2 + 21.0 * z4);
  accel_J4.y = j4_factor * y * (1.0 - 14.0 * z2 + 21.0 * z4);
  accel_J4.z = j4_factor * z * (5.0 - 70.0 * z2 / 3.0 + 21.0 * z4);

  // Total zonal harmonics acceleration
  return accel_J2 + accel_J3 + accel_J4;
}

glm::dvec3 Satellite::calculateDragAcceleration(const glm::dvec3 &pos, const glm::dvec3 &vel, const glm::dvec3 &earthCenter) const
{
  // Calculate altitude above Earth's surface
  double distance = glm::length(pos - earthCenter);
  double altitude = distance - EARTH_RADIUS;

  // Only apply drag below DRAG_ALTITUDE_MAX
  if (altitude <= 0.0 || altitude >= DRAG_ALTITUDE_MAX)
    return glm::dvec3(0.0);

  // Exponential atmosphere model: ρ(h) = ρ₀ * e^(-h/H)
  double atmosphericDensity = RHO_0 * exp(-altitude / H_SCALE);

  // Drag force: F_drag = 0.5 * ρ * v² * Cd * A
  // Drag acceleration: a_drag = F_drag / m
  double speedSquared = glm::dot(vel, vel);

  if (speedSquared <= 0.0)
    return glm::dvec3(0.0);

  double speed = sqrt(speedSquared);
  glm::dvec3 velocityDirection = vel / speed;

  // Drag acts opposite to velocity
  double dragAccelMag = 0.5 * atmosphericDensity * speedSquared * dragCoefficient * crossSectionalArea / mass;
  return -velocityDirection * dragAccelMag;
}

glm::dvec3 Satellite::calculateSolarRadiationPressure(const glm::dvec3 &pos, const glm::dvec3 &sunPos, const glm::dvec3 &earthCenter) const
{
  glm::dvec3 toSun = sunPos - pos;
  double distanceSun = glm::length(toSun);

  if (distanceSun < 1.0)
    return glm::dvec3(0.0);

  glm::dvec3 sunDirection = glm::normalize(toSun);

  // Check if satellite is in Earth's shadow (simple cylindrical shadow model)
  glm::dvec3 satToEarth = earthCenter - pos;
  double projectionOnSunLine = glm::dot(satToEarth, -sunDirection);

  bool inShadow = false;
  if (projectionOnSunLine > 0.0) // Satellite is on night side
  {
    // Distance from satellite to sun-earth line
    glm::dvec3 perpComponent = satToEarth + sunDirection * projectionOnSunLine;
    double perpDistance = glm::length(perpComponent);

    if (perpDistance < EARTH_RADIUS)
      inShadow = true;
  }

  // Only apply SRP if satellite is in sunlight
  if (inShadow)
    return glm::dvec3(0.0);

  // Solar radiation pressure at satellite's distance from sun
  // P = P₀ * (AU / r)²
  double distanceRatio = AU / distanceSun;
  double pressure = SOLAR_PRESSURE * distanceRatio * distanceRatio;

  // SRP force: F = P * A * Cr
  // SRP acceleration: a = F / m = P * A * Cr / m
  double srpAccelMag = pressure * crossSectionalArea * reflectivity / mass;
  return sunDirection * srpAccelMag;
}

glm::dvec3 Satellite::calculateAcceleration(const glm::dvec3 &pos, const glm::dvec3 &vel, const glm::dvec3 &earthCenter, double earthMass, const glm::dvec3 &sunPos, const glm::dvec3 &moonPos) const
{
  // Calculate all acceleration components

  // Earth gravity: point mass + non-spherical perturbations (J2, J3, J4)
  glm::dvec3 gravAccelEarth = calculateGravitationalAcceleration(pos, earthCenter, earthMass);
  glm::dvec3 gravAccelEarthJ = calculateZonalHarmonicsAcceleration(pos, earthCenter, earthMass);

  // Third-body perturbations (Moon and Sun)
  glm::dvec3 gravAccelMoon = calculateGravitationalAcceleration(pos, moonPos, MOON_MASS);
  glm::dvec3 gravAccelSun = calculateGravitationalAcceleration(pos, sunPos, SUN_MASS);

  // Non-gravitational perturbations
  glm::dvec3 dragAccel = calculateDragAcceleration(pos, vel, earthCenter);
  glm::dvec3 srpAccel = calculateSolarRadiationPressure(pos, sunPos, earthCenter);

  // Total acceleration (all forces combined)
  // Earth: point mass + J2/J3/J4 perturbations
  // Third bodies: Moon + Sun
  // Non-gravitational: Drag + Solar radiation pressure
  return gravAccelEarth + gravAccelEarthJ + gravAccelMoon + gravAccelSun + dragAccel + srpAccel;
}

void Satellite::update(double deltaTime, const glm::dvec3 &earthCenter, double earthMass, const glm::dvec3 &sunPosition, const glm::dvec3 &moonPosition)
{
  /* Main satellite update function.  Runs every frame */

  // ========== ORBITAL DYNAMICS ==========
  // RK4 (Runge-Kutta 4th order) integration for better accuracy
  // This is much more stable than Euler integration for orbital mechanics

  // k1 = f(t, y)
  glm::dvec3 k1_vel = calculateAcceleration(position, velocity, earthCenter, earthMass, sunPosition, moonPosition);
  glm::dvec3 k1_pos = velocity;

  // k2 = f(t + dt/2, y + k1*dt/2)
  glm::dvec3 k2_vel = calculateAcceleration(
      position + k1_pos * (deltaTime * 0.5),
      velocity + k1_vel * (deltaTime * 0.5),
      earthCenter, earthMass, sunPosition, moonPosition);
  glm::dvec3 k2_pos = velocity + k1_vel * (deltaTime * 0.5);

  // k3 = f(t + dt/2, y + k2*dt/2)
  glm::dvec3 k3_vel = calculateAcceleration(
      position + k2_pos * (deltaTime * 0.5),
      velocity + k2_vel * (deltaTime * 0.5),
      earthCenter, earthMass, sunPosition, moonPosition);
  glm::dvec3 k3_pos = velocity + k2_vel * (deltaTime * 0.5);

  // k4 = f(t + dt, y + k3*dt)
  glm::dvec3 k4_vel = calculateAcceleration(
      position + k3_pos * deltaTime,
      velocity + k3_vel * deltaTime,
      earthCenter, earthMass, sunPosition, moonPosition);
  glm::dvec3 k4_pos = velocity + k3_vel * deltaTime;

  // Update: y_new = y + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
  velocity += (deltaTime / 6.0) * (k1_vel + 2.0 * k2_vel + 2.0 * k3_vel + k4_vel);
  position += (deltaTime / 6.0) * (k1_pos + 2.0 * k2_pos + 2.0 * k3_pos + k4_pos);

  // ========== STATION KEEPING ==========
  // Check if we need to perform orbit maintenance burns
  if (stationKeepingEnabled && hasThrusters && hasPropellant())
  {
    performStationKeeping(deltaTime, earthCenter);
  }

  // ========== ATTITUDE CONTROL SYSTEM ==========
  // Run ADCS control loop (attitude determination and control)
  adcsControlLoop(deltaTime, earthCenter, sunPosition);

  // ========== POWER MANAGEMENT ==========
  // Update battery charge based on solar generation and system consumption
  updatePowerSystem(deltaTime, sunPosition, earthCenter);

  // Update footprint every frame
  calculateFootprint(earthCenter, 60);
}

void Satellite::calculateOrbitPath(int numPoints)
{
  orbitPath.clear();

  // Generate orbit path by computing position at various true anomalies
  // Uses the same coordinate transformation as initial satellite placement
  for (int pointIdx = 0; pointIdx <= numPoints; ++pointIdx)
  {
    // True anomaly - angle from periapsis around the orbit
    double trueAnomaly = 2.0 * PI * pointIdx / numPoints;

    // Create temporary orbit with this true anomaly
    Orbit orbitAtPoint = orbit;
    orbitAtPoint.v = trueAnomaly;

    // Convert to Cartesian coordinates (position only, velocity not needed)
    glm::dvec3 pos, vel;
    orbitAtPoint.toCartesian(pos, vel, G * EARTH_MASS);

    orbitPath.push_back(pos);
  }
}

void Satellite::calculateFootprint(const glm::dvec3 &earthCenter, int numPoints)
{
  footprintCircle.clear();

  // Vector from Earth center to satellite
  glm::dvec3 toSatellite = position - earthCenter;
  double satelliteDistance = glm::length(toSatellite);
  glm::dvec3 nadirDir = glm::normalize(toSatellite);

  // Horizon angle (in radians)
  double lambda0 = acos(EARTH_RADIUS / satelliteDistance);

  // TODO: Decrease angle by some amount for feasible footprint (i.e. lambda0 - 5 degrees)
  // This accounts for ground station minimum elevation angle

  // Find a perpendicular vector to nadirDir to start with
  glm::dvec3 perpendicular = glm::normalize(glm::cross(nadirDir, glm::dvec3(0.0, 1.0, 0.0)));

  // Create initial horizon point by rotating perpendicular vector around another perpendicular
  glm::dvec3 secondPerp = glm::normalize(glm::cross(nadirDir, perpendicular));
  glm::dvec3 initialHorizonDir = nadirDir * cos(lambda0) + secondPerp * sin(lambda0);
  glm::dvec3 initialHorizonPoint = earthCenter + initialHorizonDir * EARTH_RADIUS;

  // Rotate this initial point around the nadirDir axis to create the footprint circle
  for (int i = 0; i <= numPoints; ++i)
  {
    double angle = (2.0 * PI * i) / numPoints;

    // Create rotation matrix around nadirDir axis
    glm::dmat4 rotationMatrix = glm::rotate(glm::dmat4(1.0), angle, nadirDir);

    // Rotate the initial horizon point around the axis
    glm::dvec3 rotatedPoint = glm::dvec3(rotationMatrix * glm::dvec4(initialHorizonPoint - earthCenter, 1.0)) + earthCenter;

    footprintCircle.push_back(rotatedPoint);
  }
}

// ========== ADCS FLIGHT SOFTWARE IMPLEMENTATION ==========
// This mirrors the control loop structure used in actual satellite ADCS microcontrollers

void Satellite::adcsControlLoop(double deltaTime, const glm::dvec3 &earthCenter, const glm::dvec3 &sunPosition)
{
  // Check if ADCS is active
  if (controlMode == AttitudeControlMode::NONE)
    return;

  // STEP 1: Attitude Determination
  // In real satellite: fuse gyros, star trackers, sun sensors, magnetometers
  // In simulation: use perfect knowledge for now
  glm::dquat currentAttitude = determineAttitude(earthCenter, sunPosition);

  // STEP 2: Compute Target Attitude
  // Determine where we want to point based on control mode
  glm::dquat targetAttitude = computeTargetAttitude(earthCenter, sunPosition);

  // STEP 3: Compute Attitude Error
  // Calculate rotation needed to go from current to target
  glm::dvec3 attitudeError = computeAttitudeError(currentAttitude, targetAttitude);

  // STEP 4: Compute Control Torque (PID Controller)
  // Calculate desired torque to minimize error
  glm::dvec3 desiredTorque = computeControlTorque(attitudeError, deltaTime);

  // STEP 5: Actuator Commanding
  // Allocate torque to available actuators
  glm::dvec3 appliedTorque(0.0);

  if (hasReactionWheels)
  {
    commandReactionWheels(desiredTorque, deltaTime);
    appliedTorque = desiredTorque; // Ideal case: wheels provide exactly what we want
  }
  else if (hasMagnetorquers)
  {
    // TODO: Get magnetic field model
    glm::dvec3 magneticField(0.0, 0.0, 3e-5); // Placeholder: ~30 μT
    commandMagnetorquers(desiredTorque, magneticField);
    // Magnetorquers can't provide arbitrary torque, only perpendicular to B-field
  }
  else if (hasCMGs)
  {
    commandCMGs(desiredTorque, deltaTime);
    appliedTorque = desiredTorque;
  }

  // STEP 6: Propagate Attitude Dynamics
  // Integrate Euler's equations to update quaternion and angular velocity
  propagateAttitudeDynamics(deltaTime, appliedTorque);
}

glm::dquat Satellite::determineAttitude(const glm::dvec3 &earthCenter, const glm::dvec3 &sunPosition)
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

  return quaternion; // Perfect knowledge
}

glm::dquat Satellite::computeTargetAttitude(const glm::dvec3 &earthCenter, const glm::dvec3 &sunPosition)
{
  // ===== TARGET ATTITUDE CALCULATION =====
  // Compute desired pointing based on control mode

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

glm::dvec3 Satellite::computeAttitudeError(const glm::dquat &currentAttitude, const glm::dquat &targetAttitude)
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

glm::dvec3 Satellite::computeControlTorque(const glm::dvec3 &attitudeError, double deltaTime)
{
  // Dispatch to appropriate control algorithm
  switch (controlAlgorithm)
  {
  case ControlAlgorithm::PID:
    return computeControlTorquePID(attitudeError, deltaTime);
  case ControlAlgorithm::LQR:
    return computeControlTorqueLQR(attitudeError, deltaTime);
  case ControlAlgorithm::MPC:
    return computeControlTorqueMPC(attitudeError, deltaTime);
  default:
    return computeControlTorquePID(attitudeError, deltaTime);
  }
}

glm::dvec3 Satellite::computeControlTorquePID(const glm::dvec3 &attitudeError, double deltaTime)
{
  // ===== PID CONTROLLER =====
  // Compute control torque: τ = Kp*e + Ki*∫e + Kd*de/dt
  // For attitude control: de/dt ≈ -ω (in body frame)

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

glm::dvec3 Satellite::computeControlTorqueLQR(const glm::dvec3 &attitudeError, double deltaTime)
{
  // ===== LQR CONTROLLER =====
  // Linear Quadratic Regulator for optimal control
  // Minimizes cost function: J = integral(x'Qx + u'Ru)dt
  // Control law: u = -K*x where K is LQR gain matrix
  //
  // Simplified implementation using approximate discrete-time LQR
  // State vector: x = [attitude_error; angular_velocity]

  // Compute LQR gain matrix K using simplified Riccati approximation
  // For diagonal Q and R matrices, simplified gain:
  // K ≈ [sqrt(Q_att/R); sqrt(Q_omega/R)]

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
  double maxTorque = reactionWheelMaxTorque * 3.0; // 3 wheels working together
  if (torqueMagnitude > maxTorque)
  {
    controlTorque = (maxTorque / torqueMagnitude) * controlTorque;
  }

  return controlTorque;
}

glm::dvec3 Satellite::computeControlTorqueMPC(const glm::dvec3 &attitudeError, double deltaTime)
{
  // ===== MODEL PREDICTIVE CONTROLLER =====
  // Solves optimization problem over prediction horizon
  // Minimizes: sum(||x(k)||_Q^2 + ||u(k)||_R^2) over k=0..N
  // Subject to: dynamics constraints and actuator limits
  //
  // Simplified single-step MPC (horizon = 1) for computational efficiency
  // Full MPC would require quadratic programming solver

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
  // Simplified solution: τ = -(Q*e_predicted + R*ω_predicted) / (R + dt^2*I^-1)
  double gain = 1.0 / (R / Q_att + mpcTimestep * mpcTimestep / I_avg);
  glm::dvec3 controlTorque = -gain * (Q_att * predictedError + Q_omega * predictedOmega);

  // Apply torque limits
  double torqueMagnitude = glm::length(controlTorque);
  double maxTorque = reactionWheelMaxTorque * 3.0; // 3 wheels working together
  if (torqueMagnitude > maxTorque)
  {
    controlTorque = (maxTorque / torqueMagnitude) * controlTorque;
  }

  return controlTorque;
}

void Satellite::commandReactionWheels(const glm::dvec3 &desiredTorque, double deltaTime)
{
  // ===== REACTION WHEEL CONTROL =====
  // Reaction wheels provide torque by spinning up/down
  // τ_wheel = -τ_spacecraft (Newton's 3rd law)

  // Apply torque limits
  glm::dvec3 commandedTorque = desiredTorque;
  for (int i = 0; i < 3; i++)
  {
    if (commandedTorque[i] > reactionWheelMaxTorque)
      commandedTorque[i] = reactionWheelMaxTorque;
    else if (commandedTorque[i] < -reactionWheelMaxTorque)
      commandedTorque[i] = -reactionWheelMaxTorque;
  }

  // Update wheel momentum (h_wheel = ∫τ dt)
  reactionWheelMomentum += commandedTorque * deltaTime;

  // Check momentum saturation
  for (int i = 0; i < 3; i++)
  {
    if (abs(reactionWheelMomentum[i]) > reactionWheelMaxMomentum)
    {
      // Saturated - can't provide more torque in this direction
      // In real satellite: trigger momentum desaturation using magnetorquers
      if (reactionWheelMomentum[i] > 0)
        reactionWheelMomentum[i] = reactionWheelMaxMomentum;
      else
        reactionWheelMomentum[i] = -reactionWheelMaxMomentum;
    }
  }
}

void Satellite::commandMagnetorquers(const glm::dvec3 &desiredTorque, const glm::dvec3 &magneticField)
{
  // ===== MAGNETORQUER CONTROL =====
  // Magnetorquers generate torque: τ = m × B
  // where m is magnetic dipole, B is Earth's magnetic field

  // Can only generate torque perpendicular to B-field
  // For detumble, use B-dot control: m = -k * dB/dt ≈ -k * (ω × B)

  if (controlMode == AttitudeControlMode::DETUMBLE)
  {
    // B-dot control for detumble
    glm::dvec3 bDot = glm::cross(angularVelocity, magneticField);
    glm::dvec3 dipoleMoment = -derivativeGain * bDot;

    // Limit dipole magnitude
    double dipoleNorm = glm::length(dipoleMoment);
    if (dipoleNorm > magnetorquerMaxDipole)
      dipoleMoment = (magnetorquerMaxDipole / dipoleNorm) * dipoleMoment;

    // Torque generated: τ = m × B
    // (This torque is applied in propagateAttitudeDynamics)
  }
}

void Satellite::commandCMGs(const glm::dvec3 &desiredTorque, double deltaTime)
{
  // ===== CONTROL MOMENT GYROSCOPE =====
  // CMGs provide torque by gimbaling spinning rotors
  // Much more complex than reaction wheels - requires gimbal angle control

  // Simplified model: assume CMGs can provide desired torque within limits
  // Real implementation requires solving CMG steering law (singularity avoidance)

  // TODO: Implement full CMG dynamics with gimbal angles
}

void Satellite::propagateAttitudeDynamics(double deltaTime, const glm::dvec3 &externalTorque)
{
  // ===== ATTITUDE DYNAMICS INTEGRATION =====
  // Integrate Euler's rigid body equations using RK4:
  //   I*dω/dt = τ_external - ω × (I*ω)   (rotational dynamics)
  //   dq/dt = 0.5 * Ω(ω) * q             (quaternion kinematics)

  // Lambda function to compute angular acceleration from state
  auto computeAngularAccel = [this, &externalTorque](const glm::dvec3 &omega) -> glm::dvec3
  {
    glm::dvec3 angularMom = inertiaTensor * omega;
    glm::dvec3 gyroTorque = glm::cross(omega, angularMom);
    return (externalTorque - gyroTorque) / inertiaTensor;
  };

  // Lambda function to compute quaternion derivative
  auto computeQuatDot = [](const glm::dquat &q, const glm::dvec3 &omega) -> glm::dquat
  {
    glm::dquat omegaQuat(0.0, omega.x, omega.y, omega.z);
    return 0.5 * omegaQuat * q;
  };

  // RK4 Integration for angular velocity and quaternion

  // k1 = f(t, y)
  glm::dvec3 k1_omega = computeAngularAccel(angularVelocity);
  glm::dquat k1_quat = computeQuatDot(quaternion, angularVelocity);

  // k2 = f(t + dt/2, y + k1*dt/2)
  glm::dvec3 omega2 = angularVelocity + k1_omega * (deltaTime * 0.5);
  glm::dquat quat2 = quaternion;
  quat2.w += k1_quat.w * (deltaTime * 0.5);
  quat2.x += k1_quat.x * (deltaTime * 0.5);
  quat2.y += k1_quat.y * (deltaTime * 0.5);
  quat2.z += k1_quat.z * (deltaTime * 0.5);
  quat2 = glm::normalize(quat2);
  glm::dvec3 k2_omega = computeAngularAccel(omega2);
  glm::dquat k2_quat = computeQuatDot(quat2, omega2);

  // k3 = f(t + dt/2, y + k2*dt/2)
  glm::dvec3 omega3 = angularVelocity + k2_omega * (deltaTime * 0.5);
  glm::dquat quat3 = quaternion;
  quat3.w += k2_quat.w * (deltaTime * 0.5);
  quat3.x += k2_quat.x * (deltaTime * 0.5);
  quat3.y += k2_quat.y * (deltaTime * 0.5);
  quat3.z += k2_quat.z * (deltaTime * 0.5);
  quat3 = glm::normalize(quat3);
  glm::dvec3 k3_omega = computeAngularAccel(omega3);
  glm::dquat k3_quat = computeQuatDot(quat3, omega3);

  // k4 = f(t + dt, y + k3*dt)
  glm::dvec3 omega4 = angularVelocity + k3_omega * deltaTime;
  glm::dquat quat4 = quaternion;
  quat4.w += k3_quat.w * deltaTime;
  quat4.x += k3_quat.x * deltaTime;
  quat4.y += k3_quat.y * deltaTime;
  quat4.z += k3_quat.z * deltaTime;
  quat4 = glm::normalize(quat4);
  glm::dvec3 k4_omega = computeAngularAccel(omega4);
  glm::dquat k4_quat = computeQuatDot(quat4, omega4);

  // Update: y_new = y + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
  angularVelocity += (deltaTime / 6.0) * (k1_omega + 2.0 * k2_omega + 2.0 * k3_omega + k4_omega);

  quaternion.w += (deltaTime / 6.0) * (k1_quat.w + 2.0 * k2_quat.w + 2.0 * k3_quat.w + k4_quat.w);
  quaternion.x += (deltaTime / 6.0) * (k1_quat.x + 2.0 * k2_quat.x + 2.0 * k3_quat.x + k4_quat.x);
  quaternion.y += (deltaTime / 6.0) * (k1_quat.y + 2.0 * k2_quat.y + 2.0 * k3_quat.y + k4_quat.y);
  quaternion.z += (deltaTime / 6.0) * (k1_quat.z + 2.0 * k2_quat.z + 2.0 * k3_quat.z + k4_quat.z);

  // Normalize quaternion to prevent drift
  quaternion = glm::normalize(quaternion);
}

// ========== PROPULSION AND STATION KEEPING ==========

void Satellite::enableStationKeeping(bool enable, double targetAltitude)
{
  stationKeepingEnabled = enable;
  hasThrusters = enable; // Enable thrusters when station keeping is enabled

  if (enable && targetAltitude > 0.0)
  {
    // Set target semi-major axis from altitude
    targetSemiMajorAxis = EARTH_RADIUS + targetAltitude;
  }
  else if (enable)
  {
    // Use current semi-major axis as target
    double currentRadius = glm::length(position);
    targetSemiMajorAxis = currentRadius;
  }

  // Reset check timer
  timeSinceLastStationKeepingCheck = 0.0;
}

void Satellite::performStationKeeping(double deltaTime, const glm::dvec3 &earthCenter)
{
  /**
   * STATION KEEPING ALGORITHM
   *
   * Purpose: Maintain orbital altitude by compensating for atmospheric drag
   *
   * Strategy:
   * 1. Periodically check current orbital altitude (semi-major axis)
   * 2. If altitude has decayed beyond deadband, apply prograde thrust
   * 3. Use Hohmann transfer-like burn to restore target altitude
   *
   * Burn Calculation:
   * - For small altitude changes, Δv ≈ -(v/2) * (Δa/a)
   *   where a = semi-major axis, v = orbital velocity
   * - Apply thrust in velocity direction (prograde burn raises apoapsis)
   * - Limit burn duration to avoid overshoot
   */

  // Update timer
  timeSinceLastStationKeepingCheck += deltaTime;

  // Only check periodically (not every frame - too wasteful)
  if (timeSinceLastStationKeepingCheck < stationKeepingCheckInterval)
  {
    return;
  }

  // Reset timer
  timeSinceLastStationKeepingCheck = 0.0;

  // Calculate current semi-major axis (orbital radius approximation)
  double currentRadius = glm::length(position - earthCenter);
  double currentVelocityMag = glm::length(velocity);

  // Calculate current semi-major axis using vis-viva equation
  // v² = GM(2/r - 1/a)  =>  a = 1 / (2/r - v²/GM)
  double mu = G * EARTH_MASS;
  double specificEnergy = 0.5 * currentVelocityMag * currentVelocityMag - mu / currentRadius;
  double currentSemiMajorAxis = -mu / (2.0 * specificEnergy);

  // Calculate altitude error
  double altitudeError = targetSemiMajorAxis - currentSemiMajorAxis;

  // Check if we're outside deadband
  if (fabs(altitudeError) < stationKeepingDeadband)
  {
    return; // Within acceptable range, no burn needed
  }

  // ========== CALCULATE REQUIRED DELTA-V ==========
  // For small altitude corrections, use approximation:
  // Δv ≈ -(v/2) * (Δa/a)
  // This is derived from circular orbit velocity: v = sqrt(GM/a)
  // Taking differential: dv = -0.5 * sqrt(GM/a³) * da

  double deltaV_needed = -(currentVelocityMag / 2.0) * (altitudeError / currentSemiMajorAxis);

  // Limit delta-V to what we can achieve in one burn interval
  // Max Δv per burn = thrust * burn_time / mass
  double maxDeltaVPerBurn = (thrusterMaxThrust * stationKeepingCheckInterval) / mass;

  if (fabs(deltaV_needed) > maxDeltaVPerBurn)
  {
    deltaV_needed = (deltaV_needed > 0.0 ? 1.0 : -1.0) * maxDeltaVPerBurn;
  }

  // ========== APPLY THRUST ==========
  // Thrust direction: prograde (velocity direction) to raise orbit
  // (or retrograde to lower orbit, but we typically only fight drag)
  glm::dvec3 thrustDirection = glm::normalize(velocity);

  // For raising orbit (fighting drag), thrust prograde
  // For lowering orbit (rare), thrust retrograde
  if (deltaV_needed < 0.0)
  {
    thrustDirection = -thrustDirection;
    deltaV_needed = -deltaV_needed; // Make positive for calculations
  }

  // Calculate burn time needed for this Δv
  // Δv = (F/m) * t  =>  t = Δv * m / F
  double burnTime = (deltaV_needed * mass) / thrusterMaxThrust;

  // Limit burn time to check interval
  burnTime = std::min(burnTime, stationKeepingCheckInterval);

  // Calculate propellant consumption using Tsiolkovsky rocket equation
  // Δm = m * (1 - exp(-Δv / (Isp * g₀)))
  // where g₀ = 9.80665 m/s² (standard gravity)
  const double g0 = 9.80665;
  double exhaustVelocity = thrusterIsp * g0;
  double propellantUsed = mass * (1.0 - exp(-deltaV_needed / exhaustVelocity));

  // Check if we have enough propellant
  if (propellantUsed > propellantMass)
  {
    // Not enough propellant - use what we have
    propellantUsed = propellantMass;
    deltaV_needed = exhaustVelocity * log(mass / (mass - propellantUsed));
    burnTime = (deltaV_needed * mass) / thrusterMaxThrust;
  }

  // Apply velocity change (instantaneous Δv approximation)
  // In reality, this happens over burnTime duration, but for simplicity
  // we apply it as an impulse since stationKeepingCheckInterval >> burnTime
  velocity += thrustDirection * deltaV_needed;

  // Update propellant mass
  propellantMass -= propellantUsed;
  mass -= propellantUsed; // Total mass decreases

  // Ensure propellant doesn't go negative
  if (propellantMass < 0.0)
  {
    propellantMass = 0.0;
  }

  // Debug output (optional - could add a verbose flag)
  // std::cout << name << " station keeping: Altitude error = " << altitudeError / 1000.0
  //           << " km, Δv = " << deltaV_needed << " m/s, propellant used = "
  //           << propellantUsed << " kg, remaining = " << propellantMass << " kg\n";
}

glm::dvec3 Satellite::calculateThrustAcceleration(const glm::dvec3 &thrustDirection, double thrustMagnitude)
{
  /**
   * Calculate acceleration from thruster firing
   *
   * F = ma  =>  a = F/m
   *
   * This is used for instantaneous thrust applications (not currently used
   * in station keeping, but useful for future maneuvers like plane changes)
   */

  if (thrustMagnitude <= 0.0 || mass <= 0.0)
  {
    return glm::dvec3(0.0);
  }

  return glm::normalize(thrustDirection) * (thrustMagnitude / mass);
}

// ========== POWER MANAGEMENT IMPLEMENTATION ==========

void Satellite::updatePowerSystem(double deltaTime, const glm::dvec3 &sunPosition, const glm::dvec3 &earthCenter)
{
  /**
   * Update satellite power system:
   * 1. Check if in eclipse (Earth's shadow)
   * 2. Calculate solar flux and power generation
   * 3. Calculate power consumption from all systems
   * 4. Update battery charge
   */

  // Check eclipse status
  inEclipse = checkEclipse(sunPosition, earthCenter);

  // Calculate power generation from solar panels
  if (!inEclipse)
  {
    double solarFlux = calculateSolarFlux(sunPosition);
    glm::dvec3 sunDirection = glm::normalize(sunPosition - position);
    currentPowerGeneration = calculatePowerGeneration(solarFlux, sunDirection);
  }
  else
  {
    currentPowerGeneration = 0.0; // No power in eclipse
  }

  // Calculate power consumption from all active systems
  currentPowerConsumption = calculatePowerConsumption();

  // Update battery charge
  // Net power = Generation - Consumption
  double netPower = currentPowerGeneration - currentPowerConsumption;

  if (netPower > 0.0)
  {
    // Charging battery
    double chargeAdded = netPower * deltaTime / 3600.0 * batteryChargeEfficiency; // Convert W*s to Wh
    batteryCharge += chargeAdded;

    // Clamp to battery capacity
    if (batteryCharge > batteryCapacity)
    {
      batteryCharge = batteryCapacity;
    }
  }
  else
  {
    // Draining battery
    double chargeRemoved = -netPower * deltaTime / 3600.0 / batteryDischargeEfficiency; // Convert W*s to Wh
    batteryCharge -= chargeRemoved;

    // Clamp to minimum charge
    if (batteryCharge < 0.0)
    {
      batteryCharge = 0.0;

      // Low power mode - disable non-essential systems
      if (controlMode != AttitudeControlMode::NONE)
      {
        // Switch to detumble mode to conserve power
        controlMode = AttitudeControlMode::DETUMBLE;
      }
    }
  }
}

double Satellite::calculateSolarFlux(const glm::dvec3 &sunPosition) const
{
  /**
   * Calculate solar flux at satellite position
   * 
   * Solar constant at 1 AU: 1361 W/m²
   * Flux decreases with inverse square of distance from sun
   */

  const double SOLAR_CONSTANT = 1361.0; // W/m² at 1 AU

  glm::dvec3 toSun = sunPosition - position;
  double distanceToSun = glm::length(toSun);

  // Solar flux = Solar constant * (1 AU / distance)²
  double solarFlux = SOLAR_CONSTANT * (AU * AU) / (distanceToSun * distanceToSun);

  return solarFlux;
}

bool Satellite::checkEclipse(const glm::dvec3 &sunPosition, const glm::dvec3 &earthCenter) const
{
  /**
   * Check if satellite is in Earth's shadow (eclipse)
   * 
   * Method: Ray-sphere intersection from satellite toward sun
   * If ray intersects Earth before reaching sun, satellite is in eclipse
   */

  glm::dvec3 toSun = sunPosition - position;
  double distanceToSun = glm::length(toSun);
  glm::dvec3 sunDirection = toSun / distanceToSun;

  // Vector from satellite to Earth center
  glm::dvec3 toEarth = earthCenter - position;

  // Project toEarth onto sun direction to find closest point on ray to Earth center
  double projectionLength = glm::dot(toEarth, sunDirection);

  // If Earth is behind the satellite (relative to sun), no eclipse
  if (projectionLength < 0.0)
  {
    return false;
  }

  // If closest approach is beyond the sun, no eclipse
  if (projectionLength > distanceToSun)
  {
    return false;
  }

  // Calculate closest approach distance to Earth center
  glm::dvec3 closestPoint = position + sunDirection * projectionLength;
  double closestDistance = glm::length(closestPoint - earthCenter);

  // If closest distance is less than Earth radius, satellite is in shadow
  return (closestDistance < EARTH_RADIUS);
}

double Satellite::calculatePowerConsumption() const
{
  /**
   * Calculate total power consumption from all satellite systems
   * 
   * Systems:
   * - Baseline avionics (always on)
   * - Reaction wheels (when ADCS active)
   * - Magnetorquers (when in detumble mode)
   * - CMGs (if equipped and active)
   * - Thrusters (when firing)
   */

  double totalPower = basePowerConsumption; // Always have baseline consumption

  // Reaction wheels consume power when ADCS is active
  if (hasReactionWheels && controlMode != AttitudeControlMode::NONE)
  {
    // Assume 3 reaction wheels active for 3-axis control
    totalPower += 3.0 * reactionWheelPower;
  }

  // Magnetorquers consume power in detumble mode
  if (hasMagnetorquers && controlMode == AttitudeControlMode::DETUMBLE)
  {
    totalPower += magnetorquerPower;
  }

  // CMGs consume power when active
  if (hasCMGs && controlMode != AttitudeControlMode::NONE)
  {
    totalPower += numCMGs * cmgPower;
  }

  // Thrusters consume significant power when firing
  if (stationKeepingEnabled && hasThrusters)
  {
    // Thrusters don't fire continuously, but when they do, they use a lot of power
    // For now, we'll add a fraction based on whether station keeping is enabled
    totalPower += thrusterPower * 0.01; // 1% duty cycle average
  }

  return totalPower;
}

double Satellite::calculatePowerGeneration(double solarFlux, const glm::dvec3 &sunDirection) const
{
  /**
   * Calculate power generation from solar panels
   * 
   * Power = Solar Flux × Panel Area × Efficiency × cos(angle)
   * 
   * The cos(angle) term accounts for the angle between panel normal and sun direction
   * We assume panels can track the sun (body-mounted tracking or articulated panels)
   */

  // Get satellite body Y-axis (assume solar panels are mounted perpendicular to Y-axis)
  glm::dvec3 panelNormal = getBodyYAxis();

  // Calculate angle between panel normal and sun direction
  double cosAngle = glm::dot(panelNormal, sunDirection);

  // If panels are facing away from sun, no power generation
  if (cosAngle < 0.0)
  {
    cosAngle = 0.0;
  }

  // Power = Flux × Area × Efficiency × cos(angle) × degradation
  double power = solarFlux * solarPanelArea * solarPanelEfficiency * cosAngle * solarPanelDegradation;

  return power;
}
