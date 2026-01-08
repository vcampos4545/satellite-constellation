#include "Satellite.h"
#include "FlightSoftwareTask.h"
#include "Constants.h"
#include "RK4Integrator.h"
#include "GravityModel.h"
#include "EnvironmentalModels.h"
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

Satellite::Satellite(const Orbit &orbit, const glm::dvec3 &initPos, const glm::dvec3 &initVel, int planeId, int indexInPlane, const std::string &name, SatelliteType type)
    : orbit(orbit),
      position(initPos),
      velocity(initVel),
      planeId(planeId),
      indexInPlane(indexInPlane),
      name(name),
      type(type),
      quaternion(1.0, 0.0, 0.0, 0.0), // Identity quaternion (no rotation)
      angularVelocity(0.0, 0.0, 0.0)  // No initial rotation
{
  autoTunePID();

  // Add initial position to trail
  orbitPath.push_back(position);
}

glm::dvec3 Satellite::calculateAcceleration(const glm::dvec3 &pos, const glm::dvec3 &vel, const glm::dvec3 &earthCenter, double earthMass, const glm::dvec3 &sunPos, const glm::dvec3 &moonPos) const
{
  // Calculate all acceleration components using modular gravity and environmental models

  // Earth gravity: point mass + non-spherical perturbations (J2, J3, J4)
  glm::dvec3 gravAccelEarth = GravityModel::calculatePointMassGravity(pos, earthCenter, earthMass);
  glm::dvec3 gravAccelEarthJ = GravityModel::calculateZonalHarmonics(pos, earthCenter, earthMass);

  // Third-body perturbations (Moon and Sun) using differential gravity
  glm::dvec3 gravAccelMoon = GravityModel::calculateThirdBodyPerturbation(pos, earthCenter, moonPos, MOON_MASS);
  glm::dvec3 gravAccelSun = GravityModel::calculateThirdBodyPerturbation(pos, earthCenter, sunPos, SUN_MASS);

  // Non-gravitational perturbations
  glm::dvec3 dragAccel = EnvironmentalModels::calculateAtmosphericDrag(pos, vel, earthCenter, mass, crossSectionalArea, dragCoefficient);
  glm::dvec3 srpAccel = EnvironmentalModels::calculateSolarRadiationPressure(pos, sunPos, earthCenter, mass, crossSectionalArea, reflectivity);

  // Total acceleration (all forces combined)
  // Earth: point mass + J2/J3/J4 perturbations
  // Third bodies: Moon + Sun
  // Non-gravitational: Drag + Solar radiation pressure
  return gravAccelEarth + gravAccelEarthJ + gravAccelMoon + gravAccelSun + dragAccel + srpAccel;
}

void Satellite::update(double deltaTime, const glm::dvec3 &earthCenter, double earthMass, const glm::dvec3 &sunPosition, const glm::dvec3 &moonPosition)
{
  /* Main satellite update function.  Runs every frame */
  auto accelFunc = [this, &earthCenter, earthMass, &sunPosition, &moonPosition](const glm::dvec3 &pos, const glm::dvec3 &vel) -> glm::dvec3
  {
    return calculateAcceleration(pos, vel, earthCenter, earthMass, sunPosition, moonPosition);
  };

  RK4Integrator::integratePositionVelocity(position, velocity, deltaTime, accelFunc);

  // ========== ORBIT PATH HISTORY ==========
  // Increment update iteration counter
  updateIterationCount++;

  // Add current position to historical trail every N iterations
  // This gives smooth trails regardless of time warp speed
  // Saves complete history since simulation start
  if (updateIterationCount % orbitPathSaveInterval == 0)
  {
    orbitPath.push_back(position);
  }

  // ========== SENSOR UPDATES ==========
  // Update all sensors with current truth data (adds noise, bias, etc.)
  imu.measureAngularVelocity(angularVelocity); // Update IMU with noisy measurement
  imu.updateBias(deltaTime);                   // Update IMU bias (random walk)

  // ========== FLIGHT SOFTWARE EXECUTION ==========
  // Execute injected FSW if available, otherwise fall back to old hardcoded FSW
  executeFlightSoftware(deltaTime);
}

void Satellite::executeFlightSoftware(double deltaTime)
{
  /**
   * Execute flight software task
   *
   * If custom FSW has been injected via setFlightSoftware(), execute it.
   * Otherwise, fall back to the old hardcoded runFlightSoftware() for backward compatibility.
   */

  if (flightSoftware)
  {
    // Execute injected custom FSW
    flightSoftware->execute(this, deltaTime);
  }
  else
  {
    // No flight software configured - satellite operates in passive mode
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

// ========== ACTUATOR COMMANDS (called by FSW via ADCSController) ==========

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
  auto angularAccelFunc = [this, &externalTorque](const glm::dvec3 &omega) -> glm::dvec3
  {
    glm::dvec3 angularMom = inertiaTensor * omega;
    glm::dvec3 gyroTorque = glm::cross(omega, angularMom);
    return (externalTorque - gyroTorque) / inertiaTensor;
  };

  // Lambda function to compute quaternion derivative
  auto quatDotFunc = [](const glm::dquat &q, const glm::dvec3 &omega) -> glm::dquat
  {
    glm::dquat omegaQuat(0.0, omega.x, omega.y, omega.z);
    return 0.5 * omegaQuat * q;
  };

  // Use modular RK4 integrator for attitude dynamics
  RK4Integrator::integrateAttitude(quaternion, angularVelocity, deltaTime, angularAccelFunc, quatDotFunc);
}

// ========== PROPULSION AND STATION KEEPING ==========

// Station keeping methods removed - now handled by StationKeepingController in FSW
// Satellite class now only exposes hardware state (position, velocity, propellant)
// and hardware control commands (setVelocity, consumePropellant, etc.)

Satellite::OrbitalElements Satellite::getOrbitalElements(const glm::dvec3 &earthCenter) const
{
  const double mu = G * EARTH_MASS;

  glm::dvec3 r = position - earthCenter;
  double currentRadius = glm::length(r);
  double currentVelocityMag = glm::length(velocity);

  // Calculate specific angular momentum: h = r × v
  glm::dvec3 h = glm::cross(r, velocity);
  double h_mag = glm::length(h);

  // Calculate eccentricity vector: e = (v × h)/μ - r/|r|
  glm::dvec3 e_vec = (glm::cross(velocity, h) / mu) - (r / currentRadius);
  double eccentricity = glm::length(e_vec);

  // Calculate semi-major axis from vis-viva equation
  double specificEnergy = 0.5 * currentVelocityMag * currentVelocityMag - mu / currentRadius;
  double semiMajorAxis = -mu / (2.0 * specificEnergy);

  // Calculate periapsis and apoapsis
  double periapsis = semiMajorAxis * (1.0 - eccentricity);
  double apoapsis = semiMajorAxis * (1.0 + eccentricity);

  // Calculate inclination
  glm::dvec3 h_hat = h / h_mag;
  double inclination = acos(glm::clamp(h_hat.z, -1.0, 1.0)); // Angle from Z-axis

  // Calculate true anomaly
  double trueAnomaly = 0.0;
  if (eccentricity > 1e-6)
  {
    glm::dvec3 r_hat = r / currentRadius;
    double radialVelocity = glm::dot(velocity, r_hat);
    double cos_nu = glm::dot(e_vec, r) / (eccentricity * currentRadius);
    trueAnomaly = acos(glm::clamp(cos_nu, -1.0, 1.0));
    if (radialVelocity < 0.0)
    {
      trueAnomaly = 2.0 * PI - trueAnomaly;
    }
  }

  return OrbitalElements{semiMajorAxis, eccentricity, periapsis, apoapsis, inclination, trueAnomaly};
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

// Power management methods removed - now handled by PowerManager in FSW
// Satellite class now only exposes hardware state (getBatteryCharge, etc.)
// and hardware control commands (setBatteryCharge, setPowerGeneration, etc.)

// ========== ORBIT PREDICTION ==========

void Satellite::calculatePredictedOrbit(const glm::dvec3 &earthCenter, double earthMass,
                                        const glm::dvec3 &sunPosition, const glm::dvec3 &moonPosition,
                                        int numPoints)
{
  /**
   * Calculate future orbit path using RK4 integration
   *
   * This predicts where the satellite will be in the future based on current state,
   * useful for trajectory planning and visualization.
   *
   * Parameters:
   *   predictionDuration: How far into the future to predict (seconds)
   *   numPoints: Number of points to calculate along the predicted path
   */

  // Estimate period using Kepler's third law: T = 2π√(a³/μ) where μ = G*M
  double a = glm::length(position - earthCenter);
  double mu = G * earthMass;
  double predictionDuration = 2.0 * PI * sqrt((a * a * a) / mu);

  predictedOrbitPath.clear();
  predictedOrbitPath.reserve(numPoints);

  // Start from current state
  glm::dvec3 pred_pos = position;
  glm::dvec3 pred_vel = velocity;

  // Time step for prediction
  double dt = predictionDuration / (double)numPoints;

  // Add current position as first point
  predictedOrbitPath.push_back(pred_pos);

  // Propagate forward using RK4 integrator
  auto accelFunc = [this, &earthCenter, earthMass, &sunPosition, &moonPosition](const glm::dvec3 &pos, const glm::dvec3 &vel) -> glm::dvec3
  {
    return calculateAcceleration(pos, vel, earthCenter, earthMass, sunPosition, moonPosition);
  };

  for (int i = 1; i < numPoints; ++i)
  {
    // Use modular RK4 integrator
    RK4Integrator::integratePositionVelocity(pred_pos, pred_vel, dt, accelFunc);

    // Save this predicted position
    predictedOrbitPath.push_back(pred_pos);
  }
}

// ========== ALERT SYSTEM IMPLEMENTATION ==========

void Satellite::checkTelemetryLimits()
{
  /**
   * Check all satellite telemetry against operational limits
   * Generate alerts for out-of-spec conditions
   *
   * This mirrors flight software limit checking that would generate
   * alerts for ground operators
   */

  // ========== POWER SYSTEM CHECKS ==========

  // Battery charge critical
  if (batteryCharge < batteryCapacity * 0.1)
  {
    alertSystem.addAlert("Battery critically low: " + std::to_string((int)getBatteryPercentage()) + "%",
                         AlertSeverity::CRITICAL, AlertCategory::POWER);
  }
  else if (batteryCharge < batteryCapacity * 0.2)
  {
    alertSystem.addAlert("Battery low: " + std::to_string((int)getBatteryPercentage()) + "%",
                         AlertSeverity::WARNING, AlertCategory::POWER);
  }

  // Power balance negative while in eclipse
  if (inEclipse && batteryCharge < batteryCapacity * 0.5)
  {
    alertSystem.addAlert("In eclipse with low battery: " + std::to_string((int)getBatteryPercentage()) + "%",
                         AlertSeverity::WARNING, AlertCategory::POWER);
  }

  // Solar panel degradation
  if (solarPanelDegradation < 0.8)
  {
    alertSystem.addAlert("Solar panel degradation: " + std::to_string((int)(solarPanelDegradation * 100)) + "%",
                         AlertSeverity::WARNING, AlertCategory::POWER);
  }

  // ========== ADCS CHECKS ==========

  // High angular velocity (tumbling)
  double omegaMag = glm::length(angularVelocity);
  if (omegaMag > 0.1) // 0.1 rad/s = ~5.7 deg/s
  {
    alertSystem.addAlert("High angular velocity: " + std::to_string(omegaMag * 180.0 / PI) + " deg/s",
                         AlertSeverity::WARNING, AlertCategory::ADCS);
  }

  // Reaction wheel saturation
  if (hasReactionWheels)
  {
    double maxWheelMomentum = std::max({std::abs(reactionWheelMomentum.x),
                                        std::abs(reactionWheelMomentum.y),
                                        std::abs(reactionWheelMomentum.z)});
    if (maxWheelMomentum > reactionWheelMaxMomentum * 0.9)
    {
      alertSystem.addAlert("Reaction wheel near saturation: " + std::to_string((int)((maxWheelMomentum / reactionWheelMaxMomentum) * 100)) + "%",
                           AlertSeverity::WARNING, AlertCategory::ADCS);
    }
  }

  // ADCS disabled while not in safe orientation
  if (controlMode == AttitudeControlMode::NONE && omegaMag > 0.01)
  {
    alertSystem.addAlert("ADCS disabled while tumbling",
                         AlertSeverity::CRITICAL, AlertCategory::ADCS);
  }

  // ========== PROPULSION CHECKS ==========

  // Low propellant
  if (hasThrusters)
  {
    double propellantPercent = getPropellantFraction() * 100.0;
    if (propellantPercent < 5.0)
    {
      alertSystem.addAlert("Propellant critically low: " + std::to_string((int)propellantPercent) + "%",
                           AlertSeverity::CRITICAL, AlertCategory::PROPULSION);
    }
    else if (propellantPercent < 20.0)
    {
      alertSystem.addAlert("Propellant low: " + std::to_string((int)propellantPercent) + "%",
                           AlertSeverity::WARNING, AlertCategory::PROPULSION);
    }
  }

  // ========== ORBITAL CHECKS ==========

  // Altitude checks
  double altitude = (glm::length(position) - EARTH_RADIUS) / 1e3; // km

  // Too low - atmospheric drag will cause rapid decay
  if (altitude < 200.0)
  {
    alertSystem.addAlert("Altitude critically low: " + std::to_string((int)altitude) + " km - rapid decay expected",
                         AlertSeverity::CRITICAL, AlertCategory::ORBIT);
  }
  else if (altitude < 300.0)
  {
    alertSystem.addAlert("Altitude low: " + std::to_string((int)altitude) + " km - increased drag",
                         AlertSeverity::WARNING, AlertCategory::ORBIT);
  }

  // Station keeping checks
  if (stationKeepingEnabled && targetSemiMajorAxis > 0.0)
  {
    double currentRadius = glm::length(position);
    double altitudeError = (targetSemiMajorAxis - currentRadius) / 1e3; // km

    if (std::abs(altitudeError) > 5.0)
    {
      alertSystem.addAlert("Orbit altitude error: " + std::to_string((int)altitudeError) + " km from target",
                           AlertSeverity::WARNING, AlertCategory::ORBIT);
    }
  }
}
