#ifndef SATELLITE_H
#define SATELLITE_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <string>
#include <memory>
#include "Orbit.h"
#include "AlertSystem.h"

// Forward declaration
class FlightSoftwareTask;

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
  Satellite(const Orbit &orbit, const glm::dvec3 &initPos, const glm::dvec3 &initVel, const glm::vec3 &color, int planeId = 0, int indexInPlane = 0, const std::string &name = "");

  // Update physics
  void update(double deltaTime, const glm::dvec3 &earthCenter, double earthMass, const glm::dvec3 &sunPosition, const glm::dvec3 &moonPosition);

  // ========== FLIGHT SOFTWARE INTERFACE ==========
  // Set custom flight software for this satellite
  void setFlightSoftware(std::shared_ptr<FlightSoftwareTask> fsw) { flightSoftware = fsw; }

  // Get current flight software
  std::shared_ptr<FlightSoftwareTask> getFlightSoftware() const { return flightSoftware; }

  // Execute flight software (called internally by update())
  void executeFlightSoftware(double deltaTime);

  // DEPRECATED: Old hardcoded flight software (kept for backward compatibility)
  // Use setFlightSoftware() instead to inject custom FSW
  void runFlightSoftware(double deltaTime, const glm::dvec3 &earthCenter, const glm::dvec3 &sunPosition);

  // ADCS Control Loop (mirrors flight software architecture)
  // PUBLIC: FSW implementations can call this to perform attitude control
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
  const std::vector<glm::dvec3> &getPredictedOrbit() const { return predictedOrbitPath; }
  const std::vector<glm::dvec3> &getFootprintCircle() const { return footprintCircle; }
  int getPlaneId() const { return planeId; }
  int getIndexInPlane() const { return indexInPlane; }
  std::string getName() const { return name; }

  // Orbit prediction
  void calculatePredictedOrbit(const glm::dvec3 &earthCenter, double earthMass,
                               const glm::dvec3 &sunPosition, const glm::dvec3 &moonPosition, int numPoints = 500);
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

  // Station keeping control
  void enableStationKeeping(bool enable, double targetAltitude = 0.0);
  double getPropellantMass() const { return propellantMass; }
  double getPropellantFraction() const { return propellantMass / propellantMassInitial; }
  bool hasPropellant() const { return propellantMass > 0.1; } // At least 100g remaining

  // Maneuver state telemetry
  int getManeuverStateInt() const { return static_cast<int>(maneuverState); }
  const char *getManeuverStateString() const
  {
    switch (maneuverState)
    {
    case ManeuverState::IDLE:
      return "IDLE";
    case ManeuverState::BURN1_PENDING:
      return "BURN1 PENDING";
    case ManeuverState::COASTING:
      return "COASTING";
    case ManeuverState::BURN2_PENDING:
      return "BURN2 PENDING";
    default:
      return "UNKNOWN";
    }
  }
  double getBurn1DeltaV() const { return burn1DeltaV; }
  double getBurn2DeltaV() const { return burn2DeltaV; }

  // Orbital elements calculation
  struct OrbitalElements
  {
    double semiMajorAxis; // meters
    double eccentricity;  // dimensionless
    double periapsis;     // meters
    double apoapsis;      // meters
    double inclination;   // radians
    double trueAnomaly;   // radians
  };
  OrbitalElements getOrbitalElements(const glm::dvec3 &earthCenter) const;

  // Actuator configuration
  void enableReactionWheels(bool enable) { hasReactionWheels = enable; }
  void enableMagnetorquers(bool enable) { hasMagnetorquers = enable; }
  void enableCMGs(bool enable) { hasCMGs = enable; }

  // Power system getters
  double getBatteryCharge() const { return batteryCharge; }
  double getBatteryCapacity() const { return batteryCapacity; }
  double getBatteryPercentage() const { return (batteryCharge / batteryCapacity) * 100.0; }
  double getPowerGeneration() const { return currentPowerGeneration; }
  double getPowerConsumption() const { return currentPowerConsumption; }
  double getNetPower() const { return currentPowerGeneration - currentPowerConsumption; }
  bool isInEclipse() const { return inEclipse; }
  bool hasSufficientPower() const { return batteryCharge > (batteryCapacity * 0.1); } // >10% battery

  // ========== FSW ACCESSIBLE METHODS ==========
  // Power management (FSW can call these)
  void updatePowerSystem(double deltaTime, const glm::dvec3 &sunPosition, const glm::dvec3 &earthCenter);

  // Station keeping (FSW can call these)
  void performStationKeeping(double deltaTime, const glm::dvec3 &earthCenter);
  bool getStationKeepingEnabled() const { return stationKeepingEnabled; }
  double getTargetSemiMajorAxis() const { return targetSemiMajorAxis; }

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

  // Alert system
  AlertSystem &getAlertSystem() { return alertSystem; }
  const AlertSystem &getAlertSystem() const { return alertSystem; }
  void checkTelemetryLimits(); // Check all telemetry against limits and generate alerts

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

  // ========== PROPULSION METHODS ==========
  // performStationKeeping is now public (declared above in FSW ACCESSIBLE METHODS)
  glm::dvec3 calculateThrustAcceleration(const glm::dvec3 &thrustDirection, double thrustMagnitude);

  // ========== POWER MANAGEMENT METHODS ==========
  // updatePowerSystem is now public (declared above in FSW ACCESSIBLE METHODS)
  double calculateSolarFlux(const glm::dvec3 &sunPosition) const;
  bool checkEclipse(const glm::dvec3 &sunPosition, const glm::dvec3 &earthCenter) const;
  double calculatePowerConsumption() const;
  double calculatePowerGeneration(double solarFlux, const glm::dvec3 &sunDirection) const;

  glm::dvec3 calculateGravitationalAcceleration(const glm::dvec3 &pos, const glm::dvec3 &bodyPos, double bodyMass) const;
  glm::dvec3 calculateZonalHarmonicsAcceleration(const glm::dvec3 &pos, const glm::dvec3 &earthCenter, double earthMass) const;
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

  // ========== PROPULSION SYSTEM (Station Keeping) ==========
  // Hall Effect Thruster (typical for Starlink-class satellites)
  bool hasThrusters = false;
  double thrusterMaxThrust = 0.25;     // Maximum thrust per thruster (N) - typical Hall thruster
  double thrusterIsp = 1800.0;         // Specific impulse (seconds) - Hall effect typical range
  double propellantMass = 50.0;        // Propellant mass (kg) - Krypton or Xenon
  double propellantMassInitial = 50.0; // Initial propellant mass (for tracking consumption)
  double thrusterEfficiency = 0.55;    // Thruster efficiency (0-1) - typical for Hall thrusters

  // Station keeping configuration
  bool stationKeepingEnabled = false;
  double targetSemiMajorAxis = 0.0;          // Target semi-major axis (m) - orbit to maintain
  double stationKeepingDeadband = 500.0;     // Altitude deadband (m) - don't burn unless outside this
  double stationKeepingCheckInterval = 60.0; // Check orbit every N seconds
  double timeSinceLastStationKeepingCheck = 0.0;

  // Multi-burn maneuver state for Hohmann-like transfers
  enum class ManeuverState
  {
    IDLE,          // No active maneuver
    BURN1_PENDING, // Waiting for periapsis to execute first burn
    COASTING,      // Coasting to apoapsis after first burn
    BURN2_PENDING  // Waiting for apoapsis to execute second burn
  };
  ManeuverState maneuverState = ManeuverState::IDLE;
  double burn1DeltaV = 0.0;     // Delta-V for first burn (at periapsis)
  double burn2DeltaV = 0.0;     // Delta-V for second burn (at apoapsis)
  double lastTrueAnomaly = 0.0; // Track orbital position for burn timing

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

  // ========== POWER SYSTEM ==========
  // Battery properties (Lithium-ion typical for satellites)
  double batteryCapacity = 100.0;           // Battery capacity in Watt-hours (Wh)
  double batteryCharge = 100.0;             // Current battery charge (Wh) - start fully charged
  double batteryVoltage = 28.0;             // Battery bus voltage (V) - typical satellite bus voltage
  double batteryMinCharge = 10.0;           // Minimum safe charge (Wh) - 10% reserve
  double batteryChargeEfficiency = 0.95;    // Charging efficiency (95%)
  double batteryDischargeEfficiency = 0.98; // Discharge efficiency (98%)

  // Solar panel properties
  double solarPanelArea = 20.0;       // Total solar panel area (m²) - typical for small sat
  double solarPanelEfficiency = 0.30; // Solar cell efficiency (30% - multi-junction cells)
  double solarPanelDegradation = 1.0; // Panel degradation factor (1.0 = new, decreases over time)

  // Power consumption rates (Watts)
  double basePowerConsumption = 50.0; // Avionics, computer, comms baseline (W)
  double reactionWheelPower = 10.0;   // Power per active reaction wheel (W)
  double magnetorquerPower = 5.0;     // Power for magnetorquers (W)
  double cmgPower = 15.0;             // Power per CMG (W)
  double thrusterPower = 250.0;       // Power for Hall effect thruster (W)

  // Current power state
  double currentPowerGeneration = 0.0;  // Current power generation (W)
  double currentPowerConsumption = 0.0; // Current power consumption (W)
  bool inEclipse = false;               // Whether satellite is in Earth's shadow

  std::vector<glm::dvec3> footprintCircle;    // Positions for drawing footprint circle
  std::vector<glm::dvec3> orbitPath;          // Historical position trail (actual path traveled) - complete history since simulation start
  std::vector<glm::dvec3> predictedOrbitPath; // Predicted future orbit path (calculated on demand)
  int orbitPathSaveInterval = 10;             // Save position every N update iterations
  int updateIterationCount = 0;               // Counter for update iterations

  int planeId;      // Orbital plane identifier
  int indexInPlane; // Index of this satellite within its plane
  std::string name; // Satellite name/identifier

  // Alert system
  AlertSystem alertSystem;
  double timeSinceLastAlertCheck = 0.0; // Track time between alert checks

  // ========== FLIGHT SOFTWARE ==========
  // Injected flight software task (executes autonomous behavior)
  std::shared_ptr<FlightSoftwareTask> flightSoftware;
};

#endif // SATELLITE_H
