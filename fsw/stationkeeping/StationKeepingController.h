#ifndef STATION_KEEPING_CONTROLLER_H
#define STATION_KEEPING_CONTROLLER_H

#include <glm/glm.hpp>

// Forward declaration
class Satellite;

/**
 * Station Keeping Controller
 *
 * FSW controller responsible for orbital maintenance:
 * - Monitors orbit decay from atmospheric drag
 * - Plans optimal maneuvers (single burn or two-burn Hohmann transfer)
 * - Executes burns at correct orbital positions
 * - Manages propellant consumption
 *
 * Uses two-burn Hohmann-like transfer to maintain circular orbit:
 * 1. Burn 1 at periapsis: Raise apoapsis to target altitude
 * 2. Burn 2 at apoapsis: Circularize by raising periapsis
 * For nearly circular orbits (e < 0.01), uses single optimized burn
 *
 * The Satellite class provides hardware state (position, velocity, mass, propellant)
 * and hardware control commands (applyDeltaV, consumePropellant, etc.).
 * StationKeepingController implements the orbital mechanics and decision-making.
 */
class StationKeepingController
{
public:
  StationKeepingController() = default;
  ~StationKeepingController() = default;

  /**
   * Execute station keeping algorithm
   * - Calculate orbital elements
   * - Detect orbit decay
   * - Plan and execute maneuvers
   */
  void performStationKeeping(Satellite *satellite, double deltaTime,
                             const glm::dvec3 &earthCenter);

  /**
   * Reset controller state (for new satellites or maneuver abort)
   */
  void reset();

  // State query methods
  std::string getManeuverStateName() const;
  bool isManeuverActive() const { return maneuverState != ManeuverState::IDLE; }

private:
  // Maneuver state machine
  enum class ManeuverState
  {
    IDLE,          // No active maneuver
    BURN1_PENDING, // Waiting for periapsis to execute first burn
    COASTING,      // Coasting to apoapsis after first burn
    BURN2_PENDING  // Waiting for apoapsis to execute second burn
  };

  ManeuverState maneuverState = ManeuverState::IDLE;
  double burn1DeltaV = 0.0; // Delta-V for first burn (at periapsis)
  double burn2DeltaV = 0.0; // Delta-V for second burn (at apoapsis)
  double timeSinceLastCheck = 0.0;

  /**
   * Calculate current orbital elements from position and velocity
   */
  struct OrbitalElements
  {
    double eccentricity;
    double semiMajorAxis;
    double periapsis;
    double apoapsis;
    double trueAnomaly;
    double radialVelocity;
    glm::dvec3 eccentricityVector;
  };

  OrbitalElements calculateOrbitalElements(const glm::dvec3 &position,
                                           const glm::dvec3 &velocity,
                                           const glm::dvec3 &earthCenter,
                                           double earthMass) const;

  /**
   * Check if satellite is at periapsis or apoapsis
   */
  bool isAtPeriapsis(double trueAnomaly) const;
  bool isAtApoapsis(double trueAnomaly) const;

  /**
   * Plan maneuver (single or two-burn) based on orbit state
   */
  void planManeuver(const OrbitalElements &orbit, double targetSemiMajorAxis,
                    double earthMass);

  /**
   * Execute burn at current position
   * Returns true if burn was executed
   */
  bool executeBurn(Satellite *satellite, double deltaV);
};

#endif // STATION_KEEPING_CONTROLLER_H
