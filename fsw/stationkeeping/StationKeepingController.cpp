#include "StationKeepingController.h"
#include "Satellite.h"
#include "Constants.h"
#include <iostream>
#include <cmath>

void StationKeepingController::performStationKeeping(Satellite *satellite, double deltaTime,
                                                     const glm::dvec3 &earthCenter)
{
  /**
   * STATION KEEPING ALGORITHM - TWO-BURN HOHMANN TRANSFER
   *
   * Purpose: Maintain circular orbit by compensating for atmospheric drag
   *
   * Problem: Single prograde burn raises apoapsis but not periapsis, creating eccentric orbit
   * Solution: Two-burn Hohmann-like transfer to restore circular orbit
   *
   * Strategy:
   * 1. Detect when orbit has decayed (periapsis or semi-major axis below target)
   * 2. Plan two-burn transfer:
   *    - Burn 1 at periapsis: Raise apoapsis to target altitude
   *    - Burn 2 at apoapsis: Circularize by raising periapsis to match apoapsis
   * 3. Execute burns at correct orbital positions
   *
   * For small eccentricity (e < 0.01), use single optimized burn at periapsis
   */

  // Get satellite state
  glm::dvec3 position = satellite->getPosition();
  glm::dvec3 velocity = satellite->getVelocity();
  double targetSemiMajorAxis = satellite->getTargetSemiMajorAxis();

  // Calculate current orbital elements
  OrbitalElements orbit = calculateOrbitalElements(position, velocity, earthCenter, EARTH_MASS);

  // Check if at periapsis or apoapsis
  bool atPeriapsis = isAtPeriapsis(orbit.trueAnomaly);
  bool atApoapsis = isAtApoapsis(orbit.trueAnomaly);

  // ========== STATE MACHINE FOR MULTI-BURN MANEUVER ==========
  if (maneuverState == ManeuverState::IDLE)
  {
    // Update timer
    timeSinceLastCheck += deltaTime;

    // Only check periodically
    double checkInterval = satellite->getStationKeepingCheckInterval();
    if (timeSinceLastCheck < checkInterval)
    {
      return;
    }
    timeSinceLastCheck = 0.0;

    // Check if orbit needs correction
    double altitudeError = targetSemiMajorAxis - orbit.semiMajorAxis;
    double deadband = satellite->getStationKeepingDeadband();

    if (fabs(altitudeError) < deadband && orbit.eccentricity < 0.01)
    {
      return; // Orbit is good, no correction needed
    }

    // Plan maneuver
    planManeuver(orbit, targetSemiMajorAxis, EARTH_MASS);
  }
  else if (maneuverState == ManeuverState::BURN1_PENDING)
  {
    // Wait for periapsis to execute first burn
    if (atPeriapsis)
    {
      bool success = executeBurn(satellite, burn1DeltaV);

      if (success)
      {
        // Transition to next state
        if (fabs(burn2DeltaV) > 0.1) // If second burn needed
        {
          maneuverState = ManeuverState::COASTING;
          std::cout << satellite->getName() << ": Coasting to apoapsis...\n";
        }
        else
        {
          maneuverState = ManeuverState::IDLE; // Single burn complete
          std::cout << satellite->getName() << ": Single burn maneuver complete\n";
        }
      }
    }
  }
  else if (maneuverState == ManeuverState::COASTING)
  {
    // Coast to apoapsis for second burn
    if (atApoapsis)
    {
      maneuverState = ManeuverState::BURN2_PENDING;
    }
  }
  else if (maneuverState == ManeuverState::BURN2_PENDING)
  {
    // Execute burn 2 at apoapsis
    if (atApoapsis)
    {
      bool success = executeBurn(satellite, burn2DeltaV);

      if (success)
      {
        maneuverState = ManeuverState::IDLE; // Maneuver complete
        std::cout << satellite->getName() << ": Two-burn maneuver complete - orbit circularized\n";
      }
    }
  }
}

void StationKeepingController::reset()
{
  maneuverState = ManeuverState::IDLE;
  burn1DeltaV = 0.0;
  burn2DeltaV = 0.0;
  timeSinceLastCheck = 0.0;
}

std::string StationKeepingController::getManeuverStateName() const
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

StationKeepingController::OrbitalElements
StationKeepingController::calculateOrbitalElements(const glm::dvec3 &position,
                                                   const glm::dvec3 &velocity,
                                                   const glm::dvec3 &earthCenter,
                                                   double earthMass) const
{
  const double mu = G * earthMass;

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

  // Calculate radial velocity
  glm::dvec3 r_hat = r / currentRadius;
  double radialVelocity = glm::dot(velocity, r_hat);

  // True anomaly (angle from periapsis)
  double trueAnomaly = 0.0;
  if (eccentricity > 1e-6)
  {
    double cos_nu = glm::dot(e_vec, r) / (eccentricity * currentRadius);
    trueAnomaly = acos(glm::clamp(cos_nu, -1.0, 1.0));
    if (radialVelocity < 0.0)
    {
      trueAnomaly = 2.0 * PI - trueAnomaly;
    }
  }

  OrbitalElements orbit;
  orbit.eccentricity = eccentricity;
  orbit.semiMajorAxis = semiMajorAxis;
  orbit.periapsis = periapsis;
  orbit.apoapsis = apoapsis;
  orbit.trueAnomaly = trueAnomaly;
  orbit.radialVelocity = radialVelocity;
  orbit.eccentricityVector = e_vec;

  return orbit;
}

bool StationKeepingController::isAtPeriapsis(double trueAnomaly) const
{
  // Within ~3° of periapsis
  return fabs(trueAnomaly) < 0.05 || fabs(trueAnomaly - 2.0 * PI) < 0.05;
}

bool StationKeepingController::isAtApoapsis(double trueAnomaly) const
{
  // Within ~3° of apoapsis
  return fabs(trueAnomaly - PI) < 0.05;
}

void StationKeepingController::planManeuver(const OrbitalElements &orbit,
                                            double targetSemiMajorAxis,
                                            double earthMass)
{
  const double mu = G * earthMass;

  if (orbit.eccentricity < 0.01)
  {
    // Orbit is nearly circular - use single optimized burn at periapsis
    // Δv = √(μ/r_current) * (√(2*r_target/(r_current + r_target)) - 1)
    // Simplified for small changes: Δv ≈ (v/2) * (Δa/a)
    double circularVelocity = sqrt(mu / targetSemiMajorAxis);
    double currentCircularVelocity = sqrt(mu / orbit.semiMajorAxis);
    burn1DeltaV = circularVelocity - currentCircularVelocity;
    burn2DeltaV = 0.0;

    maneuverState = ManeuverState::BURN1_PENDING;
    std::cout << "StationKeeping: Single burn planned, ΔV = " << burn1DeltaV << " m/s\n";
  }
  else
  {
    // Orbit is eccentric - use two-burn Hohmann transfer

    // Burn 1 at periapsis: Raise apoapsis to target altitude
    // v_periapsis_before = √(μ * (2/r_p - 1/a_current))
    double v_peri_before = sqrt(mu * (2.0 / orbit.periapsis - 1.0 / orbit.semiMajorAxis));

    // Target transfer orbit: periapsis = current periapsis, apoapsis = target altitude
    double a_transfer = (orbit.periapsis + targetSemiMajorAxis) / 2.0;
    double v_peri_transfer = sqrt(mu * (2.0 / orbit.periapsis - 1.0 / a_transfer));

    burn1DeltaV = v_peri_transfer - v_peri_before;

    // Burn 2 at apoapsis: Circularize at target altitude
    // v_apoapsis_transfer = √(μ * (2/r_target - 1/a_transfer))
    double v_apo_transfer = sqrt(mu * (2.0 / targetSemiMajorAxis - 1.0 / a_transfer));
    double v_circular_target = sqrt(mu / targetSemiMajorAxis);

    burn2DeltaV = v_circular_target - v_apo_transfer;

    maneuverState = ManeuverState::BURN1_PENDING;
    std::cout << "StationKeeping: Two-burn maneuver planned, e=" << orbit.eccentricity
              << ", ΔV1=" << burn1DeltaV << " m/s, ΔV2=" << burn2DeltaV << " m/s\n";
  }
}

bool StationKeepingController::executeBurn(Satellite *satellite, double deltaV)
{
  // Check if satellite has propellant
  if (!satellite->hasPropellant())
  {
    std::cout << satellite->getName() << ": Cannot execute burn - no propellant remaining\n";
    return false;
  }

  const double g0 = 9.80665; // Standard gravity (m/s²)
  double exhaustVelocity = satellite->getThrusterIsp() * g0;

  // Get current state
  glm::dvec3 velocity = satellite->getVelocity();
  double mass = satellite->getMass();
  double propellantMass = satellite->getPropellantMass();

  // Calculate thrust direction (prograde)
  glm::dvec3 thrustDirection = glm::normalize(velocity);

  // Calculate propellant consumption using Tsiolkovsky rocket equation
  // Δv = v_e * ln(m0/m1)
  // m1 = m0 * exp(-Δv/v_e)
  // propellant_used = m0 - m1 = m0 * (1 - exp(-Δv/v_e))
  double propellantUsed = mass * (1.0 - exp(-fabs(deltaV) / exhaustVelocity));

  // Check if we have enough propellant
  if (propellantUsed > propellantMass)
  {
    // Not enough propellant - use what we have
    propellantUsed = propellantMass;
    deltaV = exhaustVelocity * log(mass / (mass - propellantUsed));
    std::cout << satellite->getName() << ": Insufficient propellant - partial burn executed\n";
  }

  // Apply delta-V
  glm::dvec3 newVelocity = velocity + thrustDirection * deltaV;
  satellite->setVelocity(newVelocity);

  // Consume propellant and update mass
  satellite->consumePropellant(propellantUsed);

  std::cout << satellite->getName() << ": Burn executed, ΔV = " << deltaV
            << " m/s, propellant used = " << propellantUsed << " kg\n";

  return true;
}
