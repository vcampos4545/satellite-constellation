#include "Thruster.h"
#include <cmath>
#include <algorithm>

Thruster::Thruster()
    : Actuator(),
      maxThrust(10.0),              // 10 N typical small RCS thruster
      specificImpulse(220.0),       // Hydrazine monoprop
      thrustDirection(0.0, 0.0, -1.0), // -Z direction default
      mountPosition(0.0),
      timeConstant(0.02),           // 20ms buildup
      minimumImpulseBit(0.01),      // 10 mNs MIB
      powerPerThrust(0.0),          // Chemical - no significant power
      thrusterType(Type::MONOPROP),
      commandedThrottle(0.0),
      actualThrottle(0.0),
      firing(false),
      propellantMass(10.0),         // 10 kg propellant default
      propellantConsumed(0.0),
      pulseTimeRemaining(0.0)
{
}

Thruster::Thruster(const std::string &name)
    : Actuator(name),
      maxThrust(10.0),
      specificImpulse(220.0),
      thrustDirection(0.0, 0.0, -1.0),
      mountPosition(0.0),
      timeConstant(0.02),
      minimumImpulseBit(0.01),
      powerPerThrust(0.0),
      thrusterType(Type::MONOPROP),
      commandedThrottle(0.0),
      actualThrottle(0.0),
      firing(false),
      propellantMass(10.0),
      propellantConsumed(0.0),
      pulseTimeRemaining(0.0)
{
}

Thruster::Thruster(Type type, const std::string &name)
    : Actuator(name),
      thrustDirection(0.0, 0.0, -1.0),
      mountPosition(0.0),
      thrusterType(type),
      commandedThrottle(0.0),
      actualThrottle(0.0),
      firing(false),
      propellantConsumed(0.0),
      pulseTimeRemaining(0.0)
{
  // Set characteristics based on thruster type
  switch (type)
  {
  case Type::MONOPROP:
    maxThrust = 10.0;           // 10 N
    specificImpulse = 220.0;    // 220 s (hydrazine)
    timeConstant = 0.02;        // 20 ms
    minimumImpulseBit = 0.01;   // 10 mNs
    powerPerThrust = 0.0;       // Negligible
    propellantMass = 10.0;
    break;

  case Type::BIPROP:
    maxThrust = 400.0;          // 400 N (typical biprop)
    specificImpulse = 310.0;    // 310 s (MMH/NTO)
    timeConstant = 0.05;        // 50 ms
    minimumImpulseBit = 0.1;    // 100 mNs
    powerPerThrust = 0.0;
    propellantMass = 100.0;
    break;

  case Type::COLD_GAS:
    maxThrust = 1.0;            // 1 N
    specificImpulse = 70.0;     // 70 s (N2)
    timeConstant = 0.005;       // 5 ms (fast response)
    minimumImpulseBit = 0.001;  // 1 mNs
    powerPerThrust = 0.0;
    propellantMass = 2.0;
    break;

  case Type::ION:
    maxThrust = 0.1;            // 100 mN
    specificImpulse = 3000.0;   // 3000 s
    timeConstant = 1.0;         // 1 s (slow startup)
    minimumImpulseBit = 1.0;    // 1 Ns
    powerPerThrust = 25.0;      // 25 W/mN = 2.5 kW for 100 mN
    propellantMass = 50.0;      // Xenon
    break;

  case Type::HALL:
    maxThrust = 0.2;            // 200 mN
    specificImpulse = 1500.0;   // 1500 s
    timeConstant = 0.5;         // 500 ms
    minimumImpulseBit = 0.5;    // 500 mNs
    powerPerThrust = 7.5;       // 7.5 W/mN = 1.5 kW for 200 mN
    propellantMass = 50.0;
    break;
  }
}

void Thruster::update(double deltaTime)
{
  if (!enabled)
  {
    actualThrottle = 0.0;
    return;
  }

  // Handle pulse mode
  if (pulseTimeRemaining > 0.0)
  {
    pulseTimeRemaining -= deltaTime;
    if (pulseTimeRemaining <= 0.0)
    {
      // Pulse complete
      pulseTimeRemaining = 0.0;
      commandedThrottle = 0.0;
      firing = false;
    }
  }

  // First-order thrust dynamics
  double targetThrottle = firing ? commandedThrottle : 0.0;

  if (timeConstant > 0.0)
  {
    double alpha = 1.0 - std::exp(-deltaTime / timeConstant);
    actualThrottle = actualThrottle + alpha * (targetThrottle - actualThrottle);
  }
  else
  {
    actualThrottle = targetThrottle;
  }

  // Propellant consumption
  if (actualThrottle > 0.01 && propellantMass > 0.0)
  {
    double massFlow = getMassFlowRate();
    double consumed = massFlow * deltaTime;

    if (consumed > propellantMass)
    {
      consumed = propellantMass;
    }

    propellantMass -= consumed;
    propellantConsumed += consumed;

    // If out of propellant, stop firing
    if (propellantMass <= 0.0)
    {
      propellantMass = 0.0;
      actualThrottle = 0.0;
      firing = false;
    }
  }
}

glm::dvec3 Thruster::getForce() const
{
  if (!enabled || !hasPropellant())
  {
    return glm::dvec3(0.0);
  }

  double thrust = maxThrust * actualThrottle;
  return thrust * thrustDirection;
}

glm::dvec3 Thruster::getTorque() const
{
  if (!enabled || !hasPropellant())
  {
    return glm::dvec3(0.0);
  }

  // Torque from thrust applied at offset position
  // τ = r × F
  glm::dvec3 force = getForce();
  return glm::cross(mountPosition, force);
}

void Thruster::fire(double throttle)
{
  if (!enabled || !hasPropellant())
  {
    return;
  }

  commandedThrottle = std::clamp(throttle, 0.0, 1.0);
  firing = true;
  pulseTimeRemaining = 0.0;  // Cancel any pulse
}

void Thruster::pulse(double duration, double throttle)
{
  if (!enabled || !hasPropellant() || duration <= 0.0)
  {
    return;
  }

  // Check minimum impulse bit
  double impulse = maxThrust * throttle * duration;
  if (impulse < minimumImpulseBit)
  {
    // Adjust duration to meet MIB
    duration = minimumImpulseBit / (maxThrust * throttle);
  }

  commandedThrottle = std::clamp(throttle, 0.0, 1.0);
  firing = true;
  pulseTimeRemaining = duration;
}

double Thruster::getMassFlowRate() const
{
  if (!firing || actualThrottle < 0.01)
  {
    return 0.0;
  }

  // Mass flow rate from rocket equation
  // F = m_dot * Ve = m_dot * Isp * g0
  // m_dot = F / (Isp * g0)

  double thrust = maxThrust * actualThrottle;
  return thrust / (specificImpulse * g0);
}

double Thruster::getPowerConsumption() const
{
  if (!enabled || actualThrottle < 0.01)
  {
    return 0.0;
  }

  double thrust = maxThrust * actualThrottle;
  return powerPerThrust * thrust;
}
