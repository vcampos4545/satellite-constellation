#include "Spacecraft.h"
#include "Component.h"
#include "GravityModel.h"
#include "EnvironmentalModels.h"
#include "RK4Integrator.h"
#include "Constants.h"
#include <algorithm>

Spacecraft::Spacecraft()
    : position(0.0, 0.0, 0.0),
      velocity(0.0, 0.0, 0.0),
      quaternion(1.0, 0.0, 0.0, 0.0), // Identity quaternion
      angularVelocity(0.0, 0.0, 0.0),
      mass(260.0),                    // Default mass (kg)
      inertiaMatrix(glm::dmat3(0.0)), // Initialize to zero, then set diagonal
      dragCoefficient(2.2),           // Typical satellite drag coefficient
      crossSectionalArea(10.0),       // Default area (m²)
      reflectivity(1.3),              // Default reflectivity
      name(""),
      modelName("starlink")
{
  // Set realistic inertia for ~260 kg Starlink-like satellite
  // Approximate dimensions: 3.2m x 1.6m x 0.2m (flat panel shape)
  // Using I = (1/12) * m * (a² + b²) for rectangular prism
  // Ixx (roll):  about X-axis (long axis) - (1/12)*260*(1.6² + 0.2²) ≈ 56 kg·m²
  // Iyy (pitch): about Y-axis (short axis) - (1/12)*260*(3.2² + 0.2²) ≈ 223 kg·m²
  // Izz (yaw):   about Z-axis (thin axis)  - (1/12)*260*(3.2² + 1.6²) ≈ 277 kg·m²
  inertiaMatrix[0][0] = 56.0;   // Ixx (kg·m²)
  inertiaMatrix[1][1] = 223.0;  // Iyy (kg·m²)
  inertiaMatrix[2][2] = 277.0;  // Izz (kg·m²)
}

Spacecraft::Spacecraft(const glm::dvec3 &pos, const glm::dvec3 &vel)
    : position(pos),
      velocity(vel),
      quaternion(1.0, 0.0, 0.0, 0.0), // Identity quaternion
      angularVelocity(0.0, 0.0, 0.0),
      mass(260.0),                    // Default mass (kg)
      inertiaMatrix(glm::dmat3(0.0)), // Initialize to zero, then set diagonal
      dragCoefficient(2.2),           // Typical satellite drag coefficient
      crossSectionalArea(10.0),       // Default area (m²)
      reflectivity(1.3),              // Default reflectivity
      name(""),
      modelName("starlink")
{
  // Set realistic inertia (same as default constructor)
  inertiaMatrix[0][0] = 56.0;   // Ixx (kg·m²)
  inertiaMatrix[1][1] = 223.0;  // Iyy (kg·m²)
  inertiaMatrix[2][2] = 277.0;  // Izz (kg·m²)
}

Spacecraft::Spacecraft(const Orbit &orbit, const std::string &name)
    : quaternion(1.0, 0.0, 0.0, 0.0), // Identity quaternion
      angularVelocity(0.0, 0.0, 0.0),
      mass(260.0),                    // Default mass (kg)
      inertiaMatrix(glm::dmat3(0.0)), // Initialize to zero, then set diagonal
      dragCoefficient(2.2),           // Typical satellite drag coefficient
      crossSectionalArea(10.0),       // Default area (m²)
      reflectivity(1.3),              // Default reflectivity
      name(name),
      modelName("starlink")
{
  // Set realistic inertia (same as default constructor)
  inertiaMatrix[0][0] = 56.0;   // Ixx (kg·m²)
  inertiaMatrix[1][1] = 223.0;  // Iyy (kg·m²)
  inertiaMatrix[2][2] = 277.0;  // Izz (kg·m²)

  // Compute initial position and velocity from orbital elements
  orbit.toCartesian(position, velocity, G * EARTH_MASS);
}

Spacecraft::~Spacecraft()
{
  // Components are automatically cleaned up by unique_ptr
}

Component *Spacecraft::getComponent(const std::string &name)
{
  auto it = componentLookup.find(name);
  if (it != componentLookup.end())
  {
    return it->second;
  }
  return nullptr;
}

bool Spacecraft::removeComponent(const std::string &name)
{
  // Find component in lookup table
  auto it = componentLookup.find(name);
  if (it == componentLookup.end())
  {
    return false; // Not found
  }

  Component *compPtr = it->second;

  // Remove from lookup
  componentLookup.erase(it);

  // Remove from components vector
  components.erase(
      std::remove_if(components.begin(), components.end(),
                     [compPtr](const std::unique_ptr<Component> &comp)
                     {
                       return comp.get() == compPtr;
                     }),
      components.end());

  return true;
}

glm::dvec3 Spacecraft::calculateAcceleration(const glm::dvec3 &pos,
                                             const glm::dvec3 &vel,
                                             const glm::dvec3 &earthCenter,
                                             double earthMass,
                                             const glm::dvec3 &sunPos,
                                             const glm::dvec3 &moonPos) const
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

  return gravAccelEarth + gravAccelEarthJ + gravAccelMoon + gravAccelSun + dragAccel + srpAccel;
}

void Spacecraft::update(double deltaTime,
                        const glm::dvec3 &earthCenter,
                        double earthMass,
                        const glm::dvec3 &sunPosition,
                        const glm::dvec3 &moonPosition)
{
  // ========== ORBITAL DYNAMICS ==========
  // Integrate position and velocity using RK4
  auto accelFunc = [this, &earthCenter, earthMass, &sunPosition, &moonPosition](const glm::dvec3 &pos, const glm::dvec3 &vel) -> glm::dvec3
  {
    return calculateAcceleration(pos, vel, earthCenter, earthMass, sunPosition, moonPosition);
  };

  RK4Integrator::integratePositionVelocity(position, velocity, deltaTime, accelFunc);

  // ========== ROTATIONAL DYNAMICS ==========
  // Aggregate torques from all actuators (reaction wheels, magnetorquers, etc.)
  glm::dvec3 totalTorque = aggregateTorques();

  // Angular acceleration function: dω/dt = I⁻¹ · (τ - ω × (I·ω))
  auto angularAccelFunc = [this, totalTorque](const glm::dvec3 &omega) -> glm::dvec3
  {
    return calculateAngularAcceleration(totalTorque, omega);
  };

  // Quaternion derivative function: dq/dt = 0.5 * ω_quat * q
  // Where ω_quat = [0, ωx, ωy, ωz] (pure quaternion from angular velocity)
  auto quatDotFunc = [](const glm::dquat &q, const glm::dvec3 &omega) -> glm::dquat
  {
    glm::dquat omegaQuat(0.0, omega.x, omega.y, omega.z);
    return 0.5 * omegaQuat * q;
  };

  // Integrate both angular velocity and quaternion using coupled RK4
  // This is more accurate than separate integration because it properly
  // handles the coupling between ω and q at intermediate RK4 stages
  RK4Integrator::integrateAttitude(quaternion, angularVelocity, deltaTime,
                                   angularAccelFunc, quatDotFunc);

  // ========== COMPONENT UPDATES ==========
  // Update all components
  for (auto &comp : components)
  {
    if (comp->enabled)
    {
      comp->update(deltaTime);
    }
  }
}

glm::dvec3 Spacecraft::aggregateForces() const
{
  glm::dvec3 totalForce(0.0);

  // Iterate through all components and accumulate forces from actuators
  for (const auto &comp : components)
  {
    if (!comp->enabled)
      continue;

    // Try to cast to Actuator
    if (Actuator *actuator = dynamic_cast<Actuator *>(comp.get()))
    {
      totalForce += actuator->getForce();
    }
  }

  return totalForce;
}

glm::dvec3 Spacecraft::aggregateTorques() const
{
  glm::dvec3 totalTorque(0.0);

  // Iterate through all components and accumulate torques from actuators
  for (const auto &comp : components)
  {
    if (!comp->enabled)
      continue;

    // Try to cast to Actuator
    if (Actuator *actuator = dynamic_cast<Actuator *>(comp.get()))
    {
      totalTorque += actuator->getTorque();
    }
  }

  return totalTorque;
}

glm::dvec3 Spacecraft::calculateAngularAcceleration(const glm::dvec3 &torque,
                                                    const glm::dvec3 &omega) const
{
  // Euler's rotation equation: I·ω' = τ - ω × (I·ω)
  // Rearranging: ω' = I⁻¹ · (τ - ω × (I·ω))

  // Calculate I·ω (angular momentum)
  glm::dvec3 angularMomentum = inertiaMatrix * omega;

  // Calculate gyroscopic torque: ω × (I·ω)
  glm::dvec3 gyroscopicTorque = glm::cross(omega, angularMomentum);

  // Net torque: τ - ω × (I·ω)
  glm::dvec3 netTorque = torque - gyroscopicTorque;

  // Angular acceleration: I⁻¹ · netTorque
  glm::dvec3 angularAccel = glm::inverse(inertiaMatrix) * netTorque;

  return angularAccel;
}
