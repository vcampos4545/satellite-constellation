#include "Spacecraft.h"
#include "Component.h"
#include <algorithm>

Spacecraft::Spacecraft()
    : position(0.0, 0.0, 0.0),
      velocity(0.0, 0.0, 0.0),
      quaternion(1.0, 0.0, 0.0, 0.0), // Identity quaternion
      angularVelocity(0.0, 0.0, 0.0),
      mass(100.0), // Default 100 kg
      inertiaMatrix(glm::dmat3(1.0)) // Identity matrix
{
}

Spacecraft::Spacecraft(const glm::dvec3 &pos, const glm::dvec3 &vel)
    : position(pos),
      velocity(vel),
      quaternion(1.0, 0.0, 0.0, 0.0), // Identity quaternion
      angularVelocity(0.0, 0.0, 0.0),
      mass(100.0), // Default 100 kg
      inertiaMatrix(glm::dmat3(1.0)) // Identity matrix
{
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

void Spacecraft::update(double deltaTime)
{
  // Update all components first
  for (auto &comp : components)
  {
    if (comp->enabled)
    {
      comp->update(deltaTime);
    }
  }

  // Physics integration would go here in a full implementation
  // For now, this is a placeholder for the basic structure

  // Example of what physics update would look like:
  // 1. Aggregate forces and torques from actuators
  // glm::dvec3 totalForce = aggregateForces();
  // glm::dvec3 totalTorque = aggregateTorques();
  //
  // 2. Add external forces (gravity, drag, SRP, etc.)
  // totalForce += calculateGravity();
  // totalForce += calculateDrag();
  //
  // 3. Integrate translational dynamics
  // glm::dvec3 acceleration = totalForce / mass;
  // velocity += acceleration * deltaTime;
  // position += velocity * deltaTime;
  //
  // 4. Integrate rotational dynamics (Euler's equation)
  // glm::dvec3 angularAccel = glm::inverse(inertiaMatrix) *
  //                           (totalTorque - glm::cross(angularVelocity, inertiaMatrix * angularVelocity));
  // angularVelocity += angularAccel * deltaTime;
  //
  // 5. Update quaternion from angular velocity
  // glm::dquat omegaQuat(0.0, angularVelocity.x, angularVelocity.y, angularVelocity.z);
  // glm::dquat qdot = 0.5 * omegaQuat * quaternion;
  // quaternion += qdot * deltaTime;
  // quaternion = glm::normalize(quaternion);
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
