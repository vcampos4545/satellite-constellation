#ifndef SPACECRAFT_ENVIRONMENT_H
#define SPACECRAFT_ENVIRONMENT_H

#include <glm/glm.hpp>
#include <vector>
#include <memory>

// Forward declarations
class GroundStation;
class Spacecraft;

/**
 * SpacecraftEnvironment
 *
 * Provides flight software with access to universe data needed for:
 * - Attitude determination and control (ground station pointing)
 * - Station keeping (relative positions of other spacecraft)
 * - Collision avoidance (nearby spacecraft)
 * - Power management (sun position)
 *
 * This interface decouples FSW from Universe implementation details.
 */
struct SpacecraftEnvironment
{
  // Celestial body positions (inertial frame)
  glm::dvec3 earthPosition;
  glm::dvec3 sunPosition;
  glm::dvec3 moonPosition;

  // Ground stations (for targeting)
  const std::vector<std::shared_ptr<GroundStation>> *groundStations;

  // Other spacecraft (for collision avoidance, station keeping)
  const std::vector<std::shared_ptr<Spacecraft>> *otherSpacecraft;

  // Spacecraft's own index in the constellation (to exclude self from otherSpacecraft)
  size_t selfIndex;

  SpacecraftEnvironment()
      : earthPosition(0.0),
        sunPosition(0.0),
        moonPosition(0.0),
        groundStations(nullptr),
        otherSpacecraft(nullptr),
        selfIndex(0)
  {
  }
};

#endif // SPACECRAFT_ENVIRONMENT_H
