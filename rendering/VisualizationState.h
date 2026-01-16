#ifndef VISUALIZATION_STATE_H
#define VISUALIZATION_STATE_H

#include <memory>
#include <unordered_map>

class Spacecraft;
class CelestialBody;

/**
 * Per-satellite visualization settings
 * Keeps rendering state separate from simulation logic
 */
struct SpacecraftVisualizationState
{
  std::vector<glm::dvec3> orbitPath = {};
  std::vector<glm::dvec3> predictedOrbitPath = {};
  bool showOrbitPath = false;
  bool showPredictedOrbitPath = false;
  bool showAttitudeVector = false;
  bool showVelocityVector = false;
  bool showAxes = false;
};

/**
 * Selected object wrapper - can be either a CelestialBody or Spacecraft
 */
struct SelectedObject
{
  enum class Type
  {
    None,
    CelestialBody,
    Spacecraft
  };

  Type type = Type::None;
  void *object = nullptr; // Raw pointer to either CelestialBody* or Spacecraft*

  SelectedObject() = default;

  SelectedObject(std::shared_ptr<CelestialBody> body)
      : type(Type::CelestialBody), object(body.get()) {}

  SelectedObject(std::shared_ptr<Spacecraft> sat)
      : type(Type::Spacecraft), object(sat.get()) {}

  bool isValid() const { return type != Type::None && object != nullptr; }
  void clear()
  {
    type = Type::None;
    object = nullptr;
  }

  CelestialBody *asCelestialBody() const
  {
    return (type == Type::CelestialBody) ? static_cast<CelestialBody *>(object) : nullptr;
  }

  Spacecraft *asSpacecraft() const
  {
    return (type == Type::Spacecraft) ? static_cast<Spacecraft *>(object) : nullptr;
  }
};

/**
 * Manages all visualization state separate from simulation
 */
class VisualizationState
{
public:
  // Per-spacecraft visualization settings
  std::unordered_map<const Spacecraft *, SpacecraftVisualizationState> spacecraftViz;

  // Get or create visualization state for a spacecraft
  SpacecraftVisualizationState &getOrCreate(const Spacecraft *sc)
  {
    return spacecraftViz[sc];
  }
};

#endif // VISUALIZATION_STATE_H
