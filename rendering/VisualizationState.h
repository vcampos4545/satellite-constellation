#ifndef VISUALIZATION_STATE_H
#define VISUALIZATION_STATE_H

#include <memory>
#include <unordered_map>

class Satellite;
class CelestialBody;

/**
 * Per-satellite visualization settings
 * Keeps rendering state separate from simulation logic
 */
struct SatelliteVisualization
{
  bool showOrbitPath = false;
  bool showFootprint = false;
  bool showAttitudeVector = false;
};

/**
 * Selected object wrapper - can be either a CelestialBody or Satellite
 */
struct SelectedObject
{
  enum class Type
  {
    None,
    CelestialBody,
    Satellite
  };

  Type type = Type::None;
  void *object = nullptr; // Raw pointer to either CelestialBody* or Satellite*

  SelectedObject() = default;

  SelectedObject(std::shared_ptr<CelestialBody> body)
      : type(Type::CelestialBody), object(body.get()) {}

  SelectedObject(std::shared_ptr<Satellite> sat)
      : type(Type::Satellite), object(sat.get()) {}

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

  Satellite *asSatellite() const
  {
    return (type == Type::Satellite) ? static_cast<Satellite *>(object) : nullptr;
  }
};

/**
 * Manages all visualization state separate from simulation
 */
class VisualizationState
{
public:
  // Global visualization toggles
  bool showAllOrbitPaths = true;
  bool showAllAttitudeVectors = true;

  // Per-satellite visualization settings
  std::unordered_map<const Satellite *, SatelliteVisualization> satelliteViz;

  // Get or create visualization state for a satellite
  SatelliteVisualization &getOrCreate(const Satellite *sat)
  {
    return satelliteViz[sat];
  }

  // Check if we should draw orbit for this satellite
  bool shouldDrawOrbit(const Satellite *sat) const
  {
    if (!showAllOrbitPaths)
      return false;
    auto it = satelliteViz.find(sat);
    return it != satelliteViz.end() ? it->second.showOrbitPath : false;
  }

  // Check if we should draw footprint for this satellite
  bool shouldDrawFootprint(const Satellite *sat) const
  {
    auto it = satelliteViz.find(sat);
    return it != satelliteViz.end() ? it->second.showFootprint : false;
  }

  // Check if we should draw attitude vector for this satellite
  bool shouldDrawAttitudeVector(const Satellite *sat) const
  {
    if (!showAllAttitudeVectors)
      return false;
    auto it = satelliteViz.find(sat);
    return it != satelliteViz.end() ? it->second.showAttitudeVector : false;
  }
};

#endif // VISUALIZATION_STATE_H
