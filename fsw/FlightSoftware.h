#ifndef FLIGHT_SOFTWARE_H
#define FLIGHT_SOFTWARE_H

#include "FlightSoftwareModule.h"
#include "SpacecraftEnvironment.h"
#include <vector>
#include <memory>
#include <string>

// Forward declaration
class Spacecraft;

/**
 * FlightSoftware
 *
 * Manages all flight software modules for a single spacecraft.
 *
 * This class:
 * - Owns FSW modules (ADCS, power, station keeping, etc.)
 * - Updates all modules each timestep
 * - Provides access to modules by name or type
 *
 * Usage:
 *   auto fsw = std::make_unique<FlightSoftware>(spacecraft.get());
 *   auto adcs = fsw->addModule<ADCSModule>();
 *   auto power = fsw->addModule<PowerModule>();
 *   fsw->update(deltaTime, environment);
 *
 * Architecture:
 *   Universe -> owns FlightSoftware instances
 *   FlightSoftware -> owns FlightSoftwareModules
 *   FlightSoftwareModules -> read sensors, command actuators on Spacecraft
 */
class FlightSoftware
{
public:
  /**
   * Constructor
   * @param spacecraft Pointer to the spacecraft this FSW controls
   */
  FlightSoftware(Spacecraft *spacecraft);

  /**
   * Add a flight software module
   * @param args Constructor arguments for module type T
   * @return Pointer to created module
   */
  template <typename T, typename... Args>
  T *addModule(Args &&...args)
  {
    auto module = std::make_unique<T>(std::forward<Args>(args)...);
    T *ptr = module.get();
    modules.push_back(std::move(module));
    return ptr;
  }

  /**
   * Get module by name
   * @param name Module name
   * @return Pointer to module or nullptr if not found
   */
  FlightSoftwareModule *getModule(const std::string &name);

  /**
   * Get module by type
   * @return Pointer to module or nullptr if not found
   */
  template <typename T>
  T *getModule()
  {
    for (auto &module : modules)
    {
      T *typed = dynamic_cast<T *>(module.get());
      if (typed)
      {
        return typed;
      }
    }
    return nullptr;
  }

  /**
   * Update all flight software modules
   * @param deltaTime Time step (seconds)
   * @param environment Universe environment data
   */
  void update(double deltaTime, const SpacecraftEnvironment &environment);

  /**
   * Get spacecraft being controlled
   */
  Spacecraft *getSpacecraft() { return spacecraft; }
  const Spacecraft *getSpacecraft() const { return spacecraft; }

  /**
   * Get number of modules
   */
  size_t getModuleCount() const { return modules.size(); }

private:
  Spacecraft *spacecraft; // Spacecraft being controlled (not owned)
  std::vector<std::unique_ptr<FlightSoftwareModule>> modules;
};

#endif // FLIGHT_SOFTWARE_H
