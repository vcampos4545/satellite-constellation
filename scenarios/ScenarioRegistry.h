#ifndef SCENARIO_REGISTRY_H
#define SCENARIO_REGISTRY_H

#include "Scenario.h"
#include <map>
#include <vector>
#include <memory>
#include <functional>
#include <string>

/**
 * Scenario Registry
 *
 * Central registry for all available scenarios.
 * Supports auto-registration via REGISTER_SCENARIO macro.
 *
 * Usage:
 *   // In scenario .cpp file:
 *   REGISTER_SCENARIO(SSOTestScenario);
 *
 *   // At runtime:
 *   auto scenario = ScenarioRegistry::instance().createScenario("sso-test");
 *   scenario->setup(universe);
 */
class ScenarioRegistry
{
public:
  /**
   * Get singleton instance
   */
  static ScenarioRegistry &instance()
  {
    static ScenarioRegistry registry;
    return registry;
  }

  /**
   * Register a scenario factory
   * @param name Scenario name (from getName())
   * @param factory Function that creates a new scenario instance
   */
  void registerScenario(const std::string &name,
                        std::function<std::unique_ptr<Scenario>()> factory)
  {
    scenarios[name] = factory;
  }

  /**
   * Create a scenario by name
   * @param name Scenario name
   * @return Unique pointer to scenario, or nullptr if not found
   */
  std::unique_ptr<Scenario> createScenario(const std::string &name)
  {
    auto it = scenarios.find(name);
    if (it != scenarios.end())
    {
      return it->second();
    }
    return nullptr;
  }

  /**
   * Get list of all registered scenario names
   * @return Vector of scenario names
   */
  std::vector<std::string> listScenarios() const
  {
    std::vector<std::string> names;
    names.reserve(scenarios.size());
    for (const auto &pair : scenarios)
    {
      names.push_back(pair.first);
    }
    return names;
  }

  /**
   * Check if scenario exists
   * @param name Scenario name
   * @return True if scenario is registered
   */
  bool hasScenario(const std::string &name) const
  {
    return scenarios.find(name) != scenarios.end();
  }

  /**
   * Get number of registered scenarios
   */
  size_t count() const
  {
    return scenarios.size();
  }

private:
  ScenarioRegistry() = default;
  ScenarioRegistry(const ScenarioRegistry &) = delete;
  ScenarioRegistry &operator=(const ScenarioRegistry &) = delete;

  std::map<std::string, std::function<std::unique_ptr<Scenario>()>> scenarios;
};

/**
 * Auto-registration macro
 *
 * Place this in the .cpp file of your scenario to automatically register it.
 *
 * Example:
 *   // In SSOTestScenario.cpp
 *   REGISTER_SCENARIO(SSOTestScenario);
 */
#define REGISTER_SCENARIO(ClassName)                                     \
  namespace                                                              \
  {                                                                      \
    struct ClassName##Registrar                                          \
    {                                                                    \
      ClassName##Registrar()                                             \
      {                                                                  \
        auto instance = std::make_unique<ClassName>();                   \
        std::string name = instance->getName();                          \
        ScenarioRegistry::instance().registerScenario(                   \
            name,                                                        \
            []()                                                         \
            { return std::make_unique<ClassName>(); });                  \
      }                                                                  \
    };                                                                   \
    static ClassName##Registrar ClassName##_registrar;                   \
  }

#endif // SCENARIO_REGISTRY_H
