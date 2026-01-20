#include "FlightSoftware.h"
#include "Spacecraft.h"

FlightSoftware::FlightSoftware(Spacecraft *spacecraft)
    : spacecraft(spacecraft)
{
}

FlightSoftwareModule *FlightSoftware::getModule(const std::string &name)
{
  for (auto &module : modules)
  {
    if (module->getName() == name)
    {
      return module.get();
    }
  }
  return nullptr;
}

void FlightSoftware::update(double deltaTime, const SpacecraftEnvironment &environment)
{
  // Update all enabled modules
  for (auto &module : modules)
  {
    if (module->isEnabled())
    {
      module->update(deltaTime, *spacecraft, environment);
    }
  }
}
