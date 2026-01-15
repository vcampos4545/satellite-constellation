/*
Satellite Constellation Simulation

This simulation displays the Earth and a constellation of satellites.
Supports different scenarios configured via command-line arguments.

Usage:
  ./simulation                    Run default scenario with GUI
  ./simulation -scenario=sso-test Run specific scenario
  ./simulation -list              List available scenarios
  ./simulation -headless          Run without GUI

*/

#include "Simulation.h"
#include "Universe.h"
#include "scenarios/ScenarioRegistry.h"
#include "scenarios/Scenario.h"
#include <iostream>
#include <cstdio>
#include <string>

/**
 * Parse command-line arguments
 */
void parseArguments(int argc, char *argv[],
                    std::string &scenarioName,
                    bool &headless,
                    bool &listScenarios)
{
  scenarioName = "sso-test"; // Default scenario
  headless = false;
  listScenarios = false;

  for (int i = 1; i < argc; i++)
  {
    std::string arg = argv[i];

    if (arg.find("-scenario=") == 0)
    {
      // Extract scenario name after '='
      scenarioName = arg.substr(10);
    }
    else if (arg == "-headless" || arg == "--headless")
    {
      headless = true;
    }
    else if (arg == "-list" || arg == "--list")
    {
      listScenarios = true;
    }
    else if (arg == "-h" || arg == "--help")
    {
      printf("Satellite Constellation Simulation\n\n");
      printf("Usage:\n");
      printf("  ./simulation                    Run default scenario with GUI\n");
      printf("  ./simulation -scenario=NAME     Run specific scenario\n");
      printf("  ./simulation -list              List available scenarios\n");
      printf("  ./simulation -headless          Run without GUI\n");
      printf("  ./simulation -h, --help         Show this help message\n\n");
      printf("Examples:\n");
      printf("  ./simulation -scenario=sso-test\n");
      printf("  ./simulation -scenario=sso-test -headless\n");
      exit(0);
    }
    else
    {
      printf("Warning: Unknown argument '%s' (use -h for help)\n", arg.c_str());
    }
  }
}

/**
 * List all available scenarios
 */
void listAvailableScenarios()
{
  printf("\n\033[36m=== Available Scenarios ===\033[0m\n\n");

  auto scenarios = ScenarioRegistry::instance().listScenarios();

  if (scenarios.empty())
  {
    printf("  No scenarios registered.\n");
    printf("  (This likely means no scenario .cpp files were linked)\n\n");
    return;
  }

  for (const auto &name : scenarios)
  {
    // Create temporary instance to get description
    auto scenario = ScenarioRegistry::instance().createScenario(name);
    if (scenario)
    {
      printf("  \033[32m%s\033[0m\n", name.c_str());
      printf("    %s\n", scenario->getDescription().c_str());
      printf("    Duration: %.1f minutes | Time warp: %.0fx | GUI: %s\n",
             scenario->getDuration() / 60.0,
             scenario->getTimeWarp(),
             scenario->requiresGUI() ? "yes" : "no");
      printf("\n");
    }
  }

  printf("Usage: ./simulation -scenario=NAME\n\n");
}

int main(int argc, char *argv[])
{
  try
  {
    // Parse command-line arguments
    std::string scenarioName;
    bool headless;
    bool listScenarios;
    parseArguments(argc, argv, scenarioName, headless, listScenarios);

    // Handle -list flag
    if (listScenarios)
    {
      listAvailableScenarios();
      return 0;
    }

    // Create scenario
    auto scenario = ScenarioRegistry::instance().createScenario(scenarioName);
    if (!scenario)
    {
      printf("\033[31mError: Scenario '%s' not found\033[0m\n", scenarioName.c_str());
      printf("Use -list to see available scenarios\n");
      return 1;
    }

    printf("\n\033[36m=== Satellite Constellation Simulation ===\033[0m\n");
    printf("\033[32mScenario:\033[0m %s\n", scenario->getName().c_str());
    printf("\033[32mDescription:\033[0m %s\n\n", scenario->getDescription().c_str());

    // Determine if GUI should be used
    bool useGUI = !headless && scenario->requiresGUI();

    // Create simulation (Simulation constructor expects headless flag, not useGUI)
    Simulation sim(!useGUI);

    // Setup scenario
    printf("\033[36m[Main] Setting up scenario...\033[0m\n");
    scenario->setup(sim.getUniverse());

    // Run scenario
    printf("\033[36m[Main] Starting simulation...\033[0m\n\n");
    scenario->run(sim);

    // Cleanup
    printf("\n\033[36m[Main] Cleaning up...\033[0m\n");
    scenario->teardown();

    printf("\033[32m=== Simulation Complete ===\033[0m\n\n");

    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << "\033[31mError: " << e.what() << "\033[0m" << std::endl;
    return -1;
  }
}

