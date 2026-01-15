#ifndef STANDARD_FSW_H
#define STANDARD_FSW_H

#include "FlightSoftwareTask.h"
#include "ADCSController.h"

/**
 * Standard Flight Software
 *
 * Implements typical satellite autonomous operations:
 * - Power management (solar charging, battery monitoring)
 * - ADCS (Attitude Determination and Control System)
 * - Station keeping (if enabled - orbital maintenance)
 * - Housekeeping and health monitoring
 *
 * This is the default FSW used by most satellites in the simulation.
 * Uses modular controllers for each subsystem (PowerManager, ADCSController, StationKeepingController).
 */
class StandardFSW : public FlightSoftwareTask
{
public:
  StandardFSW() : timeSinceLastCheck(0.0) {}
  virtual ~StandardFSW() = default;

  void execute(Satellite *satellite, double deltaTime) override;
  std::string getName() const override { return "StandardFSW"; }
  void reset() override
  {
    timeSinceLastCheck = 0.0;
    adcsController = ADCSController();
  }

private:
  // FSW state (if needed for multi-frame operations)
  double timeSinceLastCheck;

  // FSW Controllers
  ADCSController adcsController;
};

#endif // STANDARD_FSW_H
