#ifndef STANDARD_FSW_H
#define STANDARD_FSW_H

#include "FlightSoftwareTask.h"

/**
 * Standard Flight Software
 *
 * Implements typical satellite autonomous operations:
 * - ADCS (Attitude Determination and Control System)
 * - Power management
 * - Station keeping (if enabled)
 *
 * This is the default FSW used by most satellites in the simulation.
 * It replicates the behavior of the original hardcoded runFlightSoftware().
 */
class StandardFSW : public FlightSoftwareTask
{
public:
  StandardFSW() = default;
  virtual ~StandardFSW() = default;

  void execute(Satellite *satellite, double deltaTime) override;
  std::string getName() const override { return "StandardFSW"; }

private:
  // FSW state (if needed for multi-frame operations)
  double timeSinceLastCheck = 0.0;
};

#endif // STANDARD_FSW_H
