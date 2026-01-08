#ifndef PASSIVE_FSW_H
#define PASSIVE_FSW_H

#include "FlightSoftwareTask.h"
#include "ADCSController.h"
#include "../power/PowerManager.h"

/**
 * Passive Flight Software
 *
 * Minimal FSW for simple satellites (e.g., cubesats, technology demonstrators).
 * Only performs:
 * - Power management (solar charging, battery monitoring)
 * - Basic detumbling (if angular velocity is too high)
 *
 * Does NOT perform:
 * - Active attitude pointing
 * - Station keeping
 * - Complex autonomous operations
 *
 * Use this for passive satellites that just need to stay alive and charged.
 */
class PassiveFSW : public FlightSoftwareTask
{
public:
  PassiveFSW() : detumbleThreshold(0.1) {}
  virtual ~PassiveFSW() = default;

  void execute(Satellite *satellite, double deltaTime) override;
  std::string getName() const override { return "PassiveFSW"; }
  void reset() override
  {
    adcsController = ADCSController();
    powerManager = PowerManager();
  }

private:
  double detumbleThreshold; // rad/s - start detumbling above this rate

  // FSW Controllers
  ADCSController adcsController;
  PowerManager powerManager;
};

#endif // PASSIVE_FSW_H
