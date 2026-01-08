#ifndef PASSIVE_FSW_H
#define PASSIVE_FSW_H

#include "FlightSoftwareTask.h"

/**
 * Passive Flight Software
 *
 * Minimal FSW for simple satellites (e.g., cubesats, technology demonstrators).
 * Only performs:
 * - Basic detumbling (if angular velocity is too high)
 * - Power management (solar charging)
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
  PassiveFSW() = default;
  virtual ~PassiveFSW() = default;

  void execute(Satellite *satellite, double deltaTime) override;
  std::string getName() const override { return "PassiveFSW"; }

private:
  double detumbleThreshold = 0.1; // rad/s - start detumbling above this rate
};

#endif // PASSIVE_FSW_H
