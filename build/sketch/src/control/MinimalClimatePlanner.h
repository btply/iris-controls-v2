#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/control/MinimalClimatePlanner.h"
#ifndef MINIMAL_CLIMATE_PLANNER_H
#define MINIMAL_CLIMATE_PLANNER_H

#include "../core/AppDataTypes.h"

class MinimalClimatePlanner {
 public:
  ClimateIntent computeIntent(const PlannerInput& input, unsigned long nowMs);

 private:
  bool windEmergencyLatched = false;
  bool rainLatched = false;
  unsigned long windAboveSinceMs = 0UL;
  unsigned long windBelowSinceMs = 0UL;
  unsigned long rainClearSinceMs = 0UL;
};

#endif  // MINIMAL_CLIMATE_PLANNER_H
