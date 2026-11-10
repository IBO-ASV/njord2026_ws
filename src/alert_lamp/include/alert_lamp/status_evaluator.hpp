#pragma once

#include "alert_lamp/alert_lamp_state.hpp"

namespace alert_lamp
{

class StatusEvaluator
{
public:
  AlertState evaluate(const SystemStatus & status) const;
  bool hasCriticalFault(const SystemStatus & status) const;
  LampDisplay displayFor(
    AlertState state, float green_period, float yellow_period,
    float red_period, float duty_ratio, const SystemStatus & status) const;
};

}  // namespace alert_lamp
