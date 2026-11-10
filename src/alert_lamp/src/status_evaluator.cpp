#include "alert_lamp/status_evaluator.hpp"

namespace alert_lamp
{

bool StatusEvaluator::hasCriticalFault(const SystemStatus & s) const
{
  if (s.emergency_stop || s.critical_driver_failure || s.critical_diagnostics || s.state_unknown ||
    !s.driver_alive || s.mode == OperatingMode::UNKNOWN)
  {
    return true;
  }
  if (s.mode == OperatingMode::AUTO && (!s.high_level_alive || !s.autonomy_alive)) {
    return true;
  }
  return !s.ground_station_connected && !s.localization_stable;
}

AlertState StatusEvaluator::evaluate(const SystemStatus & s) const
{
  if (hasCriticalFault(s)) {return AlertState::CRITICAL_FAULT;}
  if (!s.ground_station_connected && s.driver_alive && s.high_level_alive && s.autonomy_alive &&
    s.localization_alive && s.localization_stable && !s.emergency_stop && !s.critical_diagnostics)
  {
    return AlertState::GROUND_COMMUNICATION_LOST;
  }
  if (s.mode == OperatingMode::AUTO && (!s.high_level_alive || !s.autonomy_alive ||
    !s.autonomy_ready || !s.rtk_fix || s.required_sensor_not_ready))
  {
    return AlertState::AUTONOMY_NOT_READY;
  }
  if (s.mode == OperatingMode::AUTO && s.driver_alive && s.high_level_alive &&
    s.autonomy_alive && s.autonomy_ready && !s.critical_diagnostics)
  {
    return AlertState::AUTO_NORMAL;
  }
  if (s.mode == OperatingMode::MANUAL && s.driver_alive && s.manual_control_alive &&
    !s.emergency_stop && !s.critical_diagnostics)
  {
    return AlertState::MANUAL_NORMAL;
  }
  return AlertState::INITIALIZING;
}

LampDisplay StatusEvaluator::displayFor(
  AlertState state, float green_period, float /*yellow_period*/,
  float red_period, float duty_ratio, const SystemStatus & status) const
{
  switch (state) {
    case AlertState::MANUAL_NORMAL:
      return {LampColor::YELLOW, LampPattern::SOLID, 0.0F, duty_ratio, "manual mode normal"};
    case AlertState::AUTO_NORMAL:
      return {LampColor::GREEN, LampPattern::SOLID, 0.0F, duty_ratio,
        "automatic mode normal"};
    case AlertState::AUTONOMY_NOT_READY:
      return {LampColor::GREEN_YELLOW, LampPattern::BLINK, green_period, duty_ratio,
        "autonomy not ready"};
    case AlertState::GROUND_COMMUNICATION_LOST:
      if (status.mode == OperatingMode::MANUAL) {
        return {LampColor::YELLOW_RED, LampPattern::SOLID, 0.0F, duty_ratio,
          "ground station communication lost in manual mode"};
      }
      return {LampColor::GREEN_RED, LampPattern::SOLID, 0.0F, duty_ratio,
        "ground station communication lost in automatic mode"};
    case AlertState::CRITICAL_FAULT:
      return {LampColor::RED, LampPattern::BLINK, red_period, duty_ratio,
        "critical fault or unknown state"};
    case AlertState::INITIALIZING:
    default:
      return {LampColor::RED, LampPattern::BLINK, red_period, duty_ratio, "initializing"};
  }
}

}  // namespace alert_lamp
