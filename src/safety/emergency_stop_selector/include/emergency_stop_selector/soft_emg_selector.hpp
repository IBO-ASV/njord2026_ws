#ifndef SIMPLE_MANUAL__SOFT_EMG_SELECTOR_HPP_
#define SIMPLE_MANUAL__SOFT_EMG_SELECTOR_HPP_

#include "sensor_msgs/msg/joy.hpp"

namespace emergency_stop_selector
{

bool sbus_soft_emg_selected(const sensor_msgs::msg::Joy & joy);

}  // namespace emergency_stop_selector

#endif  // SIMPLE_MANUAL__SOFT_EMG_SELECTOR_HPP_
