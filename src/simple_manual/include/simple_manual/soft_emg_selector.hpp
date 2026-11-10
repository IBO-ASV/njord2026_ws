#ifndef SIMPLE_MANUAL__SOFT_EMG_SELECTOR_HPP_
#define SIMPLE_MANUAL__SOFT_EMG_SELECTOR_HPP_

#include "sensor_msgs/msg/joy.hpp"

namespace simple_manual
{

bool sbus_soft_emg_selected(const sensor_msgs::msg::Joy & joy);

}  // namespace simple_manual

#endif  // SIMPLE_MANUAL__SOFT_EMG_SELECTOR_HPP_
