#pragma once

#include <optional>
#include <vector>

#include "njord_interfaces/msg/operator_command.hpp"
#include "njord_interfaces/msg/operator_response.hpp"

namespace critical_link
{
std::vector<uint8_t> encode_operator_command(const njord_interfaces::msg::OperatorCommand & message);
std::optional<njord_interfaces::msg::OperatorCommand> decode_operator_command(const std::vector<uint8_t> & bytes);
std::vector<uint8_t> encode_operator_response(const njord_interfaces::msg::OperatorResponse & message);
std::optional<njord_interfaces::msg::OperatorResponse> decode_operator_response(const std::vector<uint8_t> & bytes);
}  // namespace critical_link
