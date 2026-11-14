#pragma once

#include <optional>
#include <vector>

#include "operator_interfaces/msg/operator_command.hpp"
#include "operator_interfaces/msg/operator_response.hpp"

namespace critical_link
{
std::vector<uint8_t> encode_operator_command(const operator_interfaces::msg::OperatorCommand & message);
std::optional<operator_interfaces::msg::OperatorCommand> decode_operator_command(const std::vector<uint8_t> & bytes);
std::vector<uint8_t> encode_operator_response(const operator_interfaces::msg::OperatorResponse & message);
std::optional<operator_interfaces::msg::OperatorResponse> decode_operator_response(const std::vector<uint8_t> & bytes);
}  // namespace critical_link
