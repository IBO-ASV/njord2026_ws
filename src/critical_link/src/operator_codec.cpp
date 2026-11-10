#include "critical_link/operator_codec.hpp"

#include <limits>

namespace critical_link
{
namespace
{
void put_u8(std::vector<uint8_t> & out, uint8_t value) { out.push_back(value); }
void put_u16(std::vector<uint8_t> & out, uint16_t value) { out.push_back(value >> 8U); out.push_back(value); }
void put_u64(std::vector<uint8_t> & out, uint64_t value) { for (int i = 7; i >= 0; --i) out.push_back(value >> (i * 8)); }
bool take_u8(const std::vector<uint8_t> & in, size_t & pos, uint8_t & value) { return pos < in.size() && ((value = in[pos++]), true); }
bool take_u16(const std::vector<uint8_t> & in, size_t & pos, uint16_t & value) { if (pos + 2 > in.size()) return false; value = (uint16_t(in[pos]) << 8U) | in[pos + 1]; pos += 2; return true; }
bool take_u64(const std::vector<uint8_t> & in, size_t & pos, uint64_t & value) { if (pos + 8 > in.size()) return false; value = 0; for (int i = 0; i < 8; ++i) value = (value << 8U) | in[pos++]; return true; }
bool take_string(const std::vector<uint8_t> & in, size_t & pos, std::string & value) { uint16_t size; if (!take_u16(in, pos, size) || pos + size > in.size()) return false; value.assign(reinterpret_cast<const char *>(in.data() + pos), size); pos += size; return true; }
void put_string(std::vector<uint8_t> & out, const std::string & value) { if (value.size() > std::numeric_limits<uint16_t>::max()) throw std::length_error("operator string too long"); put_u16(out, value.size()); out.insert(out.end(), value.begin(), value.end()); }
}

std::vector<uint8_t> encode_operator_command(const njord_interfaces::msg::OperatorCommand & m) { std::vector<uint8_t> out; put_u64(out, m.request_id); put_u8(out, m.command); put_u8(out, m.target); put_u8(out, m.requested_mode); put_string(out, m.task_id); return out; }
std::optional<njord_interfaces::msg::OperatorCommand> decode_operator_command(const std::vector<uint8_t> & in) { njord_interfaces::msg::OperatorCommand m; size_t p = 0; if (!take_u64(in,p,m.request_id) || !take_u8(in,p,m.command) || !take_u8(in,p,m.target) || !take_u8(in,p,m.requested_mode) || !take_string(in,p,m.task_id) || p != in.size()) return {}; return m; }
std::vector<uint8_t> encode_operator_response(const njord_interfaces::msg::OperatorResponse & m) { std::vector<uint8_t> out; put_u64(out,m.request_id); put_u8(out,m.result_code); put_string(out,m.message); put_u8(out,m.mission_state); put_u8(out,m.requested_mode); put_u8(out,m.effective_mode); put_string(out,m.task_id); put_string(out,m.active_nav2_profile); put_u8(out,m.runtime_state); return out; }
std::optional<njord_interfaces::msg::OperatorResponse> decode_operator_response(const std::vector<uint8_t> & in) { njord_interfaces::msg::OperatorResponse m; size_t p=0; if (!take_u64(in,p,m.request_id) || !take_u8(in,p,m.result_code) || !take_string(in,p,m.message) || !take_u8(in,p,m.mission_state) || !take_u8(in,p,m.requested_mode) || !take_u8(in,p,m.effective_mode) || !take_string(in,p,m.task_id) || !take_string(in,p,m.active_nav2_profile) || !take_u8(in,p,m.runtime_state) || p != in.size()) return {}; return m; }
}  // namespace critical_link
