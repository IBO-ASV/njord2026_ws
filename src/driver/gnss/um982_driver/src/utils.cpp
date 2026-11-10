#include "um982_driver/utils.hpp"
#include <algorithm>
#include <cctype>
#include <iostream>

namespace um982_driver
{
namespace utils
{

std::vector<std::string> split(const std::string & s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    
    // NMEAデータでは ",," のように空フィールドが続くことがあるため、
    // 単純な空白スキップではなく、明示的にデリミタで切る必要がある
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    
    // 末尾がデリミタで終わる場合のケア（必要に応じて）
    if (!s.empty() && s.back() == delimiter) {
        tokens.push_back("");
    }

    return tokens;
}

bool validate_checksum(const std::string & sentence)
{
    // $ または # で始まり、* を含む必要がある
    size_t start_idx = sentence.find_first_of("$#");
    size_t star_idx = sentence.find('*');

    if (start_idx == std::string::npos || star_idx == std::string::npos || star_idx < start_idx) {
        return false;
    }

    // チェックサム計算対象の範囲を取得 ($の次から*の前まで)
    std::string content = sentence.substr(start_idx + 1, star_idx - start_idx - 1);
    
    // 提供されたチェックサム文字列
    std::string checksum_str = sentence.substr(star_idx + 1);
    // 改行コードなどを除去
    checksum_str.erase(std::remove(checksum_str.begin(), checksum_str.end(), '\r'), checksum_str.end());
    checksum_str.erase(std::remove(checksum_str.begin(), checksum_str.end(), '\n'), checksum_str.end());

    if (checksum_str.size() != 2 ||
        !std::all_of(checksum_str.begin(), checksum_str.end(), [](unsigned char character) {
            return std::isxdigit(character) != 0;
        }))
    {
        return false;
    }

    int provided_checksum = 0;
    try {
        std::size_t parsed = 0;
        provided_checksum = std::stoi(checksum_str, &parsed, 16);
        if (parsed != checksum_str.size()) {
            return false;
        }
    } catch (...) {
        return false;
    }

    // XOR計算
    int calculated_checksum = 0;
    for (char c : content) {
        calculated_checksum ^= c;
    }

    return calculated_checksum == provided_checksum;
}

bool parse_finite_double(const std::string & value, double & result) noexcept
{
    if (value.empty()) {
        return false;
    }

    try {
        std::size_t parsed = 0;
        const double candidate = std::stod(value, &parsed);
        if (parsed != value.size() || !std::isfinite(candidate)) {
            return false;
        }
        result = candidate;
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_nmea_utc_seconds(const std::string & value, double & result) noexcept
{
    double raw = 0.0;
    if (!parse_finite_double(value, raw) || raw < 0.0) {
        return false;
    }
    const int hours = static_cast<int>(raw / 10000.0);
    const int minutes = static_cast<int>(raw / 100.0) % 100;
    const double seconds = raw - hours * 10000.0 - minutes * 100.0;
    if (hours >= 24 || minutes >= 60 || seconds >= 60.0) {
        return false;
    }
    result = hours * 3600.0 + minutes * 60.0 + seconds;
    return true;
}

bool parse_int(const std::string & value, int & result) noexcept
{
    if (value.empty()) {
        return false;
    }

    try {
        std::size_t parsed = 0;
        const int candidate = std::stoi(value, &parsed, 10);
        if (parsed != value.size()) {
            return false;
        }
        result = candidate;
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_gst_standard_deviations(
    const std::string & sentence, double & latitude, double & longitude,
    double & altitude) noexcept
{
    if (!validate_checksum(sentence)) {
        return false;
    }
    const auto parts = split(sentence.substr(0, sentence.find('*')), ',');
    if (parts.size() < 9 ||
        (parts[0] != "$GPGST" && parts[0] != "$GNGST") ||
        !parse_finite_double(parts[6], latitude) ||
        !parse_finite_double(parts[7], longitude) ||
        !parse_finite_double(parts[8], altitude))
    {
        return false;
    }
    return latitude > 0.0 && longitude > 0.0 && altitude > 0.0;
}

double normalize_angle_rad(double angle_rad) noexcept
{
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

bool convert_nmea_to_latlon(
    const std::string & value, const std::string & direction, double & result) noexcept
{
    if (direction != "N" && direction != "S" && direction != "E" && direction != "W") {
        return false;
    }

    double raw_val = 0.0;
    if (!parse_finite_double(value, raw_val) || raw_val < 0.0) {
        return false;
    }

    const int degrees = static_cast<int>(raw_val / 100.0);
    const double minutes = raw_val - (degrees * 100.0);
    const int max_degrees = (direction == "N" || direction == "S") ? 90 : 180;
    if (minutes < 0.0 || minutes >= 60.0 || degrees > max_degrees ||
        (degrees == max_degrees && minutes > 0.0))
    {
        return false;
    }

    double decimal = degrees + (minutes / 60.0);
    if (direction == "S" || direction == "W") {
        decimal = -decimal;
    }
    if (!std::isfinite(decimal)) {
        return false;
    }
    result = decimal;
    return true;
}

uint32_t calculate_unicore_crc32(const uint8_t * data, size_t len)
{
    uint32_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320UL : (crc >> 1);
        }
    }
    return crc;
}

std::string base64_encode(const std::string & input)
{
    static const std::string base64_chars = 
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789+/";

    std::string ret;
    int i = 0;
    int j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];

    const char* bytes_to_encode = input.c_str();
    size_t len = input.size();

    while (len--) {
        char_array_3[i++] = *(bytes_to_encode++);
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for(i = 0; (i <4) ; i++)
                ret += base64_chars[char_array_4[i]];
            i = 0;
        }
    }

    if (i)
    {
        for(j = i; j < 3; j++)
            char_array_3[j] = '\0';

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (j = 0; (j < i + 1); j++)
            ret += base64_chars[char_array_4[j]];

        while((i++ < 3))
            ret += '=';
    }

    return ret;
}

} // namespace utils
} // namespace um982_driver
