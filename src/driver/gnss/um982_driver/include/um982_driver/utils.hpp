#ifndef UM982_DRIVER_UTILS_HPP_
#define UM982_DRIVER_UTILS_HPP_

#include <string>
#include <vector>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <cstdint>
#include <cstring>

namespace um982_driver
{
namespace utils
{

/**
 * @brief 文字列を指定したデリミタで分割する
 * NMEAセンテンスの解析に使用します。空の要素も空文字として保持します。
 */
std::vector<std::string> split(const std::string & s, char delimiter);

/**
 * @brief NMEAフォーマットのチェックサムを検証する
 * 例: "$GNGGA,025754.00,0,N,0,E,0,0,0.7,63.3224,M,-9.7848,M,00,0000*58" の *58 が正しいかチェックします。
 */
bool validate_checksum(const std::string & sentence);

/**
 * @brief NMEA形式 (ddmm.mmmm) を 十進度数 (dd.dddd) に変換する
 * * @param value NMEA形式の数値文字列 (例: "3541.6053")
 * @param direction 方角文字 (N, S, E, W)
 * @param result 変換後の度数
 * @return 数値・方位・範囲がすべて正常なら true
 */
bool convert_nmea_to_latlon(
    const std::string & value, const std::string & direction, double & result) noexcept;

/**
 * @brief 文字列全体を有限なdoubleとして解析する
 */
bool parse_finite_double(const std::string & value, double & result) noexcept;

/**
 * @brief NMEA UTC time (hhmmss.ss) を一日の秒数に変換する
 */
bool parse_nmea_utc_seconds(const std::string & value, double & result) noexcept;

/**
 * @brief 文字列全体を10進intとして解析する
 */
bool parse_int(const std::string & value, int & result) noexcept;

bool parse_gst_standard_deviations(
    const std::string & sentence, double & latitude, double & longitude,
    double & altitude) noexcept;

/**
 * @brief Base64エンコード (NTRIP認証用)
 */
std::string base64_encode(const std::string & input);

/**
 * @brief 度数法から弧度法への変換
 */
inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

/**
 * @brief Wrap an angle to [-pi, pi].
 */
double normalize_angle_rad(double angle_rad) noexcept;

/**
 * @brief Unicoreバイナリログのリトルエンディアン整数/浮動小数点読み出し
 */
inline uint16_t read_u16_le(const uint8_t * p)
{
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

inline uint32_t read_u32_le(const uint8_t * p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

inline float read_f32_le(const uint8_t * p)
{
    uint32_t bits = read_u32_le(p);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

/**
 * @brief Unicoreバイナリログ用CRC32 (初期値0、終了XORなし、多項式0xEDB88320)
 * Unicore Reference Commands Manual Appendix 1 準拠
 */
uint32_t calculate_unicore_crc32(const uint8_t * data, size_t len);

} // namespace utils
} // namespace um982_driver

#endif // UM982_DRIVER_UTILS_HPP_
