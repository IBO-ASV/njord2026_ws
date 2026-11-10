#include "um982_driver/node.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include "um982_driver/tf2_geometry_msgs_include.hpp"

namespace
{
constexpr uint8_t kBinarySync[3] = {0xAA, 0x44, 0xB5};
constexpr std::size_t kBinaryHeaderLen = 24;
constexpr uint16_t kUniheadingMessageId = 972;
}  // namespace

namespace um982_driver
{

void UM982Driver::process_gnss_buffer()
{
    while (!gnss_parse_buf_.empty()) {
        if (gnss_parse_buf_.size() >= 3 &&
            gnss_parse_buf_[0] == kBinarySync[0] &&
            gnss_parse_buf_[1] == kBinarySync[1] &&
            gnss_parse_buf_[2] == kBinarySync[2])
        {
            if (gnss_parse_buf_.size() < kBinaryHeaderLen) break;

            const uint16_t msg_id = utils::read_u16_le(&gnss_parse_buf_[4]);
            const uint16_t msg_len = utils::read_u16_le(&gnss_parse_buf_[6]);
            const std::size_t total_len = kBinaryHeaderLen + msg_len + 4;
            if (total_len > gnss_parse_buf_.size()) {
                if (total_len > 8192) {
                    gnss_parse_buf_.erase(gnss_parse_buf_.begin(), gnss_parse_buf_.begin() + 3);
                    continue;
                }
                break;
            }

            const bool crc_ok = utils::calculate_unicore_crc32(gnss_parse_buf_.data(), total_len - 4) ==
                utils::read_u32_le(&gnss_parse_buf_[total_len - 4]);
            if (!crc_ok) {
                RCLCPP_DEBUG(this->get_logger(), "Binary log CRC mismatch (msg_id=%u); resynchronizing", msg_id);
                gnss_parse_buf_.erase(gnss_parse_buf_.begin());
                continue;
            }
            if (msg_id == kUniheadingMessageId) {
                parse_uniheadingb(gnss_parse_buf_.data() + kBinaryHeaderLen, msg_len);
            }
            gnss_parse_buf_.erase(gnss_parse_buf_.begin(), gnss_parse_buf_.begin() + total_len);
            continue;
        }

        if (gnss_parse_buf_[0] == '$' || gnss_parse_buf_[0] == '#') {
            const auto nl_it = std::find(gnss_parse_buf_.begin(), gnss_parse_buf_.end(), '\n');
            if (nl_it == gnss_parse_buf_.end()) {
                if (gnss_parse_buf_.size() > 8192) {
                    gnss_parse_buf_.erase(gnss_parse_buf_.begin());
                    continue;
                }
                break;
            }
            const std::string line(gnss_parse_buf_.begin(), nl_it);
            const std::size_t consumed = std::distance(gnss_parse_buf_.begin(), nl_it) + 1;
            if (log_file_.is_open()) log_file_ << line << std::endl;
            process_gnss_line(line);
            gnss_parse_buf_.erase(gnss_parse_buf_.begin(), gnss_parse_buf_.begin() + consumed);
            continue;
        }
        gnss_parse_buf_.erase(gnss_parse_buf_.begin());
    }
}

void UM982Driver::process_gnss_line(std::string line)
{
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) line.pop_back();

    if (line.rfind("$GNGGA", 0) == 0 || line.rfind("$GPGGA", 0) == 0) {
        parse_gga(line);
        last_gpgga_ = line;
    } else if (line.rfind("$GNGST", 0) == 0 || line.rfind("$GPGST", 0) == 0) {
        parse_gst(line);
    } else if (line.rfind("#UNIHEADINGA", 0) == 0) {
        parse_uniheadinga(line);
    } else if (line.rfind("$GNTHS", 0) == 0 || line.rfind("$GPTHS", 0) == 0) {
        parse_ths(line);
    }
}

void UM982Driver::parse_gga(const std::string& line)
{
    if (!utils::validate_checksum(line)) return;
    const auto parts = utils::split(line.substr(0, line.find('*')), ',');
    if (parts.size() < 15) return;

    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.status.service = 15;
    if (!utils::convert_nmea_to_latlon(parts[2], parts[3], msg.latitude) ||
        !utils::convert_nmea_to_latlon(parts[4], parts[5], msg.longitude))
    {
        return;
    }

    double altitude = 0.0;
    double separation = 0.0;
    if ((!parts[9].empty() && !utils::parse_finite_double(parts[9], altitude)) ||
        (!parts[11].empty() && !utils::parse_finite_double(parts[11], separation)))
    {
        return;
    }
    msg.altitude = altitude - separation;

    int quality = 0;
    if (!parts[6].empty() && !utils::parse_int(parts[6], quality)) {
        return;
    }
    if (quality < 1) {
        msg.position_covariance[0] = -1;
    } else {
        constexpr double kGstMaxAgeSec = 0.25;
        if (!have_gst_ ||
            (rclcpp::Time(msg.header.stamp) - latest_gst_stamp_).seconds() > kGstMaxAgeSec) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Discarding GGA without a fresh GPGST covariance");
            return;
        }
        msg.position_covariance[0] = latest_gst_variance_[0];
        msg.position_covariance[4] = latest_gst_variance_[1];
        msg.position_covariance[8] = latest_gst_variance_[2];
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    }
    if (!stop_publish_) fix_pub_->publish(msg);
    if (quality >= 1) {
        publish_feedback_odometry(
            msg.latitude, msg.longitude, msg.header.stamp, msg.position_covariance);
    }
}

void UM982Driver::parse_gst(const std::string& line)
{
    double latitude_stddev = 0.0;
    double longitude_stddev = 0.0;
    double altitude_stddev = 0.0;
    if (!utils::parse_gst_standard_deviations(
            line, latitude_stddev, longitude_stddev, altitude_stddev))
    {
        RCLCPP_WARN(this->get_logger(), "Discarding malformed GPGST covariance");
        return;
    }
    latest_gst_variance_ = {
        latitude_stddev * latitude_stddev,
        longitude_stddev * longitude_stddev,
        altitude_stddev * altitude_stddev};
    latest_gst_stamp_ = this->now();
    have_gst_ = true;
}

void UM982Driver::parse_uniheadinga(const std::string& line)
{
    const auto separator = line.find(';');
    if (separator == std::string::npos) return;
    std::string payload = line.substr(separator + 1);
    const auto checksum = payload.find('*');
    if (checksum != std::string::npos) payload.resize(checksum);
    const auto fields = utils::split(payload, ',');
    if (fields.size() < 5) return;

    try {
        if (fields[0] != "SOL_COMPUTED" ||
            (fields[1] != "NARROW_INT" && fields[1] != "INS_RTKFIXED")) return;
        const double yaw_rad = utils::normalize_angle_rad(
            utils::deg2rad(90.0 - std::stod(fields[3])) -
            params_.heading_baseline_yaw_rad);
        const double pitch_rad = utils::deg2rad(-std::stod(fields[4]));
        latest_yaw_rad_ = yaw_rad;
        latest_heading_stamp_ = this->now();
        have_heading_ = true;

        tf2::Quaternion orientation;
        orientation.setRPY(0.0, pitch_rad, yaw_rad);
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = latest_heading_stamp_;
        msg.header.frame_id = params_.heading_frame_id;
        msg.pose.pose.orientation = tf2::toMsg(orientation);
        const double variance = std::pow(utils::deg2rad(0.5), 2);
        msg.pose.covariance[28] = variance;
        msg.pose.covariance[35] = variance;
        if (!stop_publish_) heading_pub_->publish(msg);
    } catch (...) {
    }
}

void UM982Driver::publish_feedback_odometry(
    double latitude_deg, double longitude_deg, const rclcpp::Time& stamp,
    const std::array<double, 9>& position_covariance)
{
    if (!params_.publish_feedback_odometry || !feedback_odom_pub_ || !have_heading_) return;

    constexpr double kEarthRadiusM = 6378137.0;
    constexpr double kPi = 3.14159265358979323846;
    const double latitude_rad = latitude_deg * kPi / 180.0;
    const double longitude_rad = longitude_deg * kPi / 180.0;
    if (!have_previous_fix_) {
        reference_latitude_rad_ = latitude_rad;
        reference_longitude_rad_ = longitude_rad;
        previous_east_m_ = 0.0;
        previous_north_m_ = 0.0;
        previous_fix_stamp_ = stamp;
        previous_yaw_rad_ = latest_yaw_rad_;
        have_previous_fix_ = true;
        have_previous_yaw_ = true;
    } else {
        const double east_m = kEarthRadiusM * std::cos(reference_latitude_rad_) *
            (longitude_rad - reference_longitude_rad_);
        const double north_m = kEarthRadiusM * (latitude_rad - reference_latitude_rad_);
        const double dt = (stamp - previous_fix_stamp_).seconds();
        if (dt > 1e-3 && dt <= 1.0) {
            const double east_velocity = (east_m - previous_east_m_) / dt;
            const double north_velocity = (north_m - previous_north_m_) / dt;
            const double raw_surge = std::cos(latest_yaw_rad_) * east_velocity +
                std::sin(latest_yaw_rad_) * north_velocity;
            const double raw_sway = -std::sin(latest_yaw_rad_) * east_velocity +
                std::cos(latest_yaw_rad_) * north_velocity;
            double yaw_delta = latest_yaw_rad_ - previous_yaw_rad_;
            yaw_delta = std::atan2(std::sin(yaw_delta), std::cos(yaw_delta));
            const double speed = std::hypot(raw_surge, raw_sway);
            if (speed <= params_.feedback_max_speed_mps) {
                const double alpha = params_.feedback_velocity_filter_alpha;
                filtered_surge_mps_ += alpha * (raw_surge - filtered_surge_mps_);
                filtered_sway_mps_ += alpha * (raw_sway - filtered_sway_mps_);
                filtered_yaw_rate_rps_ += alpha * ((yaw_delta / dt) - filtered_yaw_rate_rps_);
            }
        }
        previous_east_m_ = east_m;
        previous_north_m_ = north_m;
        previous_fix_stamp_ = stamp;
        previous_yaw_rad_ = latest_yaw_rad_;
    }

    const double east_m = kEarthRadiusM * std::cos(reference_latitude_rad_) *
        (longitude_rad - reference_longitude_rad_);
    const double north_m = kEarthRadiusM * (latitude_rad - reference_latitude_rad_);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = params_.feedback_frame_id;
    odom.child_frame_id = params_.feedback_child_frame_id;
    odom.pose.pose.position.x = east_m;
    odom.pose.pose.position.y = north_m;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, latest_yaw_rad_);
    odom.pose.pose.orientation = tf2::toMsg(orientation);
    odom.twist.twist.linear.x = filtered_surge_mps_;
    odom.twist.twist.linear.y = filtered_sway_mps_;
    odom.twist.twist.angular.z = filtered_yaw_rate_rps_;
    odom.twist.covariance[0] = 0.25;
    odom.twist.covariance[7] = 0.25;
    odom.twist.covariance[35] = 0.05;
    odom.pose.covariance[0] = position_covariance[0];
    odom.pose.covariance[7] = position_covariance[4];
    odom.pose.covariance[35] = 0.02;
    feedback_odom_pub_->publish(odom);
}

void UM982Driver::parse_ths(const std::string& line)
{
    if (!utils::validate_checksum(line)) return;
    const auto parts = utils::split(line.substr(0, line.find('*')), ',');
    if (parts.size() < 3 || parts[2] == "V") return;
    try {
        const double yaw_rad = utils::normalize_angle_rad(
            utils::deg2rad(90.0 - std::stod(parts[1])) -
            params_.heading_baseline_yaw_rad);
        latest_yaw_rad_ = yaw_rad;
        latest_heading_stamp_ = this->now();
        have_heading_ = true;
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, yaw_rad);
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = latest_heading_stamp_;
        msg.header.frame_id = params_.heading_frame_id;
        msg.pose.pose.orientation = tf2::toMsg(orientation);
        const double variance = std::pow(utils::deg2rad(0.5), 2);
        msg.pose.covariance[35] = variance;
        if (!stop_publish_) heading_pub_->publish(msg);
    } catch (...) {
    }
}

void UM982Driver::parse_uniheadingb(const uint8_t* body, std::size_t body_len)
{
    if (body_len < 44) return;
    constexpr uint32_t kNarrowInt = 50;
    constexpr uint32_t kInsRtkFixed = 56;
    constexpr uint32_t kSolComputed = 0;
    const uint32_t solution_status = utils::read_u32_le(body);
    const uint32_t position_type = utils::read_u32_le(body + 4);
    const double heading_deg = utils::read_f32_le(body + 12);
    const double pitch_deg = utils::read_f32_le(body + 16);
    const bool is_valid = solution_status == kSolComputed &&
        (position_type == kNarrowInt || position_type == kInsRtkFixed);
    const double yaw_rad = utils::normalize_angle_rad(
        utils::deg2rad(90.0 - heading_deg) -
        params_.heading_baseline_yaw_rad);
    const double pitch_rad = utils::deg2rad(-pitch_deg);

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, pitch_rad, yaw_rad);
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = params_.heading_frame_id;
    msg.pose.pose.orientation = tf2::toMsg(orientation);
    if (is_valid) {
        const double variance = std::pow(utils::deg2rad(0.5), 2);
        msg.pose.covariance[28] = variance;
        msg.pose.covariance[35] = variance;
        if (!stop_publish_) heading_pub_->publish(msg);
        latest_yaw_rad_ = yaw_rad;
        latest_heading_stamp_ = msg.header.stamp;
        have_heading_ = true;
    } else {
        msg.pose.covariance[0] = -1;
    }
}

}  // namespace um982_driver
