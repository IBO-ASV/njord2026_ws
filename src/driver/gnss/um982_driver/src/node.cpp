#include "um982_driver/node.hpp"
#include <chrono>
#include <algorithm>
#include <cctype>
#include <termios.h>

using namespace std::chrono_literals;

namespace
{
std::string format_period(double period)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << period;
    return ss.str();
}

// Unicoreバイナリログの同期バイト (Table 7-47)
constexpr uint8_t kBinarySync[3] = {0xAA, 0x44, 0xB5};
constexpr std::size_t kBinaryHeaderLen = 24; // Table 7-48
constexpr uint16_t kUniheadingMessageId = 972; // UNIHEADING (Message ID)

bool finite_quaternion(const geometry_msgs::msg::Quaternion & q)
{
    if (!std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || !std::isfinite(q.w))
    {
        return false;
    }
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(norm) && std::abs(norm - 1.0) < 1e-3;
}

std::string normalize_log_format(std::string format, const char* parameter_name)
{
    std::transform(format.begin(), format.end(), format.begin(),
        [](unsigned char value) { return static_cast<char>(std::toupper(value)); });
    if (format == "A" || format == "B") {
        return format;
    }
    RCLCPP_WARN(rclcpp::get_logger("um982_driver"),
        "%s must be A or B; using B", parameter_name);
    return "B";
}
} // namespace

namespace um982_driver
{

UM982Driver::UM982Driver(const rclcpp::NodeOptions & options)
: Node("um982_driver_node", options),
  work_guard_(io_context_.get_executor()),
  stop_publish_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing UM982 Driver Node (C++)...");

    init_parameters();

    // Log File
    if (!params_.log_file_name.empty()) {
        // ディレクトリ作成などは省略(C++ではfilesystem推奨だが簡易化のため直接Open)
        log_file_.open(params_.log_file_name, std::ios::app);
        if (log_file_.is_open()) {
            RCLCPP_INFO(this->get_logger(), "Log file opened: %s", params_.log_file_name.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open log file.");
        }
    }

    // Publishers
    fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/sensor/vehicle_gnss/fix/raw", 10);
    heading_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/sensor/vehicle_gnss/compass/raw", 10);
    if (params_.publish_feedback_odometry) {
        feedback_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            params_.feedback_odometry_topic, 10);
    }

    // Subscriber
    ctrl_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/sensor/vehicle_gnss/command", 10,
        std::bind(&UM982Driver::ctrl_callback, this, std::placeholders::_1));
    hot_restart_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/sensor/vehicle_gnss/hot_restart",
        std::bind(&UM982Driver::hot_restart, this, std::placeholders::_1, std::placeholders::_2));

    // IO Thread Start
    io_thread_ = std::thread([this]() {
        // An exception escaping an Asio completion handler otherwise invokes
        // std::terminate, which used to make the whole GNSS node abort.
        while (!io_context_.stopped()) {
            try {
                io_context_.run();
                break;
            } catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(), "Unhandled GNSS I/O exception: %s", e.what());
            }
        }
    });

    // Connect
    try {
        init_gnss_connection();
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "GNSS Connection Failed: %s", e.what());
    }

    if (params_.rtk_enable) {
        init_rtk_connection();
        // 5秒ごとにGGA送信
        rtk_gga_timer_ = this->create_wall_timer(
            5s, std::bind(&UM982Driver::rtk_send_gga_callback, this));
    }
}

UM982Driver::~UM982Driver()
{
    // 非同期読み込みを止めてから、起動時に書き込んだ設定を元に戻す
    // (UNLOG)。SAVECONFIGしていないためNVRAMには影響しないが、UNLOGを
    // 送らないと次回接続時にこのノードが設定したログ(UNIHEADINGB等)を
    // 出力し続けたままになる。
    io_context_.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }

    revert_gnss_output();

    if (log_file_.is_open()) {
        log_file_.close();
    }
}

void UM982Driver::revert_gnss_output()
{
    bool port_open = (serial_port_ && serial_port_->is_open()) ||
        (tcp_socket_ && tcp_socket_->is_open());
    if (!port_open) {
        return;
    }

    try {
        write_to_gnss("UNLOG\r\n");
        RCLCPP_INFO(this->get_logger(), "Reverted UM982 output configuration (UNLOG)");
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to revert UM982 output on shutdown: %s", e.what());
    }
}

void UM982Driver::init_parameters()
{
    // Declare parameters with defaults
    this->declare_parameter("GNSS_SerialPort", "/dev/ttyUSB0");
    this->declare_parameter("GNSS_Baudrate", 115200);
    this->declare_parameter("tcp_ip", "192.168.0.126");
    this->declare_parameter("tcp_port", 23);
    this->declare_parameter("uart_or_tcp", "tcp");
    this->declare_parameter("FIX_FREQ", 20);
    this->declare_parameter("HEADING_FREQ", 20);
    this->declare_parameter("RTK_STATUS_FREQ", 1);
    this->declare_parameter("UNIHEADING_FORMAT", "B");
    this->declare_parameter("RTK_STATUS_FORMAT", "B");
    this->declare_parameter("GNSS_RTK_Enable", true);
    this->declare_parameter("Heading_FrameID", "odom");
    this->declare_parameter("heading_baseline_yaw_rad", 0.0);
    this->declare_parameter("log_file_name", "um982.log"); // 簡易化のため固定名デフォルト
    this->declare_parameter("publish_feedback_odometry", false);
    // A relative name keeps the published Odometry independent from the
    // consumer.  Bringup launch files remap it to the required feedback topic.
    this->declare_parameter("feedback_odometry_topic", "odometry/feedback");
    this->declare_parameter("feedback_frame_id", "odom");
    this->declare_parameter("feedback_child_frame_id", "base_link");
    this->declare_parameter("feedback_velocity_filter_alpha", 0.35);
    this->declare_parameter("feedback_max_speed_mps", 4.0);
    this->declare_parameter("NTRIP_Server", params_.ntrip_server);
    this->declare_parameter("NTRIP_Port", params_.ntrip_port);
    this->declare_parameter("NTRIP_Mountpoint", params_.mountpoint);
    this->declare_parameter("NTRIP_Username", params_.username);
    this->declare_parameter("NTRIP_Password", params_.password);

    // Get parameters
    params_.gnss_port = this->get_parameter("GNSS_SerialPort").as_string();
    params_.gnss_baud = this->get_parameter("GNSS_Baudrate").as_int();
    params_.tcp_ip = this->get_parameter("tcp_ip").as_string();
    params_.tcp_port = this->get_parameter("tcp_port").as_int();
    params_.uart_or_tcp = this->get_parameter("uart_or_tcp").as_string();
    params_.fix_freq = this->get_parameter("FIX_FREQ").as_int();
    params_.heading_freq = this->get_parameter("HEADING_FREQ").as_int();
    params_.rtk_status_freq = static_cast<int>(std::max<int64_t>(
        1, this->get_parameter("RTK_STATUS_FREQ").as_int()));
    params_.heading_log_format = normalize_log_format(
        this->get_parameter("UNIHEADING_FORMAT").as_string(), "UNIHEADING_FORMAT");
    params_.rtk_status_log_format = normalize_log_format(
        this->get_parameter("RTK_STATUS_FORMAT").as_string(), "RTK_STATUS_FORMAT");
    params_.rtk_enable = this->get_parameter("GNSS_RTK_Enable").as_bool();
    params_.heading_frame_id = this->get_parameter("Heading_FrameID").as_string();
    params_.heading_baseline_yaw_rad =
        this->get_parameter("heading_baseline_yaw_rad").as_double();
    if (!std::isfinite(params_.heading_baseline_yaw_rad)) {
        throw std::invalid_argument("heading_baseline_yaw_rad must be finite");
    }
    params_.log_file_name = this->get_parameter("log_file_name").as_string();
    params_.publish_feedback_odometry =
        this->get_parameter("publish_feedback_odometry").as_bool();
    params_.feedback_odometry_topic =
        this->get_parameter("feedback_odometry_topic").as_string();
    params_.feedback_frame_id =
        this->get_parameter("feedback_frame_id").as_string();
    params_.feedback_child_frame_id =
        this->get_parameter("feedback_child_frame_id").as_string();
    params_.feedback_velocity_filter_alpha = std::clamp(
        this->get_parameter("feedback_velocity_filter_alpha").as_double(), 0.0, 1.0);
    params_.feedback_max_speed_mps = std::max(
        0.1, this->get_parameter("feedback_max_speed_mps").as_double());
    params_.ntrip_server = this->get_parameter("NTRIP_Server").as_string();
    params_.ntrip_port = this->get_parameter("NTRIP_Port").as_int();
    params_.mountpoint = this->get_parameter("NTRIP_Mountpoint").as_string();
    params_.username = this->get_parameter("NTRIP_Username").as_string();
    params_.password = this->get_parameter("NTRIP_Password").as_string();
    
    RCLCPP_INFO(this->get_logger(), "Mode: %s", params_.uart_or_tcp.c_str());
}

// --------------------------------------------------------------------------
// GNSS Connection & IO
// --------------------------------------------------------------------------

void UM982Driver::init_gnss_connection()
{
    if (params_.uart_or_tcp == "uart") {
        serial_port_ = std::make_unique<boost::asio::serial_port>(io_context_);
        serial_port_->open(params_.gnss_port);
        serial_port_->set_option(boost::asio::serial_port_base::baud_rate(params_.gnss_baud));
        RCLCPP_INFO(this->get_logger(), "Opened Serial: %s", params_.gnss_port.c_str());
    } else {
        tcp_socket_ = std::make_unique<boost::asio::ip::tcp::socket>(io_context_);
        boost::asio::ip::tcp::endpoint endpoint(
            boost::asio::ip::address::from_string(params_.tcp_ip), params_.tcp_port);
        tcp_socket_->connect(endpoint);
        RCLCPP_INFO(this->get_logger(), "Connected TCP: %s:%d", params_.tcp_ip.c_str(), params_.tcp_port);
    }

    configure_gnss_output();

    start_gnss_read();
}

void UM982Driver::configure_gnss_output()
{
    double fix_p = 1.0 / params_.fix_freq;
    double head_p = 1.0 / params_.heading_freq;
    std::string fix_period = format_period(fix_p);
    std::string heading_period = format_period(head_p);

    // Apply volatile startup configuration only. Do not send SAVECONFIG.
    // GPGGA has no binary counterpart (GPGGAB is rejected by the receiver: "PARSING
    // FAILED NO MATCHING FUNC"), so position stays NMEA/ASCII.  Although UM982
    // supports UNIHEADINGB, the live receiver repeatedly delivers CRC-invalid
    // frames at 115200 bps.  The N4 command reference specifies GPTHS as the
    // UM982 true-heading log with a validity flag, so use that validated ASCII
    // message as the sole heading source until the serial binary stream is proven
    // reliable.
    write_to_gnss("UNLOG\r\n");

    // UNLOG can stop a binary message halfway through transmission.  If that
    // fragment remains in the UART receive queue, appending the command reply
    // and the first newly configured message produces one apparent CRC error
    // at every startup.  No asynchronous read is active yet, so wait for the
    // receiver to process UNLOG and discard only this stale startup input.
    if (serial_port_ && serial_port_->is_open()) {
        std::this_thread::sleep_for(100ms);
        if (::tcflush(serial_port_->native_handle(), TCIFLUSH) != 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to flush stale UM982 UART input");
        }
        gnss_parse_buf_.clear();
    }

    write_to_gnss("MODE ROVER\r\n");
    write_to_gnss("GPGGA " + fix_period + "\r\n");
    write_to_gnss("GPGST " + fix_period + "\r\n");
    write_to_gnss("GPTHS " + heading_period + "\r\n");
    RCLCPP_INFO(this->get_logger(),
        "Configured volatile UM982 output: GPGGA/GPGST(ASCII)=%ss, GPTHS(ASCII)=%ss",
        fix_period.c_str(), heading_period.c_str());
}

void UM982Driver::hot_restart(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    bool expected = false;
    if (!hot_restart_in_progress_.compare_exchange_strong(expected, true)) {
        response->success = false;
        response->message = "UM982 hot restart already in progress";
        return;
    }
    try {
        // RESET restarts the receiver without clearing its stored navigation information.
        write_to_gnss("RESET\r\n");
        gnss_parse_buf_.clear();
        have_heading_ = have_previous_fix_ = have_previous_yaw_ = have_gst_ = false;
        filtered_surge_mps_ = filtered_sway_mps_ = filtered_yaw_rate_rps_ = 0.0;
        hot_restart_reconfigure_timer_ = create_wall_timer(3s, [this]() {
            configure_gnss_output();
            hot_restart_in_progress_ = false;
            hot_restart_reconfigure_timer_->cancel();
        });
        response->success = true;
        response->message = "UM982 RESET sent; volatile output will be restored in 3 seconds";
    } catch (const std::exception & error) {
        hot_restart_in_progress_ = false;
        response->success = false;
        response->message = error.what();
    }
}

void UM982Driver::write_to_gnss(const std::string& data)
{
    std::vector<uint8_t> bytes(data.begin(), data.end());
    write_to_gnss(bytes);
}

void UM982Driver::write_to_gnss(const std::vector<uint8_t>& data)
{
    // シリアル/TCPへの書き込みは排他制御推奨
    std::lock_guard<std::mutex> lock(gnss_write_mutex_);
    if (serial_port_ && serial_port_->is_open()) {
        boost::asio::write(*serial_port_, boost::asio::buffer(data));
    } else if (tcp_socket_ && tcp_socket_->is_open()) {
        boost::asio::write(*tcp_socket_, boost::asio::buffer(data));
    }
}

void UM982Driver::start_gnss_read()
{
    // ASCII(NMEA)とバイナリ(UNIHEADINGB等)が混在するため、行区切りではなく
    // 生バイトを読み込んでバッファ側でメッセージ境界を判定する。
    auto handler = std::bind(&UM982Driver::on_gnss_read, this, std::placeholders::_1, std::placeholders::_2);

    if (serial_port_) {
        serial_port_->async_read_some(boost::asio::buffer(gnss_read_chunk_), handler);
    } else if (tcp_socket_) {
        tcp_socket_->async_read_some(boost::asio::buffer(gnss_read_chunk_), handler);
    }
}

void UM982Driver::on_gnss_read(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (!error) {
        gnss_parse_buf_.insert(
            gnss_parse_buf_.end(), gnss_read_chunk_.begin(), gnss_read_chunk_.begin() + bytes_transferred);

        // Re-arm before parsing. If a future parser unexpectedly throws, an
        // outstanding read still exists and the input stream cannot silently
        // die while the process remains alive.
        start_gnss_read();
        try {
            process_gnss_buffer();
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "GNSS parse exception; resynchronizing: %s", e.what());
            gnss_parse_buf_.clear();
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Unknown GNSS parse exception; resynchronizing");
            gnss_parse_buf_.clear();
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "GNSS Read Error: %s", error.message().c_str());
        // 再接続ロジックを入れるならここ
        // 簡易実装として少し待ってリトライなどを検討
    }
}

// Moved to parsing.cpp.  Keep this transition block excluded until the next
// source-history cleanup, so the refactor remains easy to audit as a move.
#if 0
void UM982Driver::process_gnss_buffer()
{
    while (!gnss_parse_buf_.empty()) {
        // --- バイナリログ (0xAA 0x44 0xB5 始まり) ---
        if (gnss_parse_buf_.size() >= 3 &&
            gnss_parse_buf_[0] == kBinarySync[0] &&
            gnss_parse_buf_[1] == kBinarySync[1] &&
            gnss_parse_buf_[2] == kBinarySync[2])
        {
            if (gnss_parse_buf_.size() < kBinaryHeaderLen) {
                break; // ヘッダ全体がまだ届いていない
            }

            uint16_t msg_id = utils::read_u16_le(&gnss_parse_buf_[4]);
            uint16_t msg_len = utils::read_u16_le(&gnss_parse_buf_[6]);
            std::size_t total_len = kBinaryHeaderLen + msg_len + 4; // +4 = CRC32

            if (total_len > gnss_parse_buf_.size()) {
                if (total_len > 8192) {
                    // ヘッダが壊れている可能性: 同期バイトを捨てて再同期
                    gnss_parse_buf_.erase(gnss_parse_buf_.begin(), gnss_parse_buf_.begin() + 3);
                    continue;
                }
                break; // フレーム全体がまだ届いていない
            }

            bool crc_ok = utils::calculate_unicore_crc32(gnss_parse_buf_.data(), total_len - 4) ==
                utils::read_u32_le(&gnss_parse_buf_[total_len - 4]);

            if (!crc_ok) {
                RCLCPP_WARN(this->get_logger(), "Binary log CRC mismatch (msg_id=%u); resynchronizing", msg_id);
                // The declared length may span bytes belonging to a following
                // valid frame.  Drop only the first sync byte and let the
                // normal scanner find the next complete 0xAA 0x44 0xB5.
                gnss_parse_buf_.erase(gnss_parse_buf_.begin());
                continue;
            }

            if (msg_id == kUniheadingMessageId) {
                parse_uniheadingb(gnss_parse_buf_.data() + kBinaryHeaderLen, msg_len);
            }

            gnss_parse_buf_.erase(gnss_parse_buf_.begin(), gnss_parse_buf_.begin() + total_len);
            continue;
        }

        // --- ASCIIログ ($ または # 始まり、\n終端) ---
        if (gnss_parse_buf_[0] == '$' || gnss_parse_buf_[0] == '#') {
            auto nl_it = std::find(gnss_parse_buf_.begin(), gnss_parse_buf_.end(), '\n');
            if (nl_it == gnss_parse_buf_.end()) {
                if (gnss_parse_buf_.size() > 8192) {
                    gnss_parse_buf_.erase(gnss_parse_buf_.begin()); // 異常に長い: 再同期
                    continue;
                }
                break; // 行全体がまだ届いていない
            }

            std::string line(gnss_parse_buf_.begin(), nl_it);
            std::size_t consumed = std::distance(gnss_parse_buf_.begin(), nl_it) + 1;

            if (log_file_.is_open()) {
                log_file_ << line << std::endl;
            }
            process_gnss_line(line);

            gnss_parse_buf_.erase(gnss_parse_buf_.begin(), gnss_parse_buf_.begin() + consumed);
            continue;
        }

        // 認識できない先頭バイト: 1バイト捨てて再同期
        gnss_parse_buf_.erase(gnss_parse_buf_.begin());
    }
}

void UM982Driver::process_gnss_line(std::string line)
{
    // 末尾の \r などを除去
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
        line.pop_back();
    }

    if (line.rfind("$GNGGA", 0) == 0 || line.rfind("$GPGGA", 0) == 0) {
        parse_gga(line);
        last_gpgga_ = line; // RTK用に保存
    } else if (line.rfind("$GNGST", 0) == 0 || line.rfind("$GPGST", 0) == 0) {
        parse_gst(line);
    } else if (line.rfind("#UNIHEADINGA", 0) == 0) {
        parse_uniheadinga(line);
    } else if (line.rfind("$GNTHS", 0) == 0 || line.rfind("$GPTHS", 0) == 0) {
        parse_ths(line);
    }
    // UNIHEADINGB is dispatched directly from process_gnss_buffer().
}

// --------------------------------------------------------------------------
// Parsing Logic
// --------------------------------------------------------------------------

void UM982Driver::parse_gga(const std::string& line)
{
    if (!utils::validate_checksum(line)) {
        return;
    }

    // * チェックサムを除去して分割
    std::string payload = line.substr(0, line.find('*'));
    auto parts = utils::split(payload, ',');

    if (parts.size() < 15) {
        return;
    }

    int quality = 0;
    if (!utils::parse_int(parts[6], quality) || quality < 0) {
        RCLCPP_WARN(this->get_logger(), "Discarding GGA with invalid fix quality");
        return;
    }

    // $GNGGA,time,lat,N,lon,E,qual,sats,hdop,alt,M,sep,M,age,ref
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "um982_link";
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
        sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
        sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS |
        sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;

    if (quality == 0) {
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        msg.position_covariance_type =
            sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        // A no-fix record is useful for diagnostics, but must never enter
        // navsat_transform as a normal measurement.
        return;
    }

    double altitude_msl = 0.0;
    double geoid_separation = 0.0;
    if (!utils::convert_nmea_to_latlon(parts[2], parts[3], msg.latitude) ||
        !utils::convert_nmea_to_latlon(parts[4], parts[5], msg.longitude) ||
        !utils::parse_finite_double(parts[9], altitude_msl) ||
        !utils::parse_finite_double(parts[11], geoid_separation))
    {
        RCLCPP_WARN(this->get_logger(), "Discarding GGA with malformed or non-finite position");
        return;
    }
    msg.altitude = altitude_msl - geoid_separation;
    if (!std::isfinite(msg.altitude)) {
        return;
    }
    double utc_seconds = 0.0;
    if (!utils::parse_nmea_utc_seconds(parts[1], utc_seconds)) {
        RCLCPP_WARN(this->get_logger(), "Discarding GGA with invalid UTC time");
        return;
    }
    msg.status.status = quality >= 2 ?
        sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX :
        sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    pending_fix_ = msg;
    pending_fix_utc_seconds_ = utc_seconds;
    have_pending_fix_ = true;
}

void UM982Driver::parse_gst(const std::string& line)
{
    if (!utils::validate_checksum(line) || !have_pending_fix_) {
        return;
    }
    const auto parts = utils::split(line.substr(0, line.find('*')), ',');
    if (parts.size() < 9) {
        return;
    }
    double utc_seconds = 0.0;
    double std_lat_m = 0.0;
    double std_lon_m = 0.0;
    double std_alt_m = 0.0;
    if (!utils::parse_nmea_utc_seconds(parts[1], utc_seconds) ||
        !utils::parse_finite_double(parts[6], std_lat_m) ||
        !utils::parse_finite_double(parts[7], std_lon_m) ||
        !utils::parse_finite_double(parts[8], std_alt_m) ||
        std_lat_m < 0.0 || std_lon_m < 0.0 || std_alt_m < 0.0 ||
        std::abs(utc_seconds - pending_fix_utc_seconds_) > 0.05)
    {
        RCLCPP_WARN(this->get_logger(), "Discarding unmatched or invalid GST covariance");
        return;
    }

    // NavSatFix covariance is ENU, whereas GST reports latitude/longitude/height.
    pending_fix_.position_covariance[0] = std_lon_m * std_lon_m;
    pending_fix_.position_covariance[4] = std_lat_m * std_lat_m;
    pending_fix_.position_covariance[8] = std_alt_m * std_alt_m;
    pending_fix_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
    if (!stop_publish_) {
        fix_pub_->publish(pending_fix_);
    }
    publish_feedback_odometry(
        pending_fix_.latitude, pending_fix_.longitude, pending_fix_.header.stamp);
    have_pending_fix_ = false;
}

void UM982Driver::parse_uniheadinga(const std::string& line)
{
    const auto separator = line.find(';');
    if (separator == std::string::npos) return;

    std::string payload = line.substr(separator + 1);
    const auto checksum = payload.find('*');
    if (checksum != std::string::npos) payload.resize(checksum);
    const auto fields = utils::split(payload, ',');
    // sol_stat, pos_type, baseline, heading, pitch, ...
    if (fields.size() < 5) return;

    try {
        if (fields[0] != "SOL_COMPUTED" ||
            (fields[1] != "NARROW_INT" && fields[1] != "INS_RTKFIXED")) {
            return;
        }
        const double yaw_rad = utils::deg2rad(90.0 - std::stod(fields[3]));
        const double pitch_rad = utils::deg2rad(-std::stod(fields[4]));

        latest_yaw_rad_ = yaw_rad;
        latest_heading_stamp_ = this->now();
        have_heading_ = true;

        tf2::Quaternion q;
        q.setRPY(0.0, pitch_rad, yaw_rad);
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = latest_heading_stamp_;
        msg.header.frame_id = params_.heading_frame_id;
        msg.pose.pose.orientation = tf2::toMsg(q);
        const double variance = std::pow(utils::deg2rad(0.5), 2);
        msg.pose.covariance[28] = variance;
        msg.pose.covariance[35] = variance;
        if (!stop_publish_) heading_pub_->publish(msg);
    } catch (...) {
    }
}

void UM982Driver::publish_feedback_odometry(
    double latitude_deg, double longitude_deg, const rclcpp::Time& stamp)
{
    if (!params_.publish_feedback_odometry || !feedback_odom_pub_ || !have_heading_) {
        return;
    }
    if (!std::isfinite(latitude_deg) || !std::isfinite(longitude_deg) ||
        !std::isfinite(latest_yaw_rad_))
    {
        RCLCPP_WARN(this->get_logger(), "Rejecting non-finite feedback odometry input");
        return;
    }

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
            const double raw_yaw_rate = yaw_delta / dt;
            const double speed = std::hypot(raw_surge, raw_sway);
            if (speed <= params_.feedback_max_speed_mps) {
                const double alpha = params_.feedback_velocity_filter_alpha;
                filtered_surge_mps_ += alpha * (raw_surge - filtered_surge_mps_);
                filtered_sway_mps_ += alpha * (raw_sway - filtered_sway_mps_);
                filtered_yaw_rate_rps_ += alpha * (raw_yaw_rate - filtered_yaw_rate_rps_);
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
    // Relative ENU position with the first valid fix as the origin. The
    // configured frame decides whether a consumer treats that datum-fixed
    // plane as a local odom frame or as the navigation map frame.
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
    odom.pose.covariance[0] = 0.25;
    odom.pose.covariance[7] = 0.25;
    odom.pose.covariance[35] = 0.02;
    if (!std::isfinite(odom.pose.pose.position.x) ||
        !std::isfinite(odom.pose.pose.position.y) ||
        !finite_quaternion(odom.pose.pose.orientation) ||
        !std::isfinite(odom.twist.twist.linear.x) ||
        !std::isfinite(odom.twist.twist.linear.y) ||
        !std::isfinite(odom.twist.twist.angular.z))
    {
        RCLCPP_WARN(this->get_logger(), "Rejecting non-finite feedback Odometry");
        return;
    }
    feedback_odom_pub_->publish(odom);
}

void UM982Driver::parse_ths(const std::string& line)
{
    if (!utils::validate_checksum(line)) return;

    std::string payload = line.substr(0, line.find('*'));
    auto parts = utils::split(payload, ',');
    if (parts.size() < 3 || parts[2] == "V") return;

    double heading_deg = 0.0;
    if (!utils::parse_finite_double(parts[1], heading_deg)) {
        RCLCPP_WARN(this->get_logger(), "Discarding THS with invalid heading");
        return;
    }
    const double yaw_rad = utils::deg2rad(90.0 - heading_deg);
    if (!std::isfinite(yaw_rad)) {
        return;
    }

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_rad);

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = params_.heading_frame_id;
    msg.pose.pose.orientation = tf2::toMsg(q);
    if (!finite_quaternion(msg.pose.pose.orientation)) {
        RCLCPP_WARN(this->get_logger(), "Discarding THS with invalid quaternion");
        return;
    }

    const double standard_deviation = utils::deg2rad(0.5);
    msg.pose.covariance[35] = standard_deviation * standard_deviation;

    latest_yaw_rad_ = yaw_rad;
    latest_heading_stamp_ = msg.header.stamp;
    have_heading_ = true;

    if (!stop_publish_) {
        heading_pub_->publish(msg);
    }
}

void UM982Driver::parse_uniheadingb(const uint8_t* body, std::size_t body_len)
{
    // UNIHEADING Message Structure (Unicore Reference Commands Manual Table 7-115)
    // body offset: 0=sol_stat(u32) 4=pos_type(u32) 8=length(f32) 12=heading(f32)
    //              16=pitch(f32) 20=reserved(f32) 24=hdgstddev(f32) 28=ptchstddev(f32)
    //              32=stn_id(char[4]) 36=#SVs 37=#solnSVs 38=#obs 39=#multi
    //              40=reserved 41=ext_sol_stat 42=galileo/bds3 mask 43=gps/glonass/bds2 mask
    if (body_len < 44) return;

    // Position or Velocity Type (Table 0-4)
    constexpr uint32_t kNarrowInt = 50;
    constexpr uint32_t kInsRtkFixed = 56;
    // Solution Status (Table 0-5)
    constexpr uint32_t kSolComputed = 0;

    uint32_t sol_stat = utils::read_u32_le(body + 0);
    uint32_t pos_type = utils::read_u32_le(body + 4);
    double heading_deg = utils::read_f32_le(body + 12);
    double pitch_deg = utils::read_f32_le(body + 16);

    if (!std::isfinite(heading_deg) || !std::isfinite(pitch_deg)) {
        RCLCPP_WARN(this->get_logger(), "Discarding UNIHEADINGB with non-finite angles");
        return;
    }

    double yaw_rad = utils::deg2rad(90.0 - heading_deg);
    double pitch_rad = utils::deg2rad(-1.0 * pitch_deg);

    tf2::Quaternion q;
    q.setRPY(0, pitch_rad, yaw_rad);

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = params_.heading_frame_id;
    msg.pose.pose.orientation = tf2::toMsg(q);
    if (!finite_quaternion(msg.pose.pose.orientation)) {
        RCLCPP_WARN(this->get_logger(), "Discarding UNIHEADINGB with invalid quaternion");
        return;
    }

    bool is_valid = (sol_stat == kSolComputed) &&
        (pos_type == kNarrowInt || pos_type == kInsRtkFixed);

    if (is_valid) {
        double var = utils::deg2rad(0.5);
        var = var * var;
        msg.pose.covariance[28] = var; // rotation about Y (Pitch)
        msg.pose.covariance[35] = var; // rotation about Z (Yaw)
    } else {
        msg.pose.covariance[0] = -1; // invalid
    }

    if (!stop_publish_ && is_valid) {
        heading_pub_->publish(msg);
    }
    if (is_valid) {
        latest_yaw_rad_ = yaw_rad;
        latest_heading_stamp_ = msg.header.stamp;
        have_heading_ = true;
    }
}

#endif

// --------------------------------------------------------------------------
// RTK (NTRIP)
// --------------------------------------------------------------------------

void UM982Driver::init_rtk_connection()
{
    // The five-second GGA timer may fire while DNS/TCP connection is still
    // pending.  Do not replace the socket/resolver beneath that operation.
    if (rtk_connection_in_progress_.exchange(true)) {
        return;
    }

    rtk_response_buffer_.clear();
    rtk_response_header_received_ = false;
    is_rtk_connected_ = false;
    try {
        rtk_socket_ = std::make_unique<boost::asio::ip::tcp::socket>(io_context_);
        rtk_resolver_ = std::make_unique<boost::asio::ip::tcp::resolver>(io_context_);

        // DNS resolution can throw (for example while mDNS is unavailable).
        auto endpoints = rtk_resolver_->resolve(
            params_.ntrip_server, std::to_string(params_.ntrip_port));
        boost::asio::async_connect(*rtk_socket_, endpoints,
            [this](const boost::system::error_code& error, const boost::asio::ip::tcp::endpoint& /*endpoint*/) {
                if (error) {
                    RCLCPP_ERROR(this->get_logger(), "RTK Connect Failed: %s", error.message().c_str());
                    rtk_connection_in_progress_ = false;
                    return;
                }

                try {
                    connect_rtk_client();
                } catch (const std::exception & e) {
                    is_rtk_connected_ = false;
                    rtk_connection_in_progress_ = false;
                    RCLCPP_ERROR(this->get_logger(), "RTK request setup failed: %s", e.what());
                }
            });
    } catch (const std::exception & e) {
        rtk_connection_in_progress_ = false;
        RCLCPP_ERROR(this->get_logger(), "RTK connection setup failed: %s", e.what());
    }
}

void UM982Driver::connect_rtk_client()
{
    std::stringstream request;
    request << "GET /" << params_.mountpoint << " HTTP/1.1\r\n";
    request << "Host: " << params_.ntrip_server << ":" << params_.ntrip_port << "\r\n";
    request << "Ntrip-Version: Ntrip/2.0\r\n";
    request << "User-Agent: NTRIP Client/1.0\r\n";
    if (!params_.username.empty() || !params_.password.empty()) {
        std::string auth_str = params_.username + ":" + params_.password;
        request << "Authorization: Basic " << utils::base64_encode(auth_str) << "\r\n";
    }
    // NTRIP correction streams are long-lived.  Asking the caster to close the
    // connection causes compliant casters to terminate it immediately after
    // their response header.
    request << "Connection: keep-alive\r\n";
    request << "\r\n";

    std::string req_str = request.str();
    if (!rtk_socket_ || !rtk_socket_->is_open()) {
        throw std::runtime_error("RTK socket is not open after connect");
    }
    boost::system::error_code write_error;
    boost::asio::write(*rtk_socket_, boost::asio::buffer(req_str), write_error);
    if (write_error) {
        throw boost::system::system_error(write_error);
    }

    is_rtk_connected_ = true;
    rtk_connection_in_progress_ = false;
    RCLCPP_INFO(this->get_logger(), "Connected to RTK Server. Waiting for data...");

    start_rtk_read();
}

void UM982Driver::start_rtk_read()
{
    // RTKデータはバイナリかもしれないので、read_some で受信する
    auto handler = std::bind(&UM982Driver::on_rtk_read, this, std::placeholders::_1, std::placeholders::_2);
    rtk_socket_->async_read_some(boost::asio::buffer(rtk_read_chunk_), handler);
}

void UM982Driver::on_rtk_read(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (!error) {
        const uint8_t * data_begin = rtk_read_chunk_.data();
        std::size_t data_size = bytes_transferred;
        std::vector<uint8_t> response_payload;

        // The first bytes from an NTRIP caster are an HTTP/NTRIP response
        // header.  Do not forward that ASCII header to UM982 as RTCM.
        if (!rtk_response_header_received_) {
            rtk_response_buffer_.insert(
                rtk_response_buffer_.end(), data_begin, data_begin + data_size);

            static constexpr char kLineEnd[] = "\r\n";
            static constexpr char kHeaderEnd[] = "\r\n\r\n";
            const bool is_icy_response =
                rtk_response_buffer_.size() >= 4 &&
                std::equal(rtk_response_buffer_.begin(), rtk_response_buffer_.begin() + 4, "ICY ");
            const auto header_end = is_icy_response
                ? std::search(
                    rtk_response_buffer_.begin(), rtk_response_buffer_.end(),
                    kLineEnd, kLineEnd + 2)
                : std::search(
                    rtk_response_buffer_.begin(), rtk_response_buffer_.end(),
                    kHeaderEnd, kHeaderEnd + 4);
            if (header_end == rtk_response_buffer_.end()) {
                if (rtk_response_buffer_.size() > 16 * 1024) {
                    RCLCPP_ERROR(this->get_logger(), "RTK response header exceeds 16 KiB");
                    is_rtk_connected_ = false;
                    boost::system::error_code close_error;
                    rtk_socket_->close(close_error);
                    return;
                }
                start_rtk_read();
                return;
            }

            // NTRIP v1 casters, including the project's ntripcaster, send
            // only "ICY 200 OK\\r\\n" before the binary stream.  HTTP/NTRIP
            // v2 responses have a complete \r\n\r\n-terminated header block.
            const auto header_size = static_cast<std::size_t>(
                std::distance(rtk_response_buffer_.begin(), header_end)) +
                (is_icy_response ? 2 : 4);
            const std::string header(
                rtk_response_buffer_.begin(), rtk_response_buffer_.begin() + header_size);
            const bool accepted =
                header.rfind("ICY 200", 0) == 0 ||
                header.rfind("HTTP/1.0 200", 0) == 0 ||
                header.rfind("HTTP/1.1 200", 0) == 0;
            if (!accepted) {
                RCLCPP_ERROR(this->get_logger(), "RTK caster rejected request: %s",
                    header.substr(0, header.find("\r\n")).c_str());
                is_rtk_connected_ = false;
                boost::system::error_code close_error;
                rtk_socket_->close(close_error);
                return;
            }

            rtk_response_header_received_ = true;
            response_payload.assign(
                rtk_response_buffer_.begin() + header_size, rtk_response_buffer_.end());
            rtk_response_buffer_.clear();
            data_begin = response_payload.data();
            data_size = response_payload.size();
        }

        if (data_size > 0) {
            try {
                write_to_gnss(std::vector<uint8_t>(data_begin, data_begin + data_size));
            } catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to forward RTK data to UM982: %s", e.what());
                is_rtk_connected_ = false;
                return;
            }
        }

        // 次の読み込み
        start_rtk_read();
    } else {
        RCLCPP_ERROR(this->get_logger(), "RTK Read Error: %s", error.message().c_str());
        is_rtk_connected_ = false;
        // リコネクト処理を入れるならここ
    }
}

void UM982Driver::rtk_send_gga_callback()
{
    if (!is_rtk_connected_) {
        // 未接続なら再接続試行
        init_rtk_connection(); // 簡易的な再呼び出し(本来は状態管理が必要)
        return;
    }

    if (!last_gpgga_.empty() &&
        (last_gpgga_.rfind("$GNGGA", 0) == 0 || last_gpgga_.rfind("$GPGGA", 0) == 0)) {
        // GNGGAをRTKサーバーへ送信して位置を通知
        std::string msg = last_gpgga_ + "\r\n";
        boost::system::error_code ignored_error;
        boost::asio::write(*rtk_socket_, boost::asio::buffer(msg), ignored_error);
    }
}

void UM982Driver::ctrl_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == "shutdown") {
        rclcpp::shutdown();
    } else if (msg->data == "stop_publish") {
        stop_publish_ = true;
    } else if (msg->data == "start_publish") {
        stop_publish_ = false;
    }
}

} // namespace um982_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(um982_driver::UM982Driver)
