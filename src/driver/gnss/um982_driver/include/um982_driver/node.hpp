#ifndef UM982_DRIVER_NODE_HPP_
#define UM982_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <fstream>
#include <atomic>
#include <array>
#include <cstdint>

// 前述のutilsをインクルード
#include "um982_driver/utils.hpp"

namespace um982_driver
{

class UM982Driver : public rclcpp::Node
{
public:
    explicit UM982Driver(const rclcpp::NodeOptions & options);
    ~UM982Driver();

private:
    // --- 初期化 ---
    void init_parameters();
    void init_gnss_connection();
    void init_rtk_connection();
    void configure_gnss_output();
    void revert_gnss_output(); // シャットダウン時に起動時設定を元に戻す(UNLOG)

    // --- 通信 (Boost.Asio) ---
    void start_gnss_read();
    void on_gnss_read(const boost::system::error_code& error, std::size_t bytes_transferred);
    void write_to_gnss(const std::vector<uint8_t>& data); // RTKデータ転送用
    void write_to_gnss(const std::string& data);          // コマンド送信

    void connect_rtk_client();
    void start_rtk_read();
    void on_rtk_read(const boost::system::error_code& error, std::size_t bytes_transferred);

    // --- データ処理 ---
    // GNSSはASCII(NMEA/$,#行)とバイナリ(UNIHEADINGB等, 0xAA 0x44 0xB5始まり)が
    // 混在するため、行区切りではなくバイトバッファを都度スキャンして分離する。
    void process_gnss_buffer();
    void process_gnss_line(std::string line);
    void parse_gga(const std::string& line);
    void parse_gst(const std::string& line);
    void parse_uniheadinga(const std::string& line);
    void parse_uniheadingb(const uint8_t* body, std::size_t body_len);
    void parse_ths(const std::string& line);
    void publish_feedback_odometry(
        double latitude_deg, double longitude_deg, const rclcpp::Time& stamp,
        const std::array<double, 9>& position_covariance);

    // --- ROS Callbacks/Timers ---
    void ctrl_callback(const std_msgs::msg::String::SharedPtr msg);
    void rtk_send_gga_callback(); // RTKサーバーへGGAを送信(5秒周期)
    void hot_restart(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // --- メンバ変数 ---
    
    // Asio Context & Thread
    boost::asio::io_context io_context_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::thread io_thread_;

    // GNSS Device
    std::unique_ptr<boost::asio::serial_port> serial_port_;
    std::unique_ptr<boost::asio::ip::tcp::socket> tcp_socket_;
    std::array<uint8_t, 4096> gnss_read_chunk_;
    std::vector<uint8_t> gnss_parse_buf_; // ASCII/バイナリ混在の受信バッファ
    std::mutex gnss_write_mutex_;

    // RTK Client
    std::unique_ptr<boost::asio::ip::tcp::socket> rtk_socket_;
    std::unique_ptr<boost::asio::ip::tcp::resolver> rtk_resolver_;
    std::array<uint8_t, 4096> rtk_read_chunk_;
    std::vector<uint8_t> rtk_response_buffer_;
    bool rtk_response_header_received_{false};
    // The ROS timer and the Asio I/O thread both inspect RTK state.  Keep it
    // atomic so a retry cannot race an in-flight connection attempt.
    std::atomic_bool is_rtk_connected_{false};
    std::atomic_bool rtk_connection_in_progress_{false};

    // ROS Publishers
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr heading_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr feedback_odom_pub_;

    // ROS Subscribers & Timers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ctrl_sub_;
    rclcpp::TimerBase::SharedPtr rtk_gga_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hot_restart_srv_;
    rclcpp::TimerBase::SharedPtr hot_restart_reconfigure_timer_;

    // State
    std::string last_gpgga_;
    std::ofstream log_file_;
    bool stop_publish_;

    // GGA has the position while GST supplies its per-fix 1-sigma error.
    sensor_msgs::msg::NavSatFix pending_fix_;
    double pending_fix_utc_seconds_{0.0};
    bool have_pending_fix_{false};

    // State for UM982-only velocity feedback.  Position changes are converted
    // from local ENU to body-frame surge/sway using the dual-antenna heading.
    bool have_heading_{false};
    double latest_yaw_rad_{0.0};
    rclcpp::Time latest_heading_stamp_{0, 0, RCL_ROS_TIME};
    bool have_previous_fix_{false};
    double reference_latitude_rad_{0.0};
    double reference_longitude_rad_{0.0};
    double previous_east_m_{0.0};
    double previous_north_m_{0.0};
    rclcpp::Time previous_fix_stamp_{0, 0, RCL_ROS_TIME};
    double filtered_surge_mps_{0.0};
    double filtered_sway_mps_{0.0};
    double filtered_yaw_rate_rps_{0.0};
    double previous_yaw_rad_{0.0};
    bool have_previous_yaw_{false};
    bool have_gst_{false};
    rclcpp::Time latest_gst_stamp_{0, 0, RCL_ROS_TIME};
    std::array<double, 3> latest_gst_variance_{};
    std::atomic_bool hot_restart_in_progress_{false};
    
    // Parameters
    struct {
        std::string gnss_port;
        int gnss_baud;
        std::string tcp_ip;
        int tcp_port;
        std::string uart_or_tcp;
        
        int fix_freq;
        int heading_freq;
        int rtk_status_freq;
        std::string heading_log_format;
        std::string rtk_status_log_format;
        bool rtk_enable;
        std::string heading_frame_id;
        // Yaw of the UM982 primary -> secondary antenna baseline in base_link.
        // It is subtracted from UNIHEADING's baseline yaw to obtain vessel yaw.
        double heading_baseline_yaw_rad;
        std::string log_file_name;
        bool publish_feedback_odometry;
        std::string feedback_odometry_topic;
        std::string feedback_frame_id;
        std::string feedback_child_frame_id;
        double feedback_velocity_filter_alpha;
        double feedback_max_speed_mps;

        // RTK/NTRIP Settings
        std::string ntrip_server = "ntrip.ales-corp.co.jp";
        int ntrip_port = 2101;
        std::string mountpoint = "RTCM32MSM7";
        std::string username;
        std::string password;
    } params_;
};

} // namespace um982_driver

#endif // UM982_DRIVER_NODE_HPP_
