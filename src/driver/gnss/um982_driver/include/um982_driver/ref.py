"""
NavSatFixでGNSSデータを、Pose_with_CovarianceStamped/Pose/OrientationでUniheading(姿勢)を出す。

10個のパラメータを設定してください。
1. GNSS_SerialPort  (default = "/dev/ttyUSB0")
2. GNSS_Baudrate    (default = 115200)
3. FIX_FREQ         (default = 20)
4. HEADING_FREQ     (default = 20)
5. GNSS_RTK_Enable  (default = True)
6. Heading_FrameID  (default = "odom")
"""

from enum import Enum

import base64
import math
import socket
import numpy as np
import serial
from datetime import datetime


import rclpy

import tf_transformations
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

# from um982_driver_ros2j_v2.id_pass.id_pass import RTK_ID, RTK_PASS


class SolutuinStatus(Enum):
    SOL_COMPUTED = 0
    INSUFFICIENT_OBS = 1
    NO_CONVERGENCE = 2
    COV_TRACE = 4


class PositionOrVelocityType(Enum):
    # NONE = 0
    # FIXEDPOS = 1
    # FIXEDHEIGHT = 2
    # DOPPLER_VELOCITY = 8
    # SINGLE = 16
    # PSRDIFF = 17
    # SBAS = 18
    # L1_FLOAT = 32
    # IONOFREE_FLOAT = 33  # Ionosphere(電離層)-free float
    # NARROW_FLOAT = 34
    # L1_INT = 48
    # WIDE_INT = 49
    # NARROW_INT = 50
    # INS = 52
    # INS_PSRSP = 53  # good INS + Single Point Positioning
    # INS_PSRDIFF = 54  # good INS + PSRDIFF
    # INS_RTKFLOAT = 55
    # INS_RTKFIXED = 56
    # PPP_CONVERGING = 68
    # PPP = 69

    FIXED = 1
    INT = 2
    FLOAT = 3


class UM982Driver(Node):

    USERNAME = "username"
    PASSWORD = "password"

    # ROS funcs

    def __init__(self):
        super().__init__("um982_serial_ros2j_node")

        self.get_logger().info("Initializing UM982 Driver Node ...")

        self.declare_parameters_()

        # rtkサーバーに接続
        if self._RTK_enable:
            try:
                self.get_logger().info("connecting to rtk server ...")
                self.connect_to_RTKserver()

                self._is_rtk_server_connected = True
            except Exception as e:
                self.get_logger().error(
                    "\033[91m" + f"Failed to connect to server: {e}" + "\033[0m"
                )
                self._is_RTKserver_connected = False

            self.rtk_offset_timer = self.create_timer(5, self.rtk_jikoiti_send_callback)
        else:
            self.get_logger().info("RTK is disabled")

        # GNSSチップに接続
        try:
            self.connect_to_GNSS()  # この中でUSB SerialかTCPか判定・接続している
            self._is_GNSS_running = True
        except Exception as e:
            self.get_logger().error(
                "\033[91m" + f"Failed to open serial port: {e}" + "\033[0m"
            )
            self._is_GNSS_running = False

        # pub/sub/timer 設定
        self.fix_publisher = self.create_publisher(
            NavSatFix,
            "/sensor/vehicle_gnss/fix/raw",
            10,
        )

        self.fix_publisher_debug = self.create_publisher(
            NavSatFix,
            "/sensor/vehicle_gnss_debug/fix/raw",
            10,
        )

        self.heading_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "/sensor/vehicle_gnss/compass/raw",
            10,
        )

        self.heading_publisher_debug = self.create_publisher(
            PoseWithCovarianceStamped,
            "/sensor/vehicle_gnss_debug/compass/raw",
            10,
        )

        self.ctrl_sub = self.create_subscription(
            String,
            "/sensor/vehicle_gnss/command",
            self.ctrl_callback,
            10,
        )

        self.gnss_timer = self.create_timer(0.01, self.callback)

        if self._RTK_enable:
            self.rtk_offset_timer = self.create_timer(5, self.rtk_jikoiti_send_callback)
            self.rtk_recieve_timer = self.create_timer(1, self.rtk_callback)

        if self._is_GNSS_running:
            self.get_logger().info("\033[92m" + "Node Initialized" + "\033[0m")
        else:
            self.get_logger().error("\033[91m" + "Node NOT Initialized" + "\033[0m")

    def __del__(self):
        self.close()

    def declare_parameters_(self):
        # all member variables

        file_name = (
            "/home/oswystk/du_ws/bag_files/"
            + datetime.now().strftime("%Y%m%d_%H%M")
            + "/sensors/um982_driver.log"
        )
        self.declare_parameter(
            "log_file_name",
            file_name,
        )
        file_name = (
            self.get_parameter("log_file_name").get_parameter_value().string_value
        )

        self._log_file = None
        try:
            self._log_file = open(file_name, "w")
            self.get_logger().info("\033[92m" + "Log file opened" + "\033[0m")
        except Exception as e:
            self.get_logger().error(
                "\033[91m" + f"Failed to open log file: {e}" + "\033[0m"
            )

        self._is_RTKserver_connected = False
        self._ntrip_server = "ntrip.ales-corp.co.jp"
        self._ntrip_port = 2101
        self._mountpoint = "RTCM32MSM7"
        self._sock_RTK = None  # file discriptor

        self._is_GNSS_running = False
        self._port_GNSS = None  # file discriptor

        self._last_gpgga: str = None
        self._stop_publish_main = False  # True : stop publishing

        self.declare_parameter("uart_or_tcp", "tcp")
        self._uart_or_tcp = (
            self.get_parameter("uart_or_tcp").get_parameter_value().string_value
        )

        self.get_logger().info(f"uart_or_tcp: {self._uart_or_tcp}")

        self.declare_parameter("GNSS_SerialPort", "/dev/ttyUM982")
        self._serial_port_name = (
            self.get_parameter("GNSS_SerialPort").get_parameter_value().string_value
        )
        self.get_logger().info(f"Serial port name: {self._serial_port_name}")

        self.declare_parameter("GNSS_Baudrate", 115200)
        self._baudrate = (
            self.get_parameter("GNSS_Baudrate").get_parameter_value().integer_value
        )
        self.get_logger().info(f"Baudrate: {self._baudrate}")

        self.declare_parameter("tcp_ip", "um982.local")
        self._tcp_addr = self.get_parameter("tcp_ip").get_parameter_value().string_value
        self.get_logger().info(f"TCP IP address: {self._tcp_addr}")

        self.declare_parameter("tcp_port", 23)
        self._tcp_port = (
            self.get_parameter("tcp_port").get_parameter_value().integer_value
        )
        self.get_logger().info(f"TCP Port: {self._tcp_port}")

        self.declare_parameter("FIX_FREQ", 20)
        self.declare_parameter("HEADING_FREQ", 20)
        self.declare_parameter("GNSS_RTK_Enable", 1)
        self._RTK_enable = (
            self.get_parameter("GNSS_RTK_Enable").get_parameter_value().integer_value
        )

        self.declare_parameter("Heading_FrameID", "odom")

        self.get_logger().info("\033[92m" + "All Parameters declared" + "\033[0m")

    # decleare_Parameters

    def ctrl_callback(self, msg: String):
        if msg.data == "shutdown":
            self.get_logger().info("shutdown")
            self.close()
        elif msg.data == "stop_publish":
            self._stop_publish_main = True
        elif msg.data == "start_publish":
            self._stop_publish_main = False
        else:
            self.get_logger().info("unknown command")

    def rtk_callback(self):
        if self._is_RTKserver_connected:
            try:
                data = self._sock_RTK.recv(4096)
            except Exception as e:
                self.get_logger().error(
                    "\033[91m" + f"failed to receive hosei data: {e}" + "\033[91m"
                )
                return

            self.get_logger().debug(f"ntrip data length: {len(data)}")

            if not data:
                return

            self._port_GNSS.write(data)
        else:
            try:
                self.reconnect_to_RTKserver()
                self._is_RTKserver_connected = True
            except Exception as e:
                self.get_logger().error(
                    "\033[91m" + f"Failed to reconnect to server: {e}" + "\033[0m"
                )

    def rtk_jikoiti_send_callback(self):
        if self._is_RTKserver_connected and "GNGGA" in self._last_gpgga:
            self._sock_RTK.sendall(self._last_gpgga.encode("utf-8"))
        else:
            self.get_logger().warn("RTK server is not connected")
            self._is_RTKserver_connected = False

            try:
                self.reconnect_to_RTKserver()
                self._is_RTKserver_connected = True
            except Exception as e:
                self.get_logger().error(f"Failed to reconnect to server: {e}")

    def callback(self):
        """
        1. _is_GNSS_runningがTrueのとき、データを読み込む
        2. パースして、文頭によって処理を分岐(GNGGA, UNIHEADINGA, command, unknown)
        """
        if self._is_GNSS_running:
            data = self.read_data()
            # self.get_logger().info(data)

            if self._log_file:
                self._log_file.write(data + "\n")

            # 自己位置
            if data.startswith("$GNGGA"):
                # self.get_logger().info(f"processing GNGGA: {data}")
                try:
                    self.gga_pub(data)
                    self._last_gpgga = data
                except (ValueError, IndexError) as e:
                    self.get_logger().warn(
                        "\033[93m"
                        + f"Failed to parse NMEA sentence: {data}, Error: {e}"
                        + "\033[0m"
                    )

                    # 共分散が-1となるデータを送信、ではなくそもそも送らないように（−１でも使われてる説があるので）
                    # self.GGApub("$GNGGA,025754.00,0,N,0,E,0,0,0.7,63.3224,M,-9.7848,M,00,0000*58") # gps_qual=0

            # 3545.04473949,N,13927.78307637,E

            # 自己姿勢
            elif data.startswith("#UNIHEADINGA"):
                # self.get_logger().info(f"processing uniheading: {data}")
                try:
                    self.uniheading_pub(data)
                except (ValueError, IndexError) as e:
                    self.get_logger().warn(
                        f"Failed to parse UNIHEADINGA sentence: {data}, Error: {e}"
                    )

                    # 共分散が-1となるデータを送信
                    # self.UNIHEADINGpub(
                    #     '#UNIHEADINGA,65,GPS,FINE,2332,182526700,0,0,18,12;SOL_COMPUTED,NONE,0.0,123.0,0,0.0000,0.6422,1.1123,"999",39,31,31,16,3,01,3,f3*537aacb0'
                    # )

            elif data.startswith("#OBSVMA"):
                # self.get_logger().info(data)
                a = 0
            elif data.startswith("#OBSVHA"):
                # self.get_logger().info(data)
                a = 0
            elif data.startswith("#OBSVMCMPA"):
                # self.get_logger().info(data)
                a = 0

            elif data.startswith("#OBSVMCMPA"):
                # self.get_logger().info(data)
                a = 0

            # コマンドでunknown出ないように
            elif data.startswith("$command"):
                self.get_logger().info(data)

            # まぁまずないかと
            else:
                self.get_logger().info(f"unknown data format: {data}")
        else:
            self.get_logger().warn("Serial port is not readable")
            try:
                self.reconnect_to_GNSS()
                self._is_GNSS_running = True
            except Exception as e:
                self.get_logger().error(
                    "\033[93m" + f"Failed to reconnect to serial port: {e}" + "\033[0m"
                )

    #    ____        _     _ _     _                 _ _ _                _
    #   |  _ \ _   _| |__ | (_)___| |__     ___ __ _| | | |__   __ _  ___| | _____
    #   | |_) | | | | '_ \| | / __| '_ \   / __/ _` | | | '_ \ / _` |/ __| |/ / __|
    #   |  __/| |_| | |_) | | \__ \ | | | | (_| (_| | | | |_) | (_| | (__|   <\__ \
    #   |_|    \__,_|_.__/|_|_|___/_| |_|  \___\__,_|_|_|_.__/ \__,_|\___|_|\_\___/

    def gga_pub(self, _nmea_sentence: str):
        """
        http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html

        ex  $GNGGA,025754.00,35.69,         N,139.78,           E,0,0,0.7,63.3224,M,    -9.7848,M,00,0000*58
            $GNGGA,060945.65,3541.60532835, N,13947.12730873,   E,1,24,0.8,7.4235,M,    38.6277,M,,*75

        0. $GNGGA: ヘッダ
        1. 025754.00: UTC時刻
        2. 35.69: 緯度、この値は端折った東日本橋の緯度
        3. N: 北緯
        4. 139.78: 経度、この値は端折った東日本橋の経度
        5. E: 東経
        6. 0: GPS quality indicator。0=無効、1=単独、2=差分、3=PPS、4=RTK固定、5=RTK浮動、6=Dead Reckoning、7=手動入力、8=シミュレーション
        7. 0: Number of satellites in use
        8. 0.7: Horizontal dilution of precision
        9. 63.3224: MSL altitude
        10. M: 単位
        11. -9.7848: ジオイド高度
        12. M: 単位
        13. 00: DGPSデータの年数
        14. 0000: RTKサーバーのIDと思われるデータ
        15. *58: チェックサム
        """

        msg = _nmea_sentence.split(",")
        msg = (
            msg[0],
            float(msg[1]),
            float(msg[2]),
            msg[3],
            float(msg[4]),
            msg[5],
            int(msg[6]),
            int(msg[7]),
            float(msg[8]),
            float(msg[9]),
            msg[10],
            float(msg[11]),
            msg[12],
            msg[13],
            msg[14],
            # int(msg[13]),
            # int(msg[14].split('*')[0]),
            # int(msg[14].split('*')[1]), # *に意味あるのか？
        )

        # NavSatFixメッセージの生成
        navsatfix_msg = NavSatFix()

        # header
        navsatfix_msg.header.stamp = self.get_clock().now().to_msg()
        navsatfix_msg.header.frame_id = "base_link"

        navsatfix_msg.status.service = 15  # 全種類のGNSSを使っている

        # lat, lon (度数に変換）
        navsatfix_msg.latitude = self.convert_NMEA2latlon(msg[2], msg[3])
        navsatfix_msg.longitude = self.convert_NMEA2latlon(msg[4], msg[5])

        # altitude
        altitude = float(msg[9]) if msg[10] == "M" else 0.0  # 要検証
        geoid_height = float(msg[11]) if msg[12] == "M" else 0.0  # 要検証

        navsatfix_msg.altitude = altitude - geoid_height

        if self._is_GNSS_running and msg[6] >= 1:
            navsatfix_msg.position_covariance[0] = (0.02) ** 2
            navsatfix_msg.position_covariance[4] = (0.02) ** 2
            navsatfix_msg.position_covariance[8] = (0.02) ** 2
            navsatfix_msg.position_covariance_type = (
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            )

            # NavSatFixメッセージをパブリッシュ
            if not self._stop_publish_main:
                self.fix_publisher.publish(navsatfix_msg)
            else:
                self.get_logger().warn("\033[93m" + "GNSS disconnected?" + "\033[0m")
            self.fix_publisher.publish(navsatfix_msg)
            # self.get_logger().info(f'Published NavSatFix: {navsatfix_msg}')
            # self.get_logger().info("Published NavSatFix")

    def uniheading_pub(self, _data: str):
        """
        https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseWithCovarianceStamped.html

        UNIHEADINGA
        #UNIHEADINGA,65,GPS,FINE,2332,182526700,0,0,18,12;SOL_COMPUTED,NONE,0.0,123.0,0,0.0000,0.6422,1.1123,\"999\",39,31,31,16,3,01,3,f3*537aacb0

        UNIHEADINGA,65,GPS,FINE,2332,182526700,0,0,18,12; UNIHEADINGAのヘッダ
        0. SOL_COMPUTED: solution status
        1. NONE: position or velpcity type, NONE -> 衛星との接続なし, NARROW_INT -> よい NARROW_FLOAT -> ちょいいい
        2. 0.0: base line length (m)
        3. 123.0: heading (0~360deg)
        4. 0: pitch (+- 90deg)
        5. 0.0000: reserved
        6. 0.6422: heading standard deviation (deg)
        7. 1.1123: pitch standard deviation (deg)
        8. "999": base station ID
        9. 39: number of satellites
        10. 31: number of satellites used in solution
        11. 31: Number of satellites above theelevation mask angle
        12. 16: Number of satellites with L2 signal above the elevation mask angle
        13. 3: reserved
        14. 01: Extended solution status, refer to Table 7-87 Extended Solution Status
        15. 3: Galileo and BDS-3 signal mask, refer to Table 7-86 Galileo & BDS-3 Signal Mask
        16. f3: GPS, GLONASS and BDS-2 signal mask (see Table 7-85 GPS/GLONASS/BDS-2 Signal Mask
        17. 537aacb0: checksum

        """
        # self.get_logger().info(_data)

        headeranddata = _data.split(";")
        datalist = headeranddata[1].split(",")

        # connection check
        state: SolutuinStatus = -1
        if datalist[0] == "SOL_COMPUTED":
            state = SolutuinStatus.SOL_COMPUTED
        elif datalist[0] == "COV_TRACE":
            state = SolutuinStatus.COV_TRACE

        quality: int = -1
        if datalist[1] == "NARROW_INT" or datalist[1] == "INS_RTKFIXED":
            quality = PositionOrVelocityType.FIXED
        elif datalist[1] == "WIDE_INT" or datalist[1] == "L1_INT":
            quality = PositionOrVelocityType.INT
        elif (
            datalist[1] == "NARROW_FLOAT"
            or datalist[1] == "IONOFREE_FLOAT"
            or datalist[1] == "L1_FLOAT"
        ):
            quality = PositionOrVelocityType.FLOAT
        else:
            quality = -1

        # msg
        msg = PoseWithCovarianceStamped()

        # header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = (
            self.get_parameter("Heading_FrameID").get_parameter_value().string_value
        )

        if self._is_GNSS_running:
            # 中身
            # yaw and pitchに関してdeg 2 rad
            yaw_rad = math.radians(90.0 - float(datalist[3]))
            pitch_rad = math.radians(-1.0 * float(datalist[4]))

            # クォータニオンに変換（Roll, Pitch, Yaw の回転を表す）
            quat = tf_transformations.quaternion_from_euler(0, pitch_rad, yaw_rad)

            # クォータニオンをメッセージに設定
            msg.pose.pose.orientation.x = quat[0]
            msg.pose.pose.orientation.y = quat[1]
            msg.pose.pose.orientation.z = quat[2]
            msg.pose.pose.orientation.w = quat[3]

            if (
                state == SolutuinStatus.SOL_COMPUTED
                and quality == PositionOrVelocityType.FIXED
            ):
                msg.pose.covariance = np.zeros(36)
                msg.pose.covariance[28] = (0.5 * math.pi / 180) ** 2
                msg.pose.covariance[35] = (0.5 * math.pi / 180) ** 2

                if not self._stop_publish_main:
                    self.heading_publisher.publish(msg)
                self.heading_publisher.publish(msg)
                # self.get_logger().info(f'Published Quaternion: {msg}')
                # self.get_logger().info("Published Quaternion")

            # elif (
            #     state == SolutuinStatus.SOL_COMPUTED
            #     and quality == PositionOrVelocityType.INT
            # ):
            #     msg.pose.covariance = np.zeros(36)
            #     msg.pose.covariance[28] = (100 * math.pi / 180) ** 2
            #     msg.pose.covariance[35] = (100 * math.pi / 180) ** 2
            # elif (
            #     state == SolutuinStatus.SOL_COMPUTED
            #     and quality == PositionOrVelocityType.FLOAT
            # ):
            #     msg.pose.covariance = np.zeros(36)
            #     msg.pose.covariance[28] = (360 * math.pi / 180) ** 2
            #     msg.pose.covariance[35] = (360 * math.pi / 180) ** 2

            # elif state == SolutuinStatus.COV_TRACE:
            #     msg.pose.covariance = np.zeros(36)
            #     msg.pose.covariance[28] = (10e6 * math.pi / 180) ** 2
            #     msg.pose.covariance[35] = (10e6 * math.pi / 180) ** 2
            # else:
            #     msg.pose.covariance = [-1 for _ in range(36)]
        else:
            self.get_logger().warn("\033[93m" + "GNSS disconnected?" + "\033[0m")
            msg.pose.covariance = [-1 for _ in range(36)]

    def convert_NMEA2latlon(self, value: str, direction: str) -> float:
        """
        Converts NMEA format latitude/longitude to decimal degrees for NavSatFix.
        NMEAフォーマットの緯度/経度表示を、NavSatFix用度数に変換する
        Args:
            value (str): The NMEA format latitude or longitude value.
            direction (str): The direction character ('N', 'S', 'E', 'W').
        Returns:
            float: The converted latitude or longitude in decimal degrees.
        """

        if not value:
            return 0.0

        degrees = float(value) // 100
        minutes = float(value) % 100
        decimal = degrees + minutes / 60.0

        if direction in ["S", "W"]:
            decimal = -decimal

        return decimal

    def read_data(self) -> str:
        try:
            data = self._port_GNSS.readline().decode("ascii", errors="replace").strip()
        except serial.SerialException as e:
            self.get_logger().error(
                "\033[93m" + f"Failed to read data: {e}" + "\033[0m"
            )
            self._is_GNSS_running = False

        return data

    #     ____                            _   _                __
    #    / ___|___  _ __  _ __   ___  ___| |_(_) ___  _ __    / _|_   _ _ __   ___ ___
    #   | |   / _ \| '_ \| '_ \ / _ \/ __| __| |/ _ \| '_ \  | |_| | | | '_ \ / __/ __|
    #   | |__| (_) | | | | | | |  __/ (__| |_| | (_) | | | | |  _| |_| | | | | (__\__ \
    #    \____\___/|_| |_|_| |_|\___|\___|\__|_|\___/|_| |_| |_|  \__,_|_| |_|\___|___/

    def connect_to_RTKserver(self):
        self._sock_RTK = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock_RTK.settimeout(0.2)

        self._sock_RTK.connect((self._ntrip_server, self._ntrip_port))

        credentials = base64.b64encode(
            f"{self.USERNAME}:{self.PASSWORD}".encode("utf-8")
        ).decode("utf-8")

        request = (
            f"GET /{self._mountpoint} HTTP/1.1\r\n"
            f"Host: {self._ntrip_server}:{self._ntrip_port}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"User-Agent: FullDepth Ntrip Client/1.0\r\n"
            f"Authorization: Basic {credentials}\r\n"
            f"Connection: close\r\n"
            f"\r\n"
        )
        # self.get_logger().info(request)

        self._sock_RTK.sendall(request.encode("utf-8"))
        response = self._sock_RTK.recv(4096)
        self.get_logger().info(response)

        if b"ICY 200 OK" not in response:
            raise Exception("Failed to connect to Ntrip server")
        position_request = f"$GNGGA,020330.00,3503.07556378,N,13853.74305596,E,5,42,0.5,2.0067,M,40.4042,M,5.0,165*60\n"  # TODO(osw)ここはなにか方法を考えたい
        self._last_gpgga = position_request
        self._sock_RTK.sendall(
            position_request.encode("utf-8")
        )  # 本当は５秒毎に最新を送らないといけない→rtk_jikoiti_send_callbackでやる

        self.get_logger().info("\033[32m" + "RTK server connected" + "\033[0m]")

    def connect_to_GNSS(self):
        if self._uart_or_tcp == "uart":
            self._port_GNSS = serial.Serial(
                self._serial_port_name, self._baudrate, timeout=1
            )
        elif self._uart_or_tcp == "tcp":
            sock_GNSS = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock_GNSS.connect((self._tcp_addr, self._tcp_port))
            self._port_GNSS = sock_GNSS.makefile("rwb")
        else:
            raise Exception("Invalid connection type")

        if not self._port_GNSS.is_open:
            raise Exception("Failed to open serial connection")

        # 周期設定
        fix_period = (
            1 / self.get_parameter("FIX_FREQ").get_parameter_value().integer_value
        )
        fix_period = round(fix_period, 2)

        heading_period = (
            1 / self.get_parameter("HEADING_FREQ").get_parameter_value().integer_value
        )
        heading_period = round(heading_period, 2)

        # 送信
        self._port_GNSS.write(
            # f"OBSVMA {self._serial_port_name} {str(fix_period)}\r\n".encode()
            f"OBSVMA {str(fix_period)}\r\n".encode()
        )
        # self._port_GNSS.write(
        #     # f"OBSVMCMPA  {self._serial_port_name} {str(heading_period)}\r\n".encode()
        #     f"OBSVMCMPA {str(heading_period)}\r\n".encode()
        # )
        self._port_GNSS.write(f"GPGGA {str(fix_period)}\r\n".encode())
        self._port_GNSS.write(f"UNIHEADINGA {str(heading_period)}\r\n".encode())

        self.get_logger().info("GNSS connected")

    def reconnect_to_RTKserver(self):
        """
        call in try-except block
        """

        self.get_logger().info("Attempting to reconnect to RTK ...")

        sleeper = self.create_rate(0.2)  # 0.2Hz = 5s
        sleeper.sleep()

        self.connect_to_RTKserver()
        self.get_logger().info("Reconnected to RTK.")

    def reconnect_to_GNSS(self):
        """
        call in try-except block
        """

        self.get_logger().info("Attempting to reconnect to serial port...")

        sleeper = self.create_rate(0.2)  # 0.2Hz = 5s
        sleeper.sleep()

        self.connect_to_GNSS()
        self.get_logger().info("Reconnected to serial port.")

    def close(self):
        self._is_GNSS_running = False

        if self._log_file:
            self._log_file.close()

        if self._sock_RTK:
            self._sock_RTK.close()
        if self._port_GNSS and self._port_GNSS.is_open:
            self._port_GNSS.close()


########################################################################################


def main(args=None):
    rclpy.init(args=args)
    node = UM982Driver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
