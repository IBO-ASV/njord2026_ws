from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- 引数の宣言 (デフォルト値はC++コードに合わせつつ、ここで変更可能にする) ---
    # 通信モード: 'uart' or 'tcp'
    arg_mode = DeclareLaunchArgument(
        'uart_or_tcp',
        default_value='uart',
        description='Connection mode: "uart" or "tcp"'
    )

    # シリアル接続設定
    arg_port = DeclareLaunchArgument(
        'gnss_port',
        default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
        description='Stable /dev/serial/by-id path for the UM982 serial port'
    )
    arg_baud = DeclareLaunchArgument(
        'gnss_baudrate',
        default_value='115200',
        description='Baudrate for serial connection'
    )

    # TCP接続設定
    arg_ip = DeclareLaunchArgument(
        'tcp_ip',
        default_value='192.168.0.126',
        description='IP address for TCP connection'
    )
    arg_tcp_port = DeclareLaunchArgument(
        'tcp_port',
        default_value='23',
        description='Port number for TCP connection'
    )

    # GNSS設定
    arg_fix_freq = DeclareLaunchArgument(
        'fix_freq',
        default_value='20',
        description='Frequency for NavSatFix publishing (Hz)'
    )
    arg_heading_freq = DeclareLaunchArgument(
        'heading_freq',
        default_value='20',
        description='Frequency for Heading publishing (Hz)'
    )
    arg_rtk_status_freq = DeclareLaunchArgument(
        'rtk_status_freq',
        default_value='1',
        description='Frequency for RTKSTATUS diagnostic output (Hz)'
    )
    arg_uniheading_format = DeclareLaunchArgument(
        'uniheading_format',
        default_value='B',
        choices=['A', 'B'],
        description='UM982 UNIHEADING output format: ASCII (A) or binary (B)'
    )
    arg_rtk_status_format = DeclareLaunchArgument(
        'rtk_status_format',
        default_value='B',
        choices=['A', 'B'],
        description='UM982 RTKSTATUS output format: ASCII (A) or binary (B)'
    )
    arg_rtk = DeclareLaunchArgument(
        'rtk_enable',
        default_value='true',
        description='Enable RTK (NTRIP client)'
    )
    arg_ntrip_server = DeclareLaunchArgument(
        'ntrip_server',
        default_value='osw-Stealth-14-AI-Studio-A1VGG.local',
        description='NTRIP caster hostname'
    )
    arg_ntrip_port = DeclareLaunchArgument(
        'ntrip_port',
        default_value='2101',
        description='NTRIP caster port'
    )
    arg_ntrip_mountpoint = DeclareLaunchArgument(
        'ntrip_mountpoint',
        default_value='RTCM3',
        description='NTRIP mountpoint'
    )
    arg_ntrip_username = DeclareLaunchArgument(
        'ntrip_username',
        default_value='test',
        description='NTRIP username (leave empty for anonymous casters)'
    )
    arg_ntrip_password = DeclareLaunchArgument(
        'ntrip_password',
        default_value='test',
        description='NTRIP password (leave empty for anonymous casters)'
    )
    arg_frame_id = DeclareLaunchArgument(
        'heading_frame_id',
        default_value='map',
        description='World frame for the absolute ENU heading message'
    )
    arg_heading_baseline_yaw = DeclareLaunchArgument(
        'heading_baseline_yaw_rad',
        # Primary: gnss_link=(-0.5, +0.4); secondary:
        # gnss_sub_link=(+0.5, -0.4), in robot.urdf.xacro.
        # atan2(-0.8, +1.0): UM982 primary -> secondary baseline in base_link.
        default_value='-0.6747409422235526',
        description=(
            'UM982 primary-to-secondary antenna baseline yaw in base_link '
            '[rad]; subtracted from UNIHEADING to obtain vessel yaw'
        )
    )
    arg_publish_feedback_odometry = DeclareLaunchArgument(
        'publish_feedback_odometry',
        default_value='false',
        description='Publish UM982-derived local odometry for the feedback filter'
    )
    arg_feedback_frame_id = DeclareLaunchArgument(
        'feedback_frame_id',
        default_value='odom',
        description='Parent frame of the local-ENU feedback odometry'
    )
    arg_feedback_child_frame_id = DeclareLaunchArgument(
        'feedback_child_frame_id',
        default_value='base_link',
        description='Child frame of the local-ENU feedback odometry'
    )

    # ログ設定
    arg_log = DeclareLaunchArgument(
        'log_file_name',
        default_value='',
        description='Path to save raw GNSS log (leave empty to disable)'
    )

    # --- ノードの定義 ---
    um982_node = Node(
        package='um982_driver',
        executable='um982_driver_node',
        name='um982_driver',
        output='screen',
        emulate_tty=True,
        parameters=[{
            # C++側のパラメータ名 : LaunchConfiguration(引数名)
            'uart_or_tcp': LaunchConfiguration('uart_or_tcp'),
            'GNSS_SerialPort': LaunchConfiguration('gnss_port'),
            'GNSS_Baudrate': LaunchConfiguration('gnss_baudrate'),
            'tcp_ip': LaunchConfiguration('tcp_ip'),
            'tcp_port': LaunchConfiguration('tcp_port'),
            'FIX_FREQ': LaunchConfiguration('fix_freq'),
            'HEADING_FREQ': LaunchConfiguration('heading_freq'),
            'RTK_STATUS_FREQ': LaunchConfiguration('rtk_status_freq'),
            'UNIHEADING_FORMAT': LaunchConfiguration('uniheading_format'),
            'RTK_STATUS_FORMAT': LaunchConfiguration('rtk_status_format'),
            'GNSS_RTK_Enable': LaunchConfiguration('rtk_enable'),
            'NTRIP_Server': LaunchConfiguration('ntrip_server'),
            'NTRIP_Port': LaunchConfiguration('ntrip_port'),
            'NTRIP_Mountpoint': LaunchConfiguration('ntrip_mountpoint'),
            'NTRIP_Username': LaunchConfiguration('ntrip_username'),
            'NTRIP_Password': LaunchConfiguration('ntrip_password'),
            'Heading_FrameID': LaunchConfiguration('heading_frame_id'),
            'heading_baseline_yaw_rad': LaunchConfiguration('heading_baseline_yaw_rad'),
            'publish_feedback_odometry': LaunchConfiguration('publish_feedback_odometry'),
            'feedback_frame_id': LaunchConfiguration('feedback_frame_id'),
            'feedback_child_frame_id': LaunchConfiguration('feedback_child_frame_id'),
            'log_file_name': LaunchConfiguration('log_file_name'),
        }]
    )

    return LaunchDescription([
        arg_mode,
        arg_port,
        arg_baud,
        arg_ip,
        arg_tcp_port,
        arg_fix_freq,
        arg_heading_freq,
        arg_rtk_status_freq,
        arg_uniheading_format,
        arg_rtk_status_format,
        arg_rtk,
        arg_ntrip_server,
        arg_ntrip_port,
        arg_ntrip_mountpoint,
        arg_ntrip_username,
        arg_ntrip_password,
        arg_frame_id,
        arg_heading_baseline_yaw,
        arg_publish_feedback_odometry,
        arg_feedback_frame_id,
        arg_feedback_child_frame_id,
        arg_log,
        um982_node
    ])
