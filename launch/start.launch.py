from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch
import os

from launch.substitutions import FindExecutable, Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchContext
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('faruk')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='True',
                              description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=pkg_share + "/models/robot.urdf",
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=pkg_share + "/rviz/robot_config.rviz",
                              description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                              description='Flag to enable use_sim_time'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(
                ['xacro ', LaunchConfiguration('model')])}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=launch.conditions.UnlessCondition(
                LaunchConfiguration('gui'))
        ),
        # ExecuteProcess(
        #     cmd=[FindExecutable(name='cheese'), "-w"],
        #     output='screen',
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': pkg_share +
                              "/world/world.sdf"}.items(),
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=['-world', 'test_world', '-topic',
                       'robot_description', '-z', '0.15'],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/world/test_world/model/minik/link/base_link/sensor/depth_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/world/test_world/model/minik/link/base_link/sensor/depth_camera/depth_image/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                # # Clock (IGN -> ROS2)
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                # Joint states (IGN -> ROS2)
                '/world/test_world/model/minik/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',

                '/demo/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                '/model/minik/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',

                '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                '/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            ],
            remappings=[
                ('/world/test_world/model/minik/joint_state', 'joint_states'),
                ('/model/minik/odometry', '/demo/odom'),
                ('/world/test_world/model/minik/link/base_link/sensor/depth_camera/depth_image',
                 '/depth_camera/image_raw'),
                ('/world/test_world/model/minik/link/base_link/sensor/depth_camera/depth_image/points',
                 '/depth_camera/points'),

            ],
            output='screen'
        ),
        Node(
            package='faruk',
            executable='cpp_robot_controller',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config/ekf.yaml'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            output='screen',
        ),
    ])
