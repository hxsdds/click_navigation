import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory("uav_dog_sim")
    description_share = get_package_share_directory("go2_description")

    world_name = LaunchConfiguration("world").perform(context)
    world_path = LaunchConfiguration("world_file").perform(context) or os.path.join(
        package_share, "worlds", f"{world_name}.world")
    if not os.path.isfile(world_path):
        raise RuntimeError(f"World file does not exist: {world_path}")

    external_sensors = LaunchConfiguration("external_sensors").perform(context).lower()
    camera_sensors = LaunchConfiguration("camera_sensors").perform(context).lower()
    lidar3d_sensors = LaunchConfiguration("lidar3d_sensors").perform(context).lower()
    robot_description = xacro.process_file(
        os.path.join(description_share, "xacro", "robot.xacro"),
        mappings={
            "GAZEBO": "true",
            "EXTERNAL_SENSORS": external_sensors,
            "CAMERA_SENSORS": camera_sensors,
            "LIDAR3D_SENSORS": lidar3d_sensors,
        },
    ).toxml()

    use_sim_time = {"use_sim_time": True}

    if LaunchConfiguration("gui").perform(context).lower() == "true":
        gazebo_args = "-r -v 4"
    else:
        # Server-only mode still needs a rendering context: without
        # --headless-rendering the depth camera / depth image comes out empty
        # (rgb renders, depth is all inf).
        gazebo_args = "-s -r -v 4 --headless-rendering"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": [gazebo_args, " ", world_path]}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[use_sim_time, {"robot_description": robot_description}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_go2",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "go2",
            "-allow_renaming", "false",
            "-x", LaunchConfiguration("start_x"),
            "-y", LaunchConfiguration("start_y"),
            "-z", LaunchConfiguration("start_z"),
            "-Y", LaunchConfiguration("start_yaw"),
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60.0",
        ],
        parameters=[use_sim_time],
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_imu_sensor_broadcaster",
        output="screen",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60.0",
        ],
        parameters=[use_sim_time],
    )

    unitree_guide_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_unitree_guide_controller",
        output="screen",
        arguments=[
            "unitree_guide_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60.0",
        ],
        parameters=[use_sim_time],
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[use_sim_time],
    )

    lidar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="go2_lidar_bridge",
        output="screen",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        parameters=[use_sim_time],
    )

    camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="go2_camera_info_bridge",
        output="screen",
        arguments=[
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            # Depth camera info: rtabmap needs it to build the 3D cloud.
            "/rgbd_d435/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        parameters=[use_sim_time],
        condition=IfCondition(LaunchConfiguration("camera_sensors")),
    )


    lidar3d_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="go2_lidar3d_bridge",
        output="screen",
        arguments=[
            "/lidar3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        parameters=[use_sim_time],
        condition=IfCondition(LaunchConfiguration("lidar3d_sensors")),
    )

    odometry_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="go2_odometry_bridge",
        output="screen",
        arguments=[
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        parameters=[use_sim_time],
    )

    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="go2_image_bridge",
        output="screen",
        arguments=["/camera/image", "/rgbd_d435/image", "/rgbd_d435/depth_image"],
        parameters=[use_sim_time],
        condition=IfCondition(LaunchConfiguration("camera_sensors")),
    )

    external_sensor_bridges = GroupAction(
        actions=[lidar_bridge, lidar3d_bridge, camera_info_bridge, image_bridge],
        condition=IfCondition(LaunchConfiguration("external_sensors")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=[
            "-d",
            os.path.join(description_share, "config", "visualize_urdf.rviz"),
        ],
        parameters=[use_sim_time],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    keyboard = Node(
        package="keyboard_input",
        executable="keyboard_input",
        name="keyboard_input",
        output="screen",
        condition=IfCondition(LaunchConfiguration("keyboard")),
    )

    return [
        SetEnvironmentVariable("GZ_SIM_SERVER_CONFIG_PATH", LaunchConfiguration("server_config")),
        gazebo,
        ros_gz_bridge,
        robot_state_publisher,
        spawn_robot,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[TimerAction(period=12.0, actions=[joint_state_broadcaster])],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=joint_state_broadcaster, on_exit=[imu_sensor_broadcaster])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=imu_sensor_broadcaster, on_exit=[unitree_guide_controller])
        ),
        external_sensor_bridges,
        odometry_bridge,
        rviz,
        keyboard,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("server_config", default_value=os.path.join(
            get_package_share_directory("uav_dog_sim"), "config", "gz_server.config")),
        DeclareLaunchArgument("world_file", default_value="",
                              description="Optional explicit world file; empty uses installed world."),
        DeclareLaunchArgument(
            "world",
            default_value="chemical_plant_v4",
            description="Name of a world file installed in uav_dog_sim/worlds.",
        ),
        # The factory world spawns the robot on the main east-west road
        # (the corridor between y = -10.5 and y = -1.5) so the first manual
        # motion and the SLAM pass start from a clear, reproducible spot.
        DeclareLaunchArgument("start_x", default_value="-4.0"),
        DeclareLaunchArgument("start_y", default_value="-6.0"),
        DeclareLaunchArgument("start_z", default_value="0.5"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start the Gazebo GUI. Set false for WSL2/headless servers.",
        ),
        DeclareLaunchArgument(
            "external_sensors",
            default_value="false",
            description="Enable the Go2 lidar and optional camera models and bridges.",
        ),
        DeclareLaunchArgument(
            "lidar3d_sensors",
            default_value="false",
            description="Add the 8-channel lidar and bridge /lidar3d/points for 3D mapping.",
        ),
        DeclareLaunchArgument(
            "camera_sensors",
            default_value="true",
            description="Enable the camera/RGBD sensors when external_sensors is true.",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "keyboard",
            default_value="false",
            description="Start keyboard_input in this terminal.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
