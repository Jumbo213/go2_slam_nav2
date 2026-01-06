from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():

    #获取各功能包
    go2_driver_pkg = get_package_share_directory("go2_driver")
    go2_core_pkg = get_package_share_directory("go2_core")
    go2_slam_pkg = get_package_share_directory("go2_slam")
    go2_perception_pkg = get_package_share_directory("go2_perception")
    go2_nav2_pkg = get_package_share_directory("go2_navigation2")
    
    # 添加启动开关
    use_nav2 = DeclareLaunchArgument(
        name="use_nav2",
        default_value="true"
    )

    use_slamtoolbox = DeclareLaunchArgument(
        name="use_slamtoolbox",
        default_value="false"
    )

    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false"
    )

    declare_map = DeclareLaunchArgument(
        name="map",
        default_value=""
    )

    declare_nav_params = DeclareLaunchArgument(
        name="params_file",
        default_value=os.path.join(go2_nav2_pkg, "config", "nav2_params.yaml")
    )

    declare_nav_slam = DeclareLaunchArgument(
        name="slam",
        default_value="false"
    )

    declare_nav_rviz = DeclareLaunchArgument(
        name="rviz",
        default_value="true"
    )

    declare_nav_namespace = DeclareLaunchArgument(
        name="namespace",
        default_value=""
    )

    # 里程计融合imu
    go2_robot_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_core_pkg, "launch", "go2_robot_localization.launch.py")
            )
        )

    # 启动驱动包
    go2_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_driver_pkg, "launch", "driver.launch.py")
        )   
    )

    # 点云处理
    go2_pointcloud_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_perception_pkg, "launch", "go2_pointcloud.launch.py")
            )
        )

    # slam-toolbox 配置
    go2_slamtoolbox_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_slam_pkg, "launch", "go2_slamtoolbox.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration('use_slamtoolbox'))
        )

    go2_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_nav2_pkg, "launch", "go2_nav2.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration('use_nav2')),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "slam": LaunchConfiguration("slam"),
            "rviz": LaunchConfiguration("rviz"),
            "namespace": LaunchConfiguration("namespace")
        }.items()
    )

    return LaunchDescription([
        go2_driver_launch,
        use_nav2,
        use_slamtoolbox,
        declare_use_sim_time,
        declare_map,
        declare_nav_params,
        declare_nav_slam,
        declare_nav_rviz,
        declare_nav_namespace,
        go2_robot_localization,
        go2_pointcloud_launch,
        go2_slamtoolbox_launch,
        go2_nav2_launch
    ])
