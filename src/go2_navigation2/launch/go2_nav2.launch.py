from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_params = {
        # Controller Server
        # 使用更简单的RPP控制器
        'controller_server': {
            'use_sim_time': use_sim_time,
            'controller_frequency': 20.0,
            'min_x_velocity_threshold': 0.001,
            'min_y_velocity_threshold': 0.5,
            'min_theta_velocity_threshold': 0.001,
            'failure_tolerance': 0.3,
            'progress_checker_plugin': "progress_checker",
            'goal_checker_plugins': ["general_goal_checker"],
            'controller_plugins': ["FollowPath"],

            'progress_checker': {
                'plugin': "nav2_controller::SimpleProgressChecker",
                'required_movement_radius': 0.5,
                'movement_time_allowance': 10.0
            },

            'general_goal_checker': {
                'stateful': True,
                'plugin': "nav2_controller::SimpleGoalChecker",
                'xy_goal_tolerance': 0.25,
                'yaw_goal_tolerance': 0.25
            },

            'FollowPath': {
                'plugin': "nav2_mppi_controller::MPPIController",
                "time_steps": 56,
                "model_dt": 0.05,
                "batch_size": 2000,
                "vx_std": 0.2,
                "vy_std": 0.2,
                "wz_std": 0.4,
                "vx_max": 0.5,
                "vx_min": -0.35,
                "vy_max": 0.5,
                "wz_max": 1.9,
                "iteration_count": 1,
                "prune_distance": 1.7,
                "transform_tolerance": 0.1,
                "temperature": 0.3,
                "gamma": 0.015,
                "motion_model": "DiffDrive",
                "visualize": False,
                "reset_period": 1.0,  # (only in Humble)
                "TrajectoryVisualizer": {
                    "trajectory_step": 5,
                    "time_step": 3,
                },
                "AckermannConstrains": {
                    "min_turning_r": 0.2
                },
                "critics":
                    ["ConstraintCritic",
                     "ObstaclesCritic",
                     "GoalCritic",
                     "GoalAngleCritic",
                     "PathAlignCritic",
                     "PathFollowCritic",
                     "PathAngleCritic",
                     "PreferForwardCritic"
                     ],
                "ConstraintCritic": {
                    "enabled": True,
                    "cost_power": 1,
                    "cost_weight": 4.0
                },
                "GoalCritic": {
                    "enabled": True,
                    "cost_power": 1,
                    "cost_weight": 5.0,
                    "threshold_to_consider": 1.0
                },
                "GoalAngleCritic": {
                    "enabled": True,
                    "cost_power": 1,
                    "cost_weight": 3.0,
                    "threshold_to_consider": 0.4
                },
                "PreferForwardCritic": {
                    "enabled": True,
                    "cost_power": 1,
                    "cost_weight": 5.0,
                    "threshold_to_consider": 0.4
                },
                "ObstaclesCritic": {
                    "enabled": True,
                    "cost_power": 1,
                    "repulsion_weight": 1.5,
                    "critical_weight": 20.0,
                    "consider_footprint": False,
                    "collision_cost": 10000.0,
                    "collision_margin_distance": 0.1,
                    "near_goal_distance": 0.5,
                    "inflation_radius": 0.55,
                    "cost_scaling_factor": 10.0
                },
                "PathAlignCritic": {
                    "enabled": True,
                    "cost_power": 1,
                    "cost_weight": 14.0,
                    "max_path_occupancy_ratio": 0.05,
                    "trajectory_point_step": 3,
                    "threshold_to_consider": 0.40,
                    "offset_from_furthest": 20
                },
                "PathFollowCritic": {
                    "enabled": True,
                    "cost_power": 1,
                    "cost_weight": 5.0,
                    "offset_from_furthest": 5,
                    "threshold_to_consider": 0.6
                },
                "PathAngleCritic": {
                    "enabled": True,
                    "cost_power": 1,
                    "cost_weight": 2.0,
                    "offset_from_furthest": 4,
                    "threshold_to_consider": 0.40,
                    "max_angle_to_furthest": 1.0
                }
            }
        },
        # Planner Server
        'planner_server': {
            'expected_planner_frequency': 20.0,
            'use_sim_time': use_sim_time,
            'planner_plugins': ["GridBased"],
            'GridBased': {
                'plugin': "nav2_navfn_planner/NavfnPlanner",
                'tolerance': 0.5,
                'use_astar': False,
                'allow_unknown': True
            }
        },

        # Behavior Server
        # 替换原来的recoveries_server
        'behavior_server': {
            'costmap_topic': 'local_costmap/costmap_raw',
            'footprint_topic': 'local_costmap/published_footprint',
            'cycle_frequency': 10.0,
            'behavior_plugins': ["spin", "backup", "drive_on_heading", "wait"],
            'spin': {
                'plugin': "nav2_behaviors/Spin"
            },
            'backup': {
                'plugin': "nav2_behaviors/BackUp"
            },
            'drive_on_heading': {
                'plugin': "nav2_behaviors/DriveOnHeading"
            },
            'wait': {
                'plugin': "nav2_behaviors/Wait"
            },
            'global_frame': 'odom',
            'robot_base_frame': 'base_link',
            'transform_timeout': 0.1,
            'use_sim_time': use_sim_time,
            'simulate_ahead_time': 2.0,
            'max_rotational_vel': 0.4,
            'min_rotational_vel': 0.4,
            'rotational_acc_lim': 3.2
        },

        # BT Navigator
        # 修改为Humble兼容版本
        'bt_navigator': {
            'use_sim_time': use_sim_time,
            'global_frame': 'map',
            'robot_base_frame': 'base_link',
            # 'odom_topic': '/odom',
            'odom_topic': '/odom',
            'enable_groot_monitoring': True,
            'groot_zmq_publisher_port': 1666,
            'groot_zmq_server_port': 1667,
            # 使用自定义行为树文件
            # 'default_nav_to_pose_bt_xml': bt_xml_path,
            # 'default_nav_through_poses_bt_xml': bt_xml_path,
            'always_reload_bt_xml': False,
            'goal_blackboard_id': 'goal',
            'goals_blackboard_id': 'goals',
            'path_blackboard_id': 'path',
            'waypoint_statuses_blackboard_id': 'waypoint_statuses',

            # 添加navigator配置
            'navigators': ['navigate_to_pose', 'navigate_through_poses'],
            'navigate_to_pose': {
                'plugin': 'nav2_bt_navigator::NavigateToPoseNavigator'
            },
            'navigate_through_poses': {
                'plugin': 'nav2_bt_navigator::NavigateThroughPosesNavigator'
            },
            'plugin_lib_names': [
                'nav2_compute_path_to_pose_action_bt_node',
                'nav2_compute_path_through_poses_action_bt_node',
                'nav2_follow_path_action_bt_node',
                'nav2_smooth_path_action_bt_node',
                'nav2_back_up_action_bt_node',
                'nav2_spin_action_bt_node',
                'nav2_wait_action_bt_node',
                'nav2_clear_costmap_service_bt_node',
                'nav2_is_stuck_condition_bt_node',
                'nav2_goal_reached_condition_bt_node',
                'nav2_goal_updated_condition_bt_node',
                'nav2_globally_updated_goal_condition_bt_node',
                'nav2_is_path_valid_condition_bt_node',
                'nav2_initial_pose_received_condition_bt_node',
                'nav2_reinitialize_global_localization_service_bt_node',
                'nav2_rate_controller_bt_node',
                'nav2_distance_controller_bt_node',
                'nav2_speed_controller_bt_node',
                'nav2_truncate_path_action_bt_node',
                'nav2_truncate_path_local_action_bt_node',
                'nav2_goal_updater_node_bt_node',
                'nav2_recovery_node_bt_node',
                'nav2_pipeline_sequence_bt_node',
                'nav2_round_robin_node_bt_node',
                'nav2_transform_available_condition_bt_node',
                'nav2_time_expired_condition_bt_node',
                'nav2_path_expiring_timer_condition',
                'nav2_distance_traveled_condition_bt_node',
                'nav2_single_trigger_bt_node',
                'nav2_is_battery_low_condition_bt_node',
                'nav2_navigate_through_poses_action_bt_node',
                'nav2_navigate_to_pose_action_bt_node',
                'nav2_remove_passed_goals_action_bt_node',
                'nav2_planner_selector_bt_node',
                'nav2_controller_selector_bt_node',
                'nav2_goal_checker_selector_bt_node'
            ]
        },

        # Waypoint Follower
        'waypoint_follower': {
            'loop_rate': 20,
            'stop_on_failure': False,
            'waypoint_task_executor_plugin': "wait_at_waypoint",
            'wait_at_waypoint': {
                'plugin': "nav2_waypoint_follower::WaitAtWaypoint",
                'enabled': True,
                'waypoint_pause_duration': 200
            }
        },

        # Velocity Smoother
        'velocity_smoother': {
            'smoothing_frequency': 20.0,
            'scale_velocities': False,
            'feedback': "OPEN_LOOP",
            'max_velocity': [0.3, 0.0, 0.3],
            'min_velocity': [-0.3, 0.0, -0.3],
            'max_accel': [10.0, 0.0, 5.0],
            'max_decel': [-2.5, 0.0, -3.2],
            # 'odom_topic': "odom",
            'odom_topic': '/odom',
            'odom_duration': 0.1,
            'deadband_velocity': [0.0, 0.0, 0.0],
            'velocity_timeout': 1.0
        },

        # Local Costmap
        'local_costmap': {
            'local_costmap': {
                'update_frequency': 10.0,
                'publish_frequency': 10.0,
                'global_frame': 'odom',
                'robot_base_frame': 'base_link',
                'use_sim_time': use_sim_time,
                'rolling_window': True,
                'width': 3,
                'height': 3,
                'resolution': 0.05,
                'robot_radius': 0.22,
                'plugins': ["voxel_layer", "inflation_layer"],
                'inflation_layer': {
                    'plugin': "nav2_costmap_2d::InflationLayer",
                    'cost_scaling_factor': 3.0,
                    'inflation_radius': 0.55
                },
                'voxel_layer': {
                    'plugin': "nav2_costmap_2d::VoxelLayer",
                    'enabled': True,
                    'publish_voxel_map': True,
                    'origin_z': 0.0,
                    'z_resolution': 0.05,
                    'z_voxels': 16,
                    'max_obstacle_height': 2.0,
                    'mark_threshold': 0,
                    'observation_sources': 'scan',
                    'scan': {
                        'topic': '/scan',
                        'max_obstacle_height': 2.0,
                        'clearing': True,
                        'marking': True,
                        'data_type': "LaserScan"
                    }
                },
                'always_send_full_costmap': True
            }
        },

        # Global Costmap
        'global_costmap': {
            'global_costmap': {
                'update_frequency': 1.0,
                'publish_frequency': 1.0,
                'global_frame': 'map',
                'robot_base_frame': 'base_link',
                'use_sim_time': use_sim_time,
                'robot_radius': 0.22,
                'resolution': 0.05,
                'track_unknown_space': True,
                'plugins': ["static_layer", "obstacle_layer", "inflation_layer"],
                'obstacle_layer': {
                    'plugin': "nav2_costmap_2d::ObstacleLayer",
                    'enabled': True,
                    'observation_sources': 'scan',
                    'scan': {
                        'topic': '/scan',
                        'max_obstacle_height': 2.0,
                        'clearing': True,
                        'marking': True,
                        'data_type': "LaserScan"
                    }
                },
                'static_layer': {
                    'plugin': "nav2_costmap_2d::StaticLayer",
                    'map_subscribe_transient_local': True
                },
                'inflation_layer': {
                    'plugin': "nav2_costmap_2d::InflationLayer",
                    'cost_scaling_factor': 3.0,
                    'inflation_radius': 0.55
                },
                'always_send_full_costmap': True
            }
        }
    }

    # Controller Server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params['controller_server']]
    )

    # Planner Server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params['planner_server']]
    )

    # Behavior Server
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params['behavior_server']]
    )

    # BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params['bt_navigator']]
    )

    # Waypoint Follower
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params['waypoint_follower']]
    )

    # Velocity Smoother
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params['velocity_smoother']]
    )

    # Local Costmap
    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='costmap_2d_node',
        name='local_costmap',
        output='screen',
        parameters=[nav2_params['local_costmap']]
    )

    # Global Costmap
    global_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='costmap_2d_node',
        name='global_costmap',
        output='screen',
        parameters=[nav2_params['global_costmap']]
    )

    # Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'local_costmap',
                'global_costmap'
            ]
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        local_costmap_node,
        global_costmap_node,
        lifecycle_manager_node
    ])
