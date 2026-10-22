from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    world = str(Path(get_package_share_directory("warehouse_relay_ideal")) / "worlds" / "warehouse.sdf")
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="random"),
        DeclareLaunchArgument("seed", default_value="42"),
        DeclareLaunchArgument("window_start_s", default_value="2.0"),
        DeclareLaunchArgument("window_end_s", default_value="12.0"),
        DeclareLaunchArgument("output_dir", default_value="outputs_v2/ros"),
        DeclareLaunchArgument("model_plugin", default_value=""),
        DeclareLaunchArgument("predictor_path", default_value=""),
        DeclareLaunchArgument("strategy", default_value="predeploy"),
        DeclareLaunchArgument("map_observation", default_value="ideal"),
        ExecuteProcess(cmd=["ign", "gazebo", "-r", world], output="screen"),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="gazebo_clock_bridge",
             arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"], output="screen"),
        Node(package="warehouse_relay_ideal", executable="ideal_scheduler", output="screen",
             parameters=[{
                 "use_sim_time": True,
                 "mode": ParameterValue(LaunchConfiguration("mode"), value_type=str),
                 "seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
                 "window_start_s": ParameterValue(LaunchConfiguration("window_start_s"), value_type=float),
                 "window_end_s": ParameterValue(LaunchConfiguration("window_end_s"), value_type=float),
                 "output_dir": ParameterValue(LaunchConfiguration("output_dir"), value_type=str),
                 "model_plugin": ParameterValue(LaunchConfiguration("model_plugin"), value_type=str),
                 "predictor_path": ParameterValue(LaunchConfiguration("predictor_path"), value_type=str),
                 "strategy": ParameterValue(LaunchConfiguration("strategy"), value_type=str),
                 "map_observation": ParameterValue(LaunchConfiguration("map_observation"), value_type=str),
             }]),
    ])
