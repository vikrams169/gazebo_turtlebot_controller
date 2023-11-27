import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    record_bag_flag_arg = DeclareLaunchArgument(
        "record_bag", 
        default_value=TextSubstitution(text="1")
    )

    export_turtlebot_burger = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger')

    export_gazebo_model_path= SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value='$GAZEBO_MODEL_PATH:ros2 pkg prefix turtlebot3_gazebo /share/turtlebot3_gazebo/models/')

    turtlebot_world = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_gazebo'),'launch'), '/turtlebot3_world.launch.py']) )

    controller_node = Node(
        package="gazebo_turtlebot_controller",
        executable="controller",
        parameters=[{"record_bag": LaunchConfiguration('record_bag')}]
    )

    ld = LaunchDescription(
        [export_turtlebot_burger, export_gazebo_model_path, turtlebot_world, record_bag_flag_arg, controller_node]
    )

    return ld