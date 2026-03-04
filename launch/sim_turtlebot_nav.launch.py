# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

pkg_dis_tutorial3 = get_package_share_directory('dis_tutorial3')

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='', description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='bird_demo1', description='Simulation World'),
    DeclareLaunchArgument('model', default_value='standard', choices=['standard', 'lite'], description='Turtlebot4 Model'),
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='use_sim_time'),
    DeclareLaunchArgument('map', default_value=PathJoinSubstitution([pkg_dis_tutorial3, 'maps', 'bird_demo.yaml']), description='Full path to map yaml file to load')
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0', description=f'{pose_element} component of the robot pose.'))

def generate_launch_description():    
    # Launch Files
    gazebo_launch = PathJoinSubstitution([pkg_dis_tutorial3, 'launch', 'sim.launch.py'])
    robot_spawn_launch = PathJoinSubstitution([pkg_dis_tutorial3, 'launch', 'turtlebot4_spawn.launch.py'])
    localization_launch = PathJoinSubstitution([pkg_dis_tutorial3, 'launch', 'localization.launch.py'])
    nav2_launch = PathJoinSubstitution([pkg_dis_tutorial3, 'launch', 'nav2.launch.py'])

    #Simulator and world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world')),
            ('use_sim_time', LaunchConfiguration('use_sim_time'))
        ]
    )

    #Spawn turtlebot in the world
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
            ('use_sim_time', LaunchConfiguration('use_sim_time'))
        ]
    )
    
    # Localization
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([localization_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('map', LaunchConfiguration('map')),
        ]
    )

    # Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('use_sim_time', LaunchConfiguration('use_sim_time'))
        ]
    )


    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(robot_spawn)
    ld.add_action(localization)
    ld.add_action(nav2)
    return ld