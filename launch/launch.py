import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "fake_path_publisher"
    pkg_share_path = get_package_share_directory(pkg_name)
 
    ld.add_action(launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false"))

    DRONE_DEVICE_ID=os.getenv('DRONE_DEVICE_ID')

    namespace=DRONE_DEVICE_ID
    ld.add_action(
        Node(
            package=pkg_name,
            executable='talker',
            name='fake_path_publisher',
            remappings=[
                ('path_out', "/" + DRONE_DEVICE_ID + "/other_drone_plan"),
                ('trajectory_out', "/fleet/trajectory/in"),
                ('home_pos_in', "/" + DRONE_DEVICE_ID + "/fmu/home_position/out"),
            ],
            output='screen',
            parameters=[
                pkg_share_path + '/config/default.yaml',
                {"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
                {"geographic_global_frame": DRONE_DEVICE_ID+"/geo"}
            ],
        ),
    )

    return ld
