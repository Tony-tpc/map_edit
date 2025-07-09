from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import os
def generate_launch_description():
        mapedit_rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace='',
        arguments=["-d", os.path.join(get_package_share_directory('map_edit')
, 'rviz', 'default.rviz')],
        output="screen",
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )
        return LaunchDescription(
        [
        mapedit_rviz,
        ])
