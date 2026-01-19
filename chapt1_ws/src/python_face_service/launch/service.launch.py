import launch
import launch_ros
#方法名固定
def generate_launch_description():

    action_node_face_service=launch_ros.actions.Node(
        package='python_face_service',
        executable='face_detect_node',
        output='screen'
    )
    action_node_face_client=launch_ros.actions.Node(
        package='python_face_service',
        executable='face_detect_client_node',
        output='screen'
    )
    return launch.LaunchDescription([
        action_node_face_service,
        action_node_face_client,
    ])