import launch
import launch_ros
#方法名固定
def generate_launch_description():

    action_node_face_detector=launch_ros.actions.Node(
        package='face_detection',
        executable='face_detector_node',
        output='screen'
    )
    action_node_image_viewer=launch_ros.actions.Node(
        package='face_detection',
        executable='image_viewer_node',
        output='screen'
    )
    return launch.LaunchDescription([
        action_node_face_detector,
        action_node_image_viewer,
    ])