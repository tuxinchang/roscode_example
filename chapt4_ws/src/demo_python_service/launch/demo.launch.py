import launch
import launch_ros
#方法名固定
def generate_launch_description():
    
    #1.声明一个launch参数用于替换某个节点的参数值
    action_declare_arg_background_g=launch.actions.DeclareLaunchArgument(
    'launch_arg_bg',default_value='150')
    


    """产生launch描述"""
    action_node_turtlesim_node=launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        #2.把launch的参数手动传递给某个节点
        parameters=[{'background_g':launch.substitutions.LaunchConfiguration(
        'launch_arg_bg',default='150')}],
        output='screen'
    )
    action_node_turtle_client=launch_ros.actions.Node(
        package='demo_python_service',
        executable='turtle_client',
        output='log'
    )
    action_node_turtle_service=launch_ros.actions.Node(
        package='demo_python_service',
        executable='turtle_service',
        output='both'
    )
    return launch.LaunchDescription([
        # actions动作
        #带参数的启动命令：ros2 launch demo_python_service demo.launch.py launch_arg_bg:=255
        action_declare_arg_background_g,
        action_node_turtlesim_node,
        action_node_turtle_client,
        action_node_turtle_service,
    ])