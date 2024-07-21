
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    # number_publisher_node = Node(
    #     package='my_py_pkg',
    #     executable='number_publisher'
    # )

    # Remap rules

    # Nodes
    turtlesim_1_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
    )

    turtle_spawner_node = Node(
        package='turtle_project_exp',
        executable='turtle_spawner',
        name='turtle_spawner',
        parameters=[
            {'spawn_frequency': 0.33}        
            ]

        
    )

    turtle_controller_node = Node(
        package='turtle_project_exp',
        executable='turtle_controller',
        name='turtle_controller',
        parameters=[
            {'Kp_angle': 6.0},
            {'Kp_x': 2.0}
        ]
    )
    
    ld.add_action(turtlesim_1_node)
    ld.add_action(turtle_spawner_node)
    ld.add_action(turtle_controller_node)
    return ld
