from launch import LaunchDescription
from launch_ros.actions import Node # type: ignore

def generate_launch_description():
    description = LaunchDescription()

    publisher = Node(package='pubsub_demo',
                     executable='small_publisher',
                     output = 'screen')
    
    subscriber = Node(package='pubsub_demo',
                    executable='small_subscriber',
                    output = 'screen')

    description.add_action(publisher)
    description.add_action(subscriber)


 
    return description