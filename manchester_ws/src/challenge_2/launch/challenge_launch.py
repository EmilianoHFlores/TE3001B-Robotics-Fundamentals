import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('challenge_2'),
        'config',
        'params.yaml'
        )
    return launch.LaunchDescription(
        [
        launch_ros.actions.Node(
            package='challenge_2',
            executable='signal_generator',
            name='signal_generator',
            parameters=[config],
            ),
        launch_ros.actions.Node(
            package='challenge_2',
            executable='signal_reconstructor',
            name='signal_reconstructor',
            ),
        launch_ros.actions.Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='plotter',
            arguments=['/signal/data', '/reconstructed_signal/data'],),
        
  ])