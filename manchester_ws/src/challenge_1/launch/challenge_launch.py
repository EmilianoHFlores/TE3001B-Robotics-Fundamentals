import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='challenge_1',
            executable='signal_generator',
            name='signal_generator',
            prefix='terminator --execute',
            ),
        launch_ros.actions.Node(
            package='challenge_1',
            executable='process',
            name='process',
            prefix='terminator --execute',
            ),
        launch_ros.actions.Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='plotter',
            arguments=['/signal/data', '/proc_signal/data'],),
        
  ])