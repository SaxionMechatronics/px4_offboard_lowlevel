import launch

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', 
                '/command/trigger',
                '/command/pose',
                '/fmu/out/vehicle_odometry',
                '/fmu/in/vehicle_visual_odometry',
                '/fmu/out/vehicle_control_mode',
                '/fmu/out/vehicle_status_v1',
                '/fmu/out/vehicle_local_position',
                '/fmu/in/vehicle_visual_odometry',
                '/fmu/in/vehicle_rates_setpoint',
                '/nn/actions',
                '/nn/observations'
            ],
            output='screen'
        )
    ])
