from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    # Static TF
    spawner = Node(
        package="panda_vrep",
        executable="vrep_spawn",
        name="vrep_spawn",
        output="log"
    )


    vrep=ExecuteProcess(
        cmd=["$COPPELIASIM_ROOT_DIR/coppeliaSim.sh"],
        shell=True,
        output="log",
    )


    return LaunchDescription(
        [
            vrep, 
            spawner
        ]

    )