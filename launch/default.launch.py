
import launch 
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import sys
import subprocess

# default launch file for the vive tracker ros node. This
# file should not be modified if the user wants different
# behavior, they should edit the openvr_tracker_node.yaml file 
# instead.

# reference files
# https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html
# https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-Event-Handlers.html
# https://get-help.theconstruct.ai/t/stopping-robot-on-shutdown-event/27365/4
# https://stackoverflow.com/questions/89228/how-do-i-execute-a-program-or-call-a-system-command

def generate_launch_description():

    # get the config file
    config = os.path.join(
        get_package_share_directory('openvr_tracker_node'),
        'config',
        'openvr_tracker_node.yaml'
    )

    # the vive node
    openvr_tracker_node = launch_ros.actions.Node(
        package='openvr_tracker_node',
        executable='openvr_tracker_node',
        namespace='openvr_tracker',
        name='openvr_tracker',
        parameters=[config]
    )

    # command that will launch steam vr for us
    # note you may have to edit this path if the position of the steam
    # directory is different on your system. The command (for linux) should
    # be the same though, so just try and find steam-apps and copy from there
    steam_vr_process = launch.actions.ExecuteProcess(
        cmd=[[
            '~/.steam/debian-installation/steamapps/common/SteamLinuxRuntime_sniper/run ',
            '~/.steam/debian-installation/steamapps/common/SteamVR/bin/vrstartup.sh'
        ]],
        shell=True
    )

    # unnessary but helpful command that will remove the room setup 
    # window when it is detected during the steam vr launch
    steam_vr_streamline = launch.actions.ExecuteProcess(
        cmd=[[
            'while ! ps -e | grep steamvr_room_se; do sleep 3; done && pkill -9 steamvr_room_se'
        ]],
        shell=True
    )

    # command to execute during the cleanup process
    # kills the steam vr instance running in the background
    def kill_steam_vr(launch_context):
        subprocess.run(['pkill','-9','vrmonitor'])


    return launch.LaunchDescription([
        steam_vr_process,
        steam_vr_streamline,
        openvr_tracker_node,

        # execute our kill_steam_vr command on shutdown (CTRL-C)
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnShutdown(
                on_shutdown=[
                    launch.actions.OpaqueFunction(
                        function=kill_steam_vr
                    )
                ]
            )
        )
    ])
