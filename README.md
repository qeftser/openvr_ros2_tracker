
## ROS2 OpenVR Tracker Bridge

This is a simple ROS2 node that launches SteamVR, connects to it through
OpenVR, and publishes any tracking data it can find as an odometry via a
ROS2 topic. The full message type is nav_msgs/msg/odometry.

## Use

### Running 

You will have to direct the cmake build file to the include directory for
OpenVR. This can be done by exporting the directory that openvr is in:
```
export OPENVR_DIRECTORY=<path to openvr dir>
```
... or by changing line 8 to the path to the directory you built openvr in on your machine.   
   
Now, provided you have all the prerequisites ready to go, running the system
is simple. Connect a SteamVR compatible tracker to your system, begin
running a base station in the machine's line of sight, and launch the node:
```
source install/setup.bash
ros2 launch openvr_tracker_node default.launch.py
```

### Parameters

The exposed parameters for this node are listed in the config/openvr_tracker_node.yaml
file. The parameter **topic** controls the topic to publish the odometry to. The parameter
**visualization** controls whether or not to publish a visualization to rviz2. The 
visualization is a simple publishing of a single small cube at the current position
and orientation of the tracker. This cube will remain for three seconds, so you can
drag the tracker around and get a good idea of the quality and rate of the tracking.
**visual_topci** is the topic to publish this visual data to. **poll_delay** specifies how
long to wait between polling. This value is in milliseconds.

### Prerequisites

In order for this node to work, SteamVR and OpenVR must be installed on
your system. This mean Steam must also be installed. For ubuntu/debian,
which is the expected OS for ROS2, the command for installing Steam is
```
sudo apt install software-properties-common apt-transport-https curl steam-installer steam-devices
```
From here, log into Steam or create an account and install SteamVR and
the linux runtime sniper. Linux runtime sniper may install automatically
with SteamVR. To install OpenVR, clone the repo and build it.
```
git clone https://github.com/ValveSoftware/openvr.git
cd ./openvr
mkdir build && cd ./build
cmake .. && make && sudo make install
```

### Troubleshooting

#### OpenVR build failure 
OpenVR may fail to build. If so try commenting out lines 92 and 93 in
the CMakeLists.txt file. That worked on my machine :)   
   
#### SteamVR launch failure

If the launch script fails with an error relating to being unable to
find the SteamVR launch files, this is likely due to them being installed
on a different area of the machine. You are looking for the files 
```
... steamapps/common/SteamLinuxRuntime_sniper/run
and
... steamapps/common/SteamVR/bin/vrstartup.sh
```
Once you have found these, edit the launch/default.launch.py file
to reflect their positions. This is on line 45.

#### SteamVR crash

Because this is running without a headset, you may run into issues
with the system crashing due to it attempting to set up a window
to display what happens on the headset. To negate that, follow
the instructions listed on this page:
```
https://github.com/username223/SteamVRNoHeadset
```

## Implimentation

Here are the various pages used to aid in implimentation of the system:
```
https://github.com/osudrl/CassieVrControls/wiki/OpenVR-Quick-Start
https://github.com/ValveSoftware/openvr/wiki/IVRSystem_Overview
https://github.com/ValveSoftware/openvr/tree/master?tab=readme-ov-file
https://stackoverflow.com/questions/71053533/linking-openvr-through-vcpkg-via-cmake
```

## TODO

Correctly set up frame ids. All frame_id values in meesages are currently
set to base_link. 
