
# A Unity3d - ROS based Underwater Simulator

This repo is based on the published paper:

```
@inproceedings{chaudhary_2021,
   author = {Akash Chaudhary and Rajat Mishra and Bharath Kalyan and Mandar Chitre},
   title = {Development of an Underwater Simulator using Unity3D and Robot Operating System},
   year = 2021,
   month = {Sep},
   publisher = {{IEEE}},
   booktitle = {{OCEANS} 2021 {MTS}/{IEEE}}
}
```

It is an underwater simulator developed using Unity3D engine and Robot Operating System (ROS), which are connected using ROsBridge library. This bridge manages exchange of data such as recieving controls, sending video feeds and sensor parameters. This document will serve as a quick setup guide to get the project running. If you need more information about the project and its components, please refer to the Wiki of this GitHub repository.

## On Unity side

**This step can be skipped in case you wish to uses the releases directly and do not wish to build the simulator. The releases can be accessed via the [link here](https://github.com/org-arl/UWRoboticsSimulator/releases).**

1. First, you need to install UnityHub with Unity version `2019.4.4f1`. More details to download are present [here](https://unity.com/download#how-get-started). In case, this version is not available then Unity Hub will help install it later during project import step.
2. Clone this repositry onto your system -

   ```bash
   $ git clone https://github.com/org-arl/UWRoboticsSimulator.git
   ```

3. Open Unity Hub and Add the UWRoboticsSimulator folder. Unity Hub will automatically detect the sub-folders and the Unity version. Unity Hub will suggest downloading the version in case it is not downloaded.
4. Once the folder is added successfully, open it using Unity Editor. To start the simulator, find and open the scene "StartScreen" in Assests and press 'Play' (Ctrl + P).
5. Enter the IP address and the port number of the system running the rosbridge websocket in the filed provided and press "Start".

   *Note: If you press start without entering any address or port number, the system will choose ws://localhost:9090 as the default address.*

6. This will start the main screen of the simulator. This can now be controlled using the scripts running on the ROS side.

## On ROS side

1. Install ROSBridge suite on your linux machine.

   ```bash
   $ sudo apt-get install ros-<rosdistro>-rosbridge-suite
   ```

   *Note: For ROS 2 installation instructions, please refer to the wiki of this repository.*
2. Source your ROS distro and run the websocket launch file.

   ```bash
   $ source /opt/ros/<rosdistro>/setup.bash
   $ roslaunch rosbridge_server rosbridge_websocket.launch
   ```

   *This will also run 'roscore', so no need to run it seperately.*
3. Check the host IP address in a new terminal using -

   ```bash
   $ hostname -I
   ```

   *Note: If you are running Unity and ROS on the same system, the IP address would be 'ws://localhost:9090'*

4. Open a new terminal and source your ros distro. Then make a workspace with a package inside having rospy as a dependency and run the scipt in the 'ROS' folder of this repository.

   The control keys are as follows:

   | Keys | Command conveyed to Simulator|
   |------|------------------------------|
   |**r** | Releases the ROV from the station.|
   |**w** / **s** | Adds / subtracts 1 to the surge vector.|
   |**a** / **d** | Adds / subtracts 1 to the sway vector.|
   |**b** / **space** | Adds / subtracts 1 to the heave vector.|
   |**i** / **k** | Adds / subtracts 1 to the pitch rotation vector.|
   |**j** / **l** | Adds / subtracts 1 to the roll rotation vector.|
   |**q** / **w** | Adds / subtracts 1 to the yaw rotation vector.|
   |**m** | Mark/freezes the coordinates of ROV's current location.|
   |**[** / **]** | Switches the front light off / on.|
   |**;** / **'** | Switches the down light off / on.|
   |**x** | Resets all translation and rotation vector to zero.|
   |**c** | Stop the control script.|
