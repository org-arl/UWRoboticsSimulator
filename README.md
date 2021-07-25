# Project MATE
## A Unity3d - ROS based underwater simulator.

Project MATE is an underwater simulator developed in the Unity3D engine which connects with Robot Operating System (ROS), using ROsBridge library, to recieve controls and send video feed and parameters over the connection. This document will serve as a quick setup guide to get the project running. If you need more information about the project and its components, please refer to the wiki of this GitHub repository.   


**On ROS side**
1. Install ROSBridge suite on your linux machine.
   ```console
   akash@unitymachine:~$ sudo apt-get install ros-<rosdistro>-rosbridge-suite
   ```
   *Note: For ROS 2 installation instructions, please refer to the wiki of this repository.*
   
2. Source your ROS distro and run the websocket launch file. 
   ```console
   akash@unitymachine:~$ source /opt/ros/<rosdistro>/setup.bash
   akash@unitymachine:~$ roslaunch rosbridge_server rosbridge_websocket.launch
   ```
   *This will also run 'roscore'.*

3. Check the host IP address in a new terminal using 
   ```console
   akash@unitymachine:~$ hostname -I
   ```
*Note: If you are running Unity and ROS on the same system, the IP address would be 'ws://localhost:9090'*

4. Open a new terminal and source your ros distro. Then make a workspace with a package inside having rospy as a dependency and run the scipt in the 'ROS' folder of this repository. 

   The control works as follows: 

   **w** / **s** : Adds / subtracts 1 to the surge vector 
   
   **a** / **d** : Adds / subtracts 1 to the sway vector
   
   **b** / **space** : Adds / subtracts 1 to the heave vector

   **i** / **k** : Adds / subtracts 1 to the pitch rotation vector

   **j** / **l** : Adds / subtracts 1 to the roll rotation vector

   **q** / **w** : Adds / subtracts 1 to the yaw rotation vector
   
   **r** : Releases the ROV from the station

   **m** : Mark/freezes the coordinates to mark the pipeline leak

   **[** / **]** : Switches the front light off / on

   **;** / **'** : Switches the down light off / on

   **x** : Resets all translation and rotation vector to zero.

   **c** : Stop the control script




**On Unity side:**

1. Clone this repositry onto your system
   ```console 
   akash@unitymachine:~$ git clone https://github.com/rajmis/UnderWaterSimulator.git
   ```
2. Open the scene "StartScreen" in Unity3D and press 'Play' (Ctrl + P).

3. Enter the IP address and the port number of the system running the rosbridge websocket in the filed provided and press "Start".

   *Note: If you press start without entering any address or port number, the system will choose ws://localhost:9090 as the default address.*

4. This will start the main screen of the simulator. This can now be controlled using the scripts running on the ROS side. 