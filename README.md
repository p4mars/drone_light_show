# Drone Light Show Set-up
This README will show you how to set-up your environment so you have access to ROS2, Gazebo and PX4 along with creating your workspace for the single vehicle test example. It will also show you how to run the simulation provided. 

For more information about the development of this simulation, please refer to our wiki. 

*Note: This set-up is for Windows!*
  
## Setting up the Development Environment 
A development environment is achieved through the use of devcontainers in VSCode. 

### What is this Development Environment and why use one?

This repository sets up a **ROS 2 (Robot Operating System 2) development environment** within a Docker container for this drone simulation project. It allows you to easily build, develop, and run ROS 2-based applications, without the need to configure dependencies or worry about compatibility issues between different systems when collaborating with others. It is used to keep things consistant between machines so the setup is the same for everyone. Docker allows us to isolate the development environment from our local systems so we don't have any conflicts between other software on our laptops. 

### Prerequisties
Before cloning the repository, please make sure you have the following installed: 

- **Docker**: Ensure that Docker is installed and running on your system. You can follow the [official Docker installation guide](https://docs.docker.com/get-docker/) for your operating system.
- **Docker Desktop**: Make sure Docker Desktop is installed and running on your system. Here is the [installation guide](https://www.docker.com/products/docker-desktop/).
- **VS Code**: For editing and working with the ROS 2 workspace, it is recommended to use Visual Studio Code with the [Remote - Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).
- **Git**: To clone the repository.

Make sure you also have the following dependencies set up so you can work with development containers:
- vscode, setup for [dev containers](https://code.visualstudio.com/docs/devcontainers/containers)
- [docker engine](https://docs.docker.com/engine/install/ubuntu/)

### Cloning the repository 

Once you have the prerequisites set up, it is time to clone the repository. 

1. Open VSCode and open the terminal 
2. Create a folder in your desired directory where you will clone the repository

```powershell
cd <your-directory>
mkdir <folder_name>
```
3. Clone the repository 
```bash
git clone https://github.com/p4mars/drone_light_show.git
```
4. Once it has been cloned, open the folder you cloned the repo into by pressing 'Open Folder' on the left side of the screen. Once opened, press F1 and type into the search bar "Dev Container: Rebuild and Reopen in Container" and click on it. A box on the bottom right hand corner should display "Connecting to Dev Container (show log)". You can click on the 'show log' to view the logs while it is building.

Once the container has been built, you can begin to build your ROS2 Workspace

## ROS2 Workspace 

1. Open a new terminal
2. Create the src directory in the ROS2 workspace repo (created by the dev environment). Clone the example and PX4 message directory in this directory. This will provide the ROS 2 message definitions for the PX4 Autopilot project. Building this package generates all the required interfaces to interface ROS 2 nodes with the PX4 internals. More information can be found [here](https://docs.px4.io/main/en/middleware/uxrce_dds.html#supported-uorb-messages). 
```
  cd ros2_ws
  mkdir src
  cd src
  git clone https://github.com/PX4/px4_msgs.git
  git clone https://github.com/PX4/px4_ros_com.git
```
3. Source the ROS 2 development environment into the current terminal and compile the workspace using colcon. Please keep in mind that this may take up to 20 minutes. 
```
  cd ..
  source /opt/ros/humble/setup.bash
  colcon build
```
3.b. If you have already built the package using colcon build and need to implement changes to a file, rebuild the changed file by doing:
```
colcon build --packages-select <NAME_OF_PACKAGE>
```

4. Source your directory to access the newly built (or rebuilt) package by typing:
```
cd ..
source ros2_ws/install/setup.bash
```

This builds all the folders under /src using the sourced toolchain. To run the example to ensure your set up was done correctly please refer to the [official PX4 documentation](https://docs.px4.io/main/en/ros2/user_guide.html#running-the-example). 

## Establishing Communication between the Agent and the Client 
PX4 is the autopilot firmware which acts as the flight control for the drone. This firmware includes many flight modes that help with the control of the drone such as holding attitude, acrobatic mode for drone racing but also autonomous modes such as return-to-launch and an offboard mode. This offboard mode will be used for this simulation. The [offboard mode](https://docs.px4.io/main/en/flight_modes/offboard.html) allows the companion computer on the drone to take over its operation autonomously by giving the controller control inputs. For us, this is the ROS2 framework that we develop. On the drone, there is a client which allows communication with our ROS2 network. The client communicates with [uORB](https://docs.px4.io/main/en/msg_docs/) topics in the PX4 firmware to turn the commands into code. Our ROS2 network communicates with this client through the agent. This communication link can be seen in the image below. The set up of the client and agent has already been done when building the development environment. 

![image](https://github.com/user-attachments/assets/1a828b53-ca2c-4274-822d-ad8c367571f2)

To establish this communication, the following should be taken:

1. Start the agent in a designated terminal
```bash
MicroXRCEAgent udp4 -p 8888
```
2. In a new terminal, start a PX4-Gazebo simulation with the drone in the PX4-Autopilot directory (you might have to exit the ros2_ws directory by using "cd .." first).
```bash
cd PX4-Autopilot
PX4_GZ_WORLD=sim_world make px4_sitl gz_x500
```
A new pop-up with Gazebo and the X-500 model should show. The agent and client are now running and they should connect.

3. The PX4 terminal should display:
```
   ...
  INFO  [uxrce_dds_client] synchronized with time offset 1675929429203524us
  INFO  [uxrce_dds_client] successfully created rt/fmu/out/failsafe_flags data writer, topic id: 83
  INFO  [uxrce_dds_client] successfully created rt/fmu/out/sensor_combined data writer, topic id: 168
  INFO  [uxrce_dds_client] successfully created rt/fmu/out/timesync_status data writer, topic id: 188
...
```
The Micro XCRE-DDS agent terminal will show the equivalent topics that are created in the DDS network.
```
...
  [1675929445.268957] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x00000001,   publisher_id: 0x0DA(3), participant_id: 0x001(1)
  [1675929445.269521] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x00000001,   datawriter_id: 0x0DA(5), publisher_id: 0x0DA(3)
  [1675929445.270412] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x00000001, topic_id: 0x0DF(2), participant_id: 0x001(1)
  ...
```
4. Once the terminal displays the following:
```
INFO  [px4] Startup script returned successfully
pxh> ...
```
Press enter and input the command (with "pxh>" already provided by the terminal) below to bypass certain pre-arming checks that are not applicable for this simulation.
```
pxh> param import /home/ros/ros2_ws/working.params
```
After which, the terminal will display some messages, ending with:
```
pxh> INFO  [commander] Ready for takeoff!
```
Once this displays, you can now move to running the simulation. 

## Running the Simulation

1. Open a new terminal and source ROS2 and the ROS2 workspace
```
cd ..
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

2. Once everything has been sourced, you can now launch the simulation. 
```
cd ros2_ws
ros2 launch offboard_control_pkg offboard_control_launch_file.launch.py
```

If everything went well, the following messages are displayed to indicate that the offboard control node commands are being sent:
```
[INFO] [launch]: All log files can be found below /home/ros/.ros/log/2025-05-04-13-38-24-823719-docker-desktop-3154
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [offboard_control_node-1]: process started with pid [3195]
...
```

Switch to the Gazebo GUI to watch the simulation!

For more information about our development and how to run our simulation, please refer to our [wiki](https://github.com/p4mars/drone_light_show/wiki).
