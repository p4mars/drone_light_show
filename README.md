# Drone Light Show Set-up
This README will show you how to do the following:
- Set-up the environment so you have access to ROS2 and Gazebo along with creating your workspace. 
- TBD!

*Note: This set-up is for Windows!*
  
## Setting up the Development Environment 
A development environment is achieved through the use of devcontainers in VSCode. 

### What is this Development Environment and why use one?

This repository sets up a **ROS 2 (Robot Operating System 2) development environment** within a Docker container for this drone simulation project. It allows you to easily build, develop, and run ROS 2-based applications, without the need to configure dependencies or worry about compatibility issues between different systems when collaborating with others. It is used to keep things consistant between machines so the setup is the same for everyone. Docker allows us to isolate the development environment from our local systems so we don't have any conflicts between other software on our laptops. 

### Prerequisties
Before cloning the repository, please make sure you have the following installed: 

- **Docker**: Ensure that Docker is installed and running on your system. You can follow the [official Docker installation guide](https://docs.docker.com/get-docker/) for your operating system.
- **Docker Desktop**: Make sure Docker Desktop is installed and running on your system. Here is the [installation guide](https://www.docker.com/products/docker-desktop/).
-**VS Code**: For editing and working with the ROS 2 workspace, it is recommended to use Visual Studio Code with the [Remote - Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).
- **Git**: To clone the repository.

Make sure you also have the following dependencies set up so you can work with development containers:
- vscode, setup for [dev containers](https://code.visualstudio.com/docs/devcontainers/containers)
-[docker engine](https://docs.docker.com/engine/install/ubuntu/)

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
4. Once it has been cloned, press F1 and type into the search bar "Dev Container: Rebuild and Reopen in Container" and click on it. A box on the bottom right hand corner should display "Connecting to Dev Container (show log)". Click on the 'show log' to view the logs while it is building.

Once the container has been built and open your workspace should now be successfully created and ready to use!





