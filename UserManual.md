# User Manual

## Table of Contents

1. [Environment Setup](#environment-setup)
    1. [Windows](#windows)
        1. [WSL](#wsl)
        2. [ROS](#ros-windows)
        3. [ROS Bridge](#ros-bridge-windows)
        4. [VSCode](#vscode-windows)
        5. [Microsoft ROS Ext](#microsoft-ros-ext-windows)
        6. [RIDE](#ride-windows)
    2. [Linux - Ubuntu 20.04](#linux-ubuntu)
        1. [VM Virtual Box](#vm-virtual-box-linux)
        2. [ROS](#ros-linux)
        3. [ROS Bridge](#ros-bridge-linux)
        4. [VSCode](#vscode-linux)
        5. [Microsoft ROS Ext](#microsoft-ros-ext-linux)
        6. [RIDE](#ride-linux)
    3. [Mac](#mac)
        1. [VM Virtual Box](#vm-virtual-box-mac)
        2. [ROS](#ros-mac)
        3. [ROS Bridge](#ros-bridge-mac)
        4. [VSCode](#vscode-mac)
        5. [Microsoft ROS Ext](#microsoft-ros-ext-mac)
        6. [RIDE](#ride-mac)
2. [RIDE](#ride)
    1. [Catkin Workspace](#catkin-workspace)
    2. [Create Package (with MS ROS extension)](#create-package)
    3. [Creation Wizard](#creation-wizard)
    4. [Topic Monitor](#topic-monitor)
    5. [Message Publisher](#message-publisher)


## Environment Setup

### Windows

#### WSL

1. Install WSL2 on your Windows machine following the official Microsoft [WSL installation guide](https://docs.microsoft.com/en-us/windows/wsl/install).
2. Install Ubuntu 20.04 from the Microsoft Store.
3. Launch the Ubuntu terminal and follow the steps for [Linux - Ubuntu 20.04](#linux-ubuntu) for the remaining setup.

#### ROS (Windows)

*Windows support for ROS is limited. It is recommended to use WSL2 with Ubuntu 20.04 for a better experience.*

#### ROS Bridge (Windows)

*Refer to the ROS Bridge setup in the [Linux - Ubuntu 20.04](#ros-bridge-linux) section and follow the same steps in the Ubuntu terminal on WSL2.*

#### VSCode (Windows)

1. Download and install [Visual Studio Code](https://code.visualstudio.com/download) for Windows.
2. Launch Visual Studio Code.

#### Microsoft ROS Ext (Windows)

1. Open the Extensions view in VSCode by clicking on the Extensions icon in the Activity Bar on the side of the window.
2. Search for "ROS" in the search bar.
3. Click on "Robot Operating System (ROS)" by Microsoft and then click "Install".

#### RIDE (Windows)

*Refer to the RIDE setup in the [Linux - Ubuntu 20.04](#ride-linux) section and follow the same steps in the Ubuntu terminal on WSL2.*

### Linux - Ubuntu 20.04

#### VM Virtual Box (Linux)

1. Download and install [VirtualBox](https://www.virtualbox.org/wiki/Downloads) for your host operating system.
2. Download the Ubuntu 20.04 LTS Desktop ISO from the [official website](https://releases.ubuntu.com/20.04/).
3. Create a new virtual machine in VirtualBox using the downloaded ISO file and follow the installation steps.

#### ROS (Linux)

1. Follow the official ROS installation guide for [ROS Noetic on Ubuntu 20.04](http://wiki.ros.org/noetic/Installation/Ubuntu).
2. Set up the ROS environment by adding the following lines to your `~/.bashrc` file:

```bash
source /opt/ros/noetic/setup.bash
```

3. Restart your terminal or run source ~/.bashrc.

#### ROS Bridge (Linux)

1. Install rosbridge-suite with the following command:
```bash
sudo apt-get install ros-noetic-rosbridge-server
```
