# User Manual

## Table of Contents

1. [Environment Setup](#environment-setup)
    1. [Windows](#windows)
        1. [WSL](#wsl)
        2. [ROS](#ros-windows)
        3. [ROS Bridge](#ros-bridge-windows)
        4. [VSCode](#vscode-windows)
        5. [Microsoft ROS Ext](#microsoft-ros-ext-windows)
        6. [R-IDE](#r-ide-windows)
    2. [Linux - Ubuntu 20.04](#linux-ubuntu)
        1. [VM Virtual Box](#vm-virtual-box-linux)
        2. [ROS](#ros-linux)
        3. [ROS Bridge](#ros-bridge-linux)
        4. [VSCode](#vscode-linux)
        5. [Microsoft ROS Ext](#microsoft-ros-ext-linux)
        6. [R-IDE](#r-ide-linux)
    3. [Mac](#mac)
        1. [VM Virtual Box](#vm-virtual-box-mac)
        2. [ROS](#ros-mac)
        3. [ROS Bridge](#ros-bridge-mac)
        4. [VSCode](#vscode-mac)
        5. [Microsoft ROS Ext](#microsoft-ros-ext-mac)
        6. [R-IDE](#r-ide-mac)
2. [R-IDE](#r-ide)
    1. [Catkin Workspace](#catkin-workspace)
    2. [Create Package (with MS ROS extension)](#create-package)
    3. [Creation Wizard](#creation-wizard)
        1. [ROS Node](#ros-node)
        2. [ROS Msg](#ros-msg)
        3. [ROS Srv](#ros-srv)
    4. [Topic Monitor](#topic-monitor)
    5. [ROS Bags](#ros-bags)
    5. [Message Publisher](#message-publisher)
    6. [Add an Executable](#adding-c-executables)
    7. [Add a Library](#adding-c-libraries)
3. [Other Recommended Extensions](#other-recommended-extensions)


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

#### R-IDE (Windows)

*Refer to the R-IDE setup in the [Linux - Ubuntu 20.04](#r-ide-linux) section and follow the same steps in the Ubuntu terminal on WSL2.*

### Linux - Ubuntu 20.04

#### VM Virtual Box (Linux)

1. Download and install [VirtualBox](https://www.virtualbox.org/wiki/Downloads) for your host OS.
2. Download the [Ubuntu 20.04 ISO file](https://releases.ubuntu.com/20.04/).
3. Create a new virtual machine in VirtualBox, selecting "Linux" and "Ubuntu (64-bit)" as the OS type.
4. Allocate at least 2 CPU cores, 4 GB of RAM, and 20 GB of storage.
5. Mount the downloaded Ubuntu 20.04 ISO file as a virtual optical disk and start the virtual machine.
6. Follow the on-screen prompts to install Ubuntu 20.04.

#### ROS (Linux)

1. Follow the official ROS installation guide for [ROS Noetic on Ubuntu 20.04](http://wiki.ros.org/noetic/Installation/Ubuntu).
2. Set up the ROS environment by adding the following lines to your `~/.bashrc` file:

```bash
source /opt/ros/noetic/setup.bash
```

3. Restart your terminal or run source ~/.bashrc.  

#### ROS Bridge (Linux)  

1. Install the necessary dependencies:  
```bash
sudo apt-get update
```  
and  
```bash
sudo apt-get install ros-noetic-rosbridge-server
```  

2. To start the rosbridge WebSocket server, run:
`roslaunch rosbridge_server rosbridge_websocket.launch`  

#### VSCode (Linux)

1. Download and install [Visual Studio Code](https://code.visualstudio.com/download) for Linux.
2. Launch Visual Studio Code.

#### Microsoft ROS Ext (Linux)

1. Open the Extensions view in VSCode by clicking on the Extensions icon in the Activity Bar on the side of the window.
2. Search for "ROS" in the search bar.
3. Click on "Robot Operating System (ROS)" by Microsoft and then click "Install".

#### R-IDE (Linux)

1. Open the Extensions view in VSCode by clicking on the Extensions icon in the Activity Bar on the side of the window.
2. Search for "r-ide" in the search bar.
3. Click on the icon and then select "Install".

### Mac

#### VM Virtual Box (Mac)

1. Download and install [VirtualBox](https://www.virtualbox.org/wiki/Downloads) for your host OS.
2. Download the [Ubuntu 20.04 ISO file](https://releases.ubuntu.com/20.04/).
3. Create a new virtual machine in VirtualBox, selecting "Linux" and "Ubuntu (64-bit)" as the OS type.
4. Allocate at least 2 CPU cores, 4 GB of RAM, and 20 GB of storage.
5. Mount the downloaded Ubuntu 20.04 ISO file as a virtual optical disk and start the virtual machine.
6. Follow the on-screen prompts to install Ubuntu 20.04.
7. Follow the steps for [Linux - Ubuntu 20.04](#linux-ubuntu) for the remaining setup.

#### ROS (Mac)

*Mac support for ROS is limited. It is recommended to use VM Virtual Box with Ubuntu 20.04 for a better
experience.*

#### ROS Bridge (Mac)

*Refer to the ROS Bridge setup in the [Linux - Ubuntu 20.04](#ros-bridge-linux) section and follow the same steps in the Ubuntu virtual machine on VirtualBox.*

#### VSCode (Mac)

1. Download and install [Visual Studio Code](https://code.visualstudio.com/download) for Mac.
2. Launch Visual Studio Code.

#### Microsoft ROS Ext (Mac)

1. Open the Extensions view in VSCode by clicking on the Extensions icon in the Activity Bar on the side of the window.
2. Search for "ROS" in the search bar.
3. Click on "Robot Operating System (ROS)" by Microsoft and then click "Install".

#### R-IDE (Mac)

*Refer to the R-IDE setup in the [Linux - Ubuntu 20.04](#r-ide-linux) section and follow the same steps in the Ubuntu virtual machine on VirtualBox.*

## R-IDE

### Catkin Workspace

1. Follow the official [ROS tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) to create a new Catkin workspace.

### Create Package (with MS ROS extension)

1. In Visual Studio Code, open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P`).
2. Type "ROS: Create Package" and select the command.
3. Follow the prompts to create your new ROS package.

### Creation Wizard

1. In R-IDE, click the Wizards tab followed by the Create New tab.
2. There are currently three option to create a code template for:

#### ROS Node

1. Choose the file type: C++ or Py.
2. Type the name of the file.
3. Specify the location of the package to put the node in.

#### ROS Msg

1. Choose the file type: Message File.
2. Type the name of the file.
3. Specify the location of the package to put the message file in.

#### ROS Srv

1. Type the service file name.
2. Specify the location of the package to be created in.

### Topic Monitor

1. In R-IDE, click the "Topic Monitor" tab.
2. If ROS Bridge is not running, but is installed then a window will pop up, select "Start ROSBridge". Refer to the [Linux](#ros-bridge-linux), [Windows](#ros-bridge-windows) and [Mac](#ros-bridge-mac) installation guides.
3. Refresh the status to change to Connected.
4. Select the desired topics to monitor from the list by clicking the box.
3. Observe the real-time messages being published on the selected topics.

### ROS Bags

1. Complete Steps 1-3 in [Topic Monitor](#topic-monitor) to establish [ROS Bridge](#ros-bridge-linux) connection.
2. In R-IDE, click the "ROS Bags" tab.
3. Currently only the Manage Bags tab is available. 
4. Select the ROS Bag to play and subscribe to topics to view the message coming through each selected topic.

### Message Publisher

1. In R-IDE, click the "ROS Topic Tools" followed by the "ROS Topic Monitor" tab.
2. Click on the pencil icon next to the topic you to publish to.
3. The type of the topic is displayed and the user can adjust the frequency
3. Click "Publish" to send the message.

### Adding C++ executables

After creating a source file, you can add an executable in `CMakeLists.txt`. R-IDE can create a basic template via the command palette.

1. Open the command palette (`Ctrl + Shift + P` or `Cmd + Shift + P`).
2. Type and select `R-IDE: Add an executable to a package` from the options listed.
3. VScode will then prompt for an executable name. ROS suggests a name of the format: `$ProjectName_function_node`
4. Then select the source files used to create the executable. These should be `.cpp` files visible to the catkin package.
5. R-IDE will then generate a basic function inside `CMakeLists.txt` that will add the executable and link the target libraries.

For further information on `CMakeLists.txt`, go to [the wiki page](http://wiki.ros.org/catkin/CMakeLists.txt)

### Adding C++ libraries

After creating a source file, you can add a library in `CMakeLists.txt`. R-IDE can create a basic template via the command palette.

1. Open the command palette (`Ctrl + Shift + P` or `Cmd + Shift + P`).
2. Type and select `R-IDE: Add a library to a package` from the options listed.
3. VScode will then prompt for an library name. ROS suggests a name of the format: `$ProjectName_function_node`
4. Then select the source files used to create the library. These should be `.cpp` files visible to the catkin package.
5. R-IDE will then generate a basic function inside `CMakeLists.txt` that will add the library.

For further information on `CMakeLists.txt`, go to [the wiki page](http://wiki.ros.org/catkin/CMakeLists.txt)


#### Other Recommended Extensions

1. Language extension for Python and/or C++  
   [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python)  
   [C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)

2. WSL or SSH extension  
   [WSL](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl)  
   [SSH](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh)

3. CMake Extension including Intellisense  
   [CMake](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools)