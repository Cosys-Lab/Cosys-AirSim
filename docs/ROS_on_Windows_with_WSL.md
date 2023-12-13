# Using Ros on Windows with WSL

In Windows command prompt as admin:
`wsl --install`

`reboot`

-Ubuntu terminal will install (default Ubuntu 20.04), choose a username and password

-Optional: Run this command to never prompt the current user for a password when that user uses sudo
`echo "$USER ALL=(ALL:ALL) NOPASSWD: ALL" | sudo tee "/etc/sudoers.d/dont-prompt-$USER-for-sudo-password"`


## Install ROS (for example noetic) in the ubuntu terminal
	
`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`

`sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`

`sudo apt update`

`sudo apt install ros-noetic-desktop-full`

`echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc`

`sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential`

`sudo apt install python3-rosdep`

`sudo rosdep init`

`rosdep update`
     

## Install PIP
`sudo apt install python3-pip`

## Install msgpack-rpc-python
`pip install msgpack-rpc-python`

## API connection 
For the API to work you need to insert your wsl IP address every time you reboot by doing the following:

In windows command prompt:
`ipconfig`

Scroll down and for "Ethernet adapter vEthernet (WSL)" copy the  IPv4 Address

now open file: AirsimUnreal\airsim\PythonClient\airsim\client.py

In line 17 paste the copied IPv4 Address inside ip= "..." 

A similar procedure is needed to use the launch files, replace the IP address in these launch files and for "port" put `default="41451"`

## Example: DroneAPI: in Ubuntu (WSL) terminal:
`cd /mnt/c`  (to go to c drive for example) 

For drone API for example to takeoff:

`cd /mnt/c/Users/{USERNAME}/AirsimUnreal/airsim/PythonClient/multirotor`

`python3 ./takeoff.py` 