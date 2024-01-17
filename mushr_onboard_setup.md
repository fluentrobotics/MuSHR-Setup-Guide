---
title: MuSHR Robot Setup Guide - Fluent Robotics Lab
author: Jeeho Ahn
date: Nov. 2023
---

# MuSHR Setup Guide - Fluent Robotics Lab

There is an official tutorial page for MuSHR system, yet it is not very easy to follow as it does not provides a straightforwad flow to follow.

This guide suggest a single flow to follow for setting up the software system for a MuSHR car.

## Table of Content

- Flashing SD Card with Jetson Nano Image
- Wireless Network 
- Remote Environment with VNC and SSH
- Use of Noetic Docker
- Testing Teleoperation
- Tune Steering Servo Offset


## Remote PC and On-board PC
MuSHR is based on a small-sized ARM-based on-board embedded PC, Jetson Nano. Each on-board PC is expected to handle the real-time control of the motors and light-weighted sensor data process or decentralized plannings. For resource-heavy operations and centralized multi-agent plannings, we will need a remote PC with higher computing power, such as a desktop.

## About Quickstart tutorial
The quickstart parts, legacy and with Foxglove, are for setting up a Remote PC for simulations and remote operations, not exactly the onboard PCs (Jetson Nano) for real-world experiments. If there is a need for setting up a real hardware, I suggest to setup the onboard system first.

## Flashing SD Card and First Boot

As guided in the official tutorial, we will first flash an SD card with the Jetson Nano image by NVidia. Flashing can be done by either terminal tools such as `dd` or a gui tools such as `Balena Etcher`.

It is recommened to use the version of the image MuSHR tutorial provides.
- [NVidia Jetpack](https://developer.nvidia.com/embedded/l4t/r32_release_v7.1/jp_4.6.1_b110_sd_card/jeston_nano/jetson-nano-jp461-sd-card-image.zip) (change the link if the official tutorial has an update on it)
- [Balena Etcher](https://www.balena.io/etcher/)

The flash process is as simple as you can imagine.

Once flashing is finished, insert the card into Jetson Nano and supply power to turn on. The MuSHR tutorial has recommended names for user and computer, but for our lab use, I suggest the followings:

- user: robot
- computer: fluent-mushr[`unique_number`] (i.e. fluent-mushr2)
- password: `fluent`

For `unique_number`, use the numbered ID of each car.

- App Partition Size: set maximum capacity available.
- Select Nvpmodel Mode: set MAXN

## Wireless Network (Lab Router)
Setup WiFi as described in the lab network documentation below. (You will need permission to view)

https://docs.google.com/document/d/1-JioNW9RgK9IkaMXef70A-jW1nOKywiEXf0jTHesY2Q/edit?usp=sharing

As of the point of writing this documentation, the designated access credential is as follows:

```
SSID: fluent-5
Password: This-Network-Is-For-Robot-Use-Only
```

### Recommended setup for manual IP address
While it is not requirement, it is recommended to setup the IP address of each mushr car in the following format.
```
IP address: 192.168.1.(car number + 10) (i.e. 192.168.1.12 for car 2)
subnet mask: 255.255.255.0
```

## Remote Development Environment 

### On VNC

By default, the given image has the `Desktop Sharing` feature of Ubuntu disabled. We can enable it back to connect over VNC.

(source: https://medium.com/@bharathsudharsan023/jetson-nano-remote-vnc-access-d1e71c82492b)

Open the `xml` file to edit.
```
sudo vim /usr/share/glib-2.0/schemas/org.gnome.Vino.gschema.xml
```

Add the following code.

```
<key name='enabled' type='b'>
<summary>Enable remote access to the desktop</summary>
<description>
If true, allows remote access to the desktop via the RFB
protocol. Users on remote machines may then connect to the
desktop using a VNC viewer.
</description>
<default>true</default>
</key>
```
Compile.
```
sudo glib-compile-schemas /usr/share/glib-2.0/schemas
```

The `Desktop Sharing` feature should now be accessible on the `Settings` panel.

Setup a password, and uncheck `confirm everytime` option.

Next, setup VNC server to run at startup.
```
mkdir -p ~/.config/autostart

cp /usr/share/applications/vino-server.desktop ~/.config/autostart/.
```

Then, disable encryption and prompt.
```
gsettings set org.gnome.Vino require-encryption false
gsettings set org.gnome.Vino prompt-enabled false
```

Finally, reboot Jetson Nano.



### On SSH

Remote operations over SSH can be setup as usual. Normally, there is no need for an additional setup.

## MuSHR Noetic Docker

The given Jetson Nano image is on Ubuntu 18.04, but the MuSHR system is later updated to use Ubuntu 20.04. The official tutorial suggests to use a Docker image it provides for the easiness in setting up. We will also follow this way.

https://mushr.io/tutorials/noetic_first_steps/

### Install Docker Compose

The Jetson Nano image comes with `nvidia-docker`, so additional docker is not needed. However, we still need to install `docker-compose` as suggested in the MuSHR tutorial.

```
sudo apt-get update
sudo apt-get install curl
sudo apt-get install python-pip
sudo curl -L "https://github.com/docker/compose/releases/download/v2.17.2/docker-compose-linux-armv7" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
pip install docker-compose
```

The last error message regarding python versions is neglectable.

Also, it is recommended to execute the following commands for previlage issues.
https://docs.docker.com/engine/install/linux-postinstall/

```
#sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

We can verify the fix in previlage with the following command.
```
docker run hello-world
```

If it runs successfully, you will see `Hello from Docker!` message.

### Install MuSHR Docker Container

https://mushr.io/tutorials/noetic_quickstart/

```
mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/src

#clone repository
git clone --branch noetic https://github.com/prl-mushr/mushr.git

#run install script
./mushr/mushr_utils/install/mushr_install.bash
```

Select options as below:
- Are you install on robot and need all the sensor drivers?: ``y``
- Building from scratch?: ``n``

### Running MuSHR Noetic Docker Image

Open another terminal (you can close the current one) or source .bashrc. Then run the commands below:
```
mushr_noetic
```
At the first run, it will start initiating the Docker container.

On the terminal running the Docker container, build catkin_ws with the scripts below:

```
cd catkin_ws && catkin build
```

Add source script to `.bashrc`.
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

source .bashrc
```


### (Optional) Setup for Logitech F710 Joystick Controller
Normally, Logitech F710 joystick does not require any additional installation other than the default `joystick` Ubuntu package. However, specifially for Nvidia Jetson, there is a hardware compatibility issue. Out of the box, the controller's dongle is first recognized then gets disconnected after any single input from the controller.

This can be fixed by installing a module in this documentation repository. Run the following shell commands and reboot.

```
cd logitech-f710-module
chmod +x install-module-fluent.sh
./install-module-fluent.sh
```

After a re-boot, the functionality can be tested with the following command on the Docker container:
```
jstest /dev/input/js0 #js0 is the most commonly found device name
```

If `jstest` is not found, install `joystick` package.
```
sudo apt-get install joystick
```

It may fail at first, but plugging out and back in then waiting for 5 seconds usually fixes it.

The original repository for the module installation is here: https://github.com/jetsonhacks/logitech-f710-module

## Teleoperation Test

In our lab, we have two different joystick controllers, PS4 Dual Shock 4 and Logitech F710.

Dual Shock 4               |  F710
:-------------------------:|:-------------------------:
![Dual Shock 4](images/dualshock4.jpg) |  ![F710 controller](images/f710.jpg)

Dual Shock 4 is a bluetooth device, and it is paired with the MuSHR car number 1. Logitech F710 uses a usb dongle, so it is visually easy to find out with car is paired with an F710.

If MuSHR system is setup properly as described above, the teleoperation can be tested by starting the corresponding ROS node. Be sure to confirm the IP address of the car you are trying to test, which is preferred to be 192.168.1.1{car_number} on the lab network as described above.

Also, make sure VESC is powered with enough of battery charge. If the battery to VESC is not charged sufficiently, VESC may communicate but may not drive the motor.

Initiate an SSH, VNC, or even connect Jetson Nano directly to a monitor, and run the following commands.


```console
#initiate mushr_noetic docker container if not started
mushr_noetic

#start teleoperation
roslaunch mushr_base teleop.launch
```

By default, the safty lock is the left bumper switch.