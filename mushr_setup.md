---
title: MuSHR Setup Guide - Fluent Robotics Lab
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
- Building Maps
- Localization
- Motion Planning and Navigation

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

## Wireless Network for MWireless (to be changed for internal network)

## Remote Development Environment 

### on VNC

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

### on SSH

Remote operations over SSH can be setup as usual. (details to be added)

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
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

We can verify the fix in previlage with the following command.
```
docker run hello-world
```

### Running MuSHR Noetic Docker Image

For every new terminal session, run the following command to initiate the docker image.
```
mushr_noetic
```

