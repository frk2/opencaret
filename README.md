# Intro
An opensource / opendata Level-3 Highway autopilot system for all modern cars. Think Tesla Autopilot but for your civic, corolla, kia. The idea is to initially perfect this on the Kia Soul EV (since Polysync graciously gives us a opensource/openhardware drive by wire kit) and then build open hardware drive by wire kits for other cars.

# Getting started

## Hardware requirements
Everything has been tested on real hardware. I know its possible to setup fake interfaces (like a vcan0) device but none of that has been tested.

### Can to USB covertor
You need a CAN to USB convertor. I currently use the one from canable.io which is excellent if you flash the candlelight firmware onto it. The default SLCAN implementation wasn't usable for me. Once thats done you can start up the CAN interface on your system using:

`sudo ip link  set can0 up type can bitrate 500000`

### Toyota 2016-2017 Radar from Denso
You can pick these up on ebay. We will eventually support many radars but this one is easily available and known to work well!

## Software Requirements
Opencaret is being developed and tested on:
- ROS2 (master)
- Ubuntu 16.04/18.04

Opencaret is built around ROS2. You need to get ROS2 from master since ardent has a bug with the launch system thats being worked on.

ROS2: https://github.com/ros2/ros2/wiki/Linux-Development-Setup

You also need to get all submodules after you pull:

`git submodule update --init --recursive`

to build:

`ament build --simlink-install`

then source the ros setup file:

`. ./install/setup.sh`


The rest assumes that you have can0 up and running. The CAN setup is designed to start listening on all canX interfaces on your linux system.

launch the entire system. This currently means:

- Working CAN bus controller
- Working Radar controller

`launch src/launch/all_launch.launch`

you can observe radar topics with:
`ros2 topic echo /radar_tracks`

