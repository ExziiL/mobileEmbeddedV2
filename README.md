# README for Turtlebot 3 Project

## Overview

This project outlines the setup and usage instructions for a Turtlebot 3 robotic system. The setup involves configuring a local machine with Ubuntu 20.04 and ROS2 Foxy, setting up a Raspberry Pi with Ubuntu Server 20.04, and preparing the Turtlebot 3 hardware. Furthermore, the project includes a WebApplication for interacting with the Turtlebot 3.

## Installation Instructions

### Local Machine Setup

1. **Ubuntu 20.04 Installation:**
    - Install Ubuntu 20.04 on your local machine. You can download the ISO and follow the installation instructions from the [official Ubuntu website](https://releases.ubuntu.com/20.04/).

2. **Install ROS2 Foxy:**
    - Follow the official instructions to [install ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html) on your local machine.

### Raspberry Pi Setup

1. **Install Ubuntu Server 20.04 on Raspberry Pi:**
    - Download the Ubuntu Server 20.04 image for Raspberry Pi from the [Ubuntu for Raspberry Pi website](https://ubuntu.com/download/raspberry-pi).
    - Follow the installation instructions provided on the website to set up your Raspberry Pi.

### Turtlebot 3 Setup

1. **Set up Turtlebot 3:**
    - Follow the comprehensive guide provided by Robotis in the [Turtlebot 3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview).

### WebApplication Dependencies

1. **Install Node.js Dependencies:**
    - Navigate to the root directory of the WebApplication.
    - Run the following command to install the necessary Node.js dependencies:
        ```sh
        npm install --legacy-peer-deps
        ```

## Usage Instructions

### Raspberry Pi Operations

1. **Connect to Raspberry Pi via SSH:**
    - Use the following command to connect to your Raspberry Pi. Replace `<ip-address>` with the actual IP address of your Raspberry Pi.
        ```sh
        ssh ubuntu@<ip-address>
        ```

2. **Start the Robot:**
    - Run the `startup.sh` script to initialize the robot.
        ```sh
        ./startup.sh
        ```

3. **Run the Video Streaming Node:**
    - Start the video streaming node with the following command:
        ```sh
        ros2 run cv_basics img_publisher
        ```

### Local Machine Operations

1. **Start the ROS Bridge:**
    - Switch to your local machine and start the ROS bridge by executing:
        ```sh
        ros2 launch rosbridge_server rosbridge_websocket_launch.xml
        ```

### WebApplication Setup

1. **Prepare the WebApplication:**
    - Navigate to the WebApplication codebase directory on your local machine.

2. **Set Node.js Options:**
    - Run the following command to set the required Node.js options:
        ```sh
        export NODE_OPTIONS=--openssl-legacy-provider
        ```

3. **Run the WebApplication:**
    - Start the WebApplication with:
        ```sh
        npm run start
        ```

You should now be able to interact with the Turtlebot 3 via the WebApplication. Enjoy exploring with your robot!