# Installation
    1. install Ubuntu20.04 on local machine
    2. install ROS2 Foxy 
    3. install Ubuntu Server 20.04 on Raspberry Pi 
    4. set up Turtlebot 3 (https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview)
    5. install the dependencies for the WebApplication `npm install --legacy-peer-deps`

# Usage
    1. connect to Raspberrry Pi via ssh `ssh ubuntu@<ip-address>`
    2. Start up robot with `./startup.sh` skript
    3. run the video streaming node `ros2 run cv_basics img_publisher` 
    4. change to local machine
    5. start up the ros bridge `ros2 launch rosbridge_server            rosbridge_websocket_launch.xml`
    6. go to web application codebase
    7. export NODE_OPTIONS `export NODE_OPTIONS=--openssl-legacy-provider`
    8. run the application `npm run start`