# ROS 2 Workspace Docker Setup

This guide provides step-by-step instructions to build and run a ROS 2 workspace inside a Docker container, install necessary dependencies, and run ROS 2 nodes.

## 1. Build the Docker Image
Navigate to the ROS 2 workspace directory and build the Docker image:

```bash
cd /ros2_ws
docker build -t ros2_ws .
```

## 2. Run the Docker Container
Run the container with the necessary devices and environment variables:

```bash
docker run -it --rm \
    --net=host \
    --privileged \
    --device=/dev/i2c-1 \
    --device=/dev/spidev0.0 \
    --device=/dev/spidev0.1 \
    --device=/dev/ttyUSB0 \
    --env ROS_DOMAIN_ID=0 \
    -v /home/core/Arducam_tof_camera:/Arducam_tof_camera \
    -v /home/core/ros2_ws:/ros2_ws \
    ros2_ws
```

## 3. Install Dependencies
Inside the container, install the required Python packages:

```bash
sudo pip install sparkfun-qwiic-icm20948
```

Navigate to the Arducam ToF Camera example directory and install additional dependencies:

```bash
cd /Arducam_tof_camera/example/python
pip install -r requirements.txt
```

## 4. Fix ROS 2 Sensor Messages Issue
If you encounter issues with `Arducam_tof_camera`, update your system and install the necessary ROS 2 package:

```bash
sudo apt update
sudo apt install ros-humble-sensor-msgs-py
```

## 5. Set Up ROS 2 Domain ID
On all terminals, set the ROS 2 domain ID to ensure communication between nodes:

```bash
export ROS_DOMAIN_ID=7
```

## 6. Build the ROS 2 Workspace
Run the following commands to build the ROS 2 workspace:

```bash
colcon build --symlink-install
source install/setup.sh
```

## 7. Run ROS 2 Nodes
### On the Raspberry Pi:
```bash
ros2 run raw_data_publisher tof_publisher
```

### On the Host PC:
Build the depth_processor package and source it.
```bash
colcon build
source install/setup.sh
ros2 run depth_processor process_depth
rviz2
```

This setup ensures a properly configured ROS 2 workspace running inside Docker with all necessary dependencies installed. Happy coding!

