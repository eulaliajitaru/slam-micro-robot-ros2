#!/bin/bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

LIDAR_PORT=""
for port in /dev/ttyUSB0 /dev/ttyUSB1; do
    if [ -e "$port" ]; then
        LIDAR_PORT=$port
        break
    fi
done

if [ -z "$LIDAR_PORT" ]; then
    echo "EROARE: LIDAR nu e gasit pe USB!"
    exit 1
fi

echo "LIDAR pe: $LIDAR_PORT"

ros2 run rplidar_ros rplidar_composition \
    --ros-args \
    -p serial_port:=$LIDAR_PORT \
    -p frame_id:=laser_frame \
    -p angle_compensate:=true \
    -p scan_mode:=Standard &
LIDAR_PID=$!
echo "RPLIDAR pornit (PID $LIDAR_PID)"

sleep 2

ros2 run robot_creier hardware_interface &
echo "Motor driver pornit"

echo ""
echo "=== ROBOT PORNIT ==="
echo "Pe laptop: ros2 launch robot_descriere slam_physical.launch.py"

wait $LIDAR_PID
