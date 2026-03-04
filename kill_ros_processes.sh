#!/bin/bash
pkill -9 -f ros && ros2 daemon stop

echo "Finding processes in /opt/ros/jazzy..."
PIDS=$(ps aux | grep '/opt/ros/jazzy' | grep -v grep | awk '{print $2}')

if [ -z "$PIDS" ]; then
    echo "No ROS processes found."
    exit 0
fi

echo "Killing PIDs: $PIDS"
echo "$PIDS" | xargs -r sudo kill -9

echo "Verifying..."
ps aux | grep '/opt/ros/jazzy' | grep -v grep