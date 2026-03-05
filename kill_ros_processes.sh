#!/bin/bash
echo "Finding processes in /opt/ros/jazzy..."
PIDS=$(ps aux | grep '/opt/ros/jazzy' | grep -v grep | awk '{print $2}')

echo "Finding gz sim gui processes..."
GZ_GUI_PIDS=$(ps aux | grep -E 'gz[[:space:]]+sim[[:space:]]+gui|gz_sim_gui|gzserver gui' | grep -v grep | awk '{print $2}')
# Also catch the "gz sim gui" process as shown by htop (process name style)
GZ_GUI_PIDS2=$(ps aux | grep -E '\bgz\b' | grep -E '\bgui\b' | grep -v grep | awk '{print $2}')

echo "Finding gz sim server processes..."
GZ_SERVER_PIDS=$(ps aux | grep -E 'gz[[:space:]]+sim[[:space:]]+server|gz_sim_server|gzserver' | grep -v grep | awk '{print $2}')
# Also catch "gz sim server" process as shown by htop (process name style)
GZ_SERVER_PIDS2=$(ps aux | grep -E '\bgz\b' | grep -E '\bserver\b' | grep -v grep | awk '{print $2}')

# Combine and deduplicate all PIDs
ALL_PIDS=$(echo -e "$PIDS\n$GZ_GUI_PIDS\n$GZ_GUI_PIDS2\n$GZ_SERVER_PIDS\n$GZ_SERVER_PIDS2" | sort -u | tr '\n' ' ' | xargs)

if [ -z "$ALL_PIDS" ]; then
    echo "No ROS or Gazebo processes found."
    exit 0
fi

echo "Killing PIDs: $ALL_PIDS"
echo "$ALL_PIDS" | xargs -r kill -9

echo "Verifying..."
echo "--- /opt/ros/jazzy processes ---"
ps aux | grep '/opt/ros/jazzy' | grep -v grep
echo "--- gz sim processes ---"
ps aux | grep -E '\bgz\b' | grep -v grep

echo "Stopping any router..."
pkill -9 -f ros; ros2 daemon stop