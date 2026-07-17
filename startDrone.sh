#!/bin/bash
export ROS_DOMAIN_ID=99
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General></Domain></CycloneDDS>'

# ensure dockedr0 is down
sudo ip link set docker0 down

# kill any leftover processes from previous runs
echo "Cleaning up previous processes..."
pkill -9 -f "gz sim"
pkill -9 -f "gz-sim"
pkill -9 -f "ruby.*gz"
pkill -9 -f "ruby"
pkill -9 -f px4
pkill -9 -f MicroXRCEAgent
sleep 2

# kill background jobs and gz-sim server when script exits
cleanup() {
    echo "Cleaning up on exit..."

    echo "Waiting for gz-sim to fully stop..."
    for i in $(seq 1 60); do
        pkill -9 -f "gz sim" 2>/dev/null
        sleep 1
        if ! pgrep -f "gz sim" > /dev/null; then
            echo "gz sim confirmed stopped after ${i}s."
            break
        fi
    done

    # Now safe to clean up any other background jobs (Agent, QGC, etc.)
    kill 0 2>/dev/null
}
trap cleanup EXIT

# 1. Start the Agent in the background
echo "Starting Agent..."
MicroXRCEAgent udp4 -p 8888 &
sleep 2

# 2. Start QGroundControl in the background
echo "Starting QGC..."
~/bin/QGroundControl.AppImage &
sleep 5  # Give QGC a full 5 seconds to settle

# 3. Start PX4 and Gazebo in the FOREGROUND of this exact terminal
echo "Starting PX4 and Gazebo..."
cd ~/droneSITL/PX4-Autopilot
make px4_sitl gz_x500_depth_down
