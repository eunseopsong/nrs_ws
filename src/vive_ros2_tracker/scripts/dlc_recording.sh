#!/bin/bash
# Start surgical_recording && comp_recording && dlc_recording with proper termination handling

# Start surgical_recording and save its PID
echo 0 | rosrun vive_ros_tracker surgical_recording &
SURGICAL_PID=$!

# Start comp_recording and save its PID
echo 0 | rosrun vive_ros_tracker comp_recording &
COMP_PID=$!

echo 0 | rosrun vive_ros_tracker dlc_recording &
DLC_PID=$!

# Function to handle termination signal (Ctrl+C)
terminate_processes() {
    echo "Terminating recording processes..."
    kill -SIGINT $SURGICAL_PID 2>/dev/null
    kill -SIGINT $COMP_PID 2>/dev/null
    kill -SIGINT $DLC_PID 2>/dev/null
    wait $SURGICAL_PID $COMP_PID $DLC_PID 2>/dev/null
    echo "Processes terminated."
    exit 0
}

# Trap Ctrl+C (SIGINT) and call the terminate function
trap terminate_processes SIGINT

# Wait for both processes to complete
wait

