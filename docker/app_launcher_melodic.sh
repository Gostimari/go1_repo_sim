#!/bin/bash

# Variable to store the PID of the last command executed
COMMAND_PID=""

# Function to execute commands based on button clicked
execute_command() {
    case $1 in
        1)
            echo "Traversability clicked"
            setsid roslaunch traversability_mapping offline.launch &
            COMMAND_PID=$!
            ;;
        2)
            echo "Kill button clicked"
            if [ -n "$COMMAND_PID" ]; then
                echo "Sending SIGINT to process group $COMMAND_PID"
                kill -SIGINT -- -$COMMAND_PID
                COMMAND_PID=""
            else
                echo "No specific process to kill."
            fi
            # Kill all ROS nodes and processes
            echo "Killing all ROS nodes and roscore..."
            rosnode kill -a 2>/dev/null  # Gracefully kill nodes
            sleep 1  # Allow time for nodes to terminate
            killall -9 roscore rosmaster rosout 2>/dev/null  # Force kill core processes
            pkill -9 -f "ros/master" 2>/dev/null   # Target rosmaster-related processes
            pkill -9 -f "ros/launch" 2>/dev/null  # Target roslaunch processes
            ;;
        252)
            echo "Window closed by user. Exiting."
            exit 0
            ;;
        *)
            echo "Unknown button clicked"
            ;;
    esac
}

# Loop to re-open the dialog after each button click
while true; do
    yad --title "Roslaunch Melodic App" \
        --form \
        --width=300 --height=200 \
        --button="Traversability:1" \
        --button="Kill:2" \
        --buttons-layout=spread \
        --text="Click a button to execute a launchfile or Kill to stop it."
    
    BUTTON_EXIT_CODE=$?
    
    # If the dialog was closed (you might check for a specific exit code), exit the loop.
    # For example, if the user presses the window close button, YAD might return 1 (this can vary).
    if [ "$BUTTON_EXIT_CODE" -eq 252 ]; then
        echo "Window closed by user. Exiting."
        exit 0
    fi
    
    execute_command $BUTTON_EXIT_CODE
done

