#!/bin/bash

COMMAND_PID=""

execute_command() {
    case "$1" in
        2)
            echo "MEBT clicked"
            setsid roslaunch ig_lio noetic_main_mebt.launch &
            COMMAND_PID=$!
            ;;
        3)
            echo "Traversability clicked"
            setsid roslaunch ig_lio noetic_main_trav.launch &
            COMMAND_PID=$!
            ;;
        4)
            echo "Elevation clicked"
            setsid roslaunch ig_lio noetic_main_elev.launch &
            COMMAND_PID=$!
            ;;
        5)
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
            echo "Unknown button clicked: $1"
            ;;
    esac
}

while true; do
    yad --title "Roslaunch Noetic App" \
        --form \
        --width=300 --height=200 \
        --button="MEBT:2" \
        --button="Traversability:3" \
        --button="Elevation:4" \
        --button="Kill:5" \
        --buttons-layout=spread \
        --text="Click a button to execute a command or Kill to stop the process. If you want to launch the traversability_mapping, you need to launch it on the Roslaunch Melodic App too."
    
    BUTTON_EXIT_CODE=$?
    
    if [ "$BUTTON_EXIT_CODE" -eq 252 ]; then
        echo "Window closed by user. Exiting."
        exit 0
    fi

    execute_command "$BUTTON_EXIT_CODE"
done
