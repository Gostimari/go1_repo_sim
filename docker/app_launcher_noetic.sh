#!/bin/bash

COMMAND_PID=""

execute_command() {
    case "$1" in
    	2)
            echo "Gazebo clicked"
            roslaunch unitree_gazebo robot_simulation.launch &
            COMMAND_PID=$!
            ;;
        3)
	    echo "Go1 control clicked"
            rosrun unitree_guide junior_ctrl &
            COMMAND_PID=$!
            ;;
        4)
            echo "MEBT clicked"
            roslaunch ig_lio noetic_main_mebt.launch &
            COMMAND_PID=$!
            ;;
        5)
            echo "Traversability clicked"
            roslaunch ig_lio noetic_main_trav.launch &
            COMMAND_PID=$!
            ;;
        6)
            echo "Elevation clicked"
            roslaunch ig_lio noetic_main_elev.launch &
            COMMAND_PID=$!
            ;;
        7)
            echo "Kill button clicked"
            if [ -n "$COMMAND_PID" ]; then
                echo "Sending SIGINT to process $COMMAND_PID"
                kill -SIGINT "$COMMAND_PID"
                COMMAND_PID=""
            else
                echo "No process to kill."
            fi
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
    # Run YAD and then capture its exit code immediately.
    yad --title "Roslaunch Noetic App" \
        --form \
        --width=300 --height=200 \
        --button="Gazebo:2" \
        --button="Go1 Control:3" \
        --button="MEBT:4" \
        --button="Traversability:5" \
        --button="Elevation:6" \
        --button="Kill:7" \
        --buttons-layout=spread \
        --text="Click a button to execute a command or Kill to stop the process. If you want to launch the traversability_mapping, you need to launch it on the Roslaunch Melodic App too."
    
    BUTTON_EXIT_CODE=$?
    
    # If the exit code is 1 (the default for window-close), then exit.
    if [ "$BUTTON_EXIT_CODE" -eq 252 ]; then
        echo "Window closed by user. Exiting."
        exit 0
    fi

    execute_command "$BUTTON_EXIT_CODE"
done

