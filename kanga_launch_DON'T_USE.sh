#!/bin/bash


echo "Package List:
1. Rtabmap
2. Multi Camera Node
3. Initialise ODrive Node
4. "

while true; do
    read -p "Enter a number next to the package (type 'q' to quit): " user_input

    if [[ "$user_input" == "q" ]]; then
        echo "Goodbye!"
        break
    fi


    #PLEASE ADD MORE CASE STATEMENTS IF WE NEED THEM!
    case $user_input in
        "1")
        gnome-terminal --title="rtabmap" --tab -- bash -c "ros2 launch rtabmap_examples realsense_d435i_color.launch.py; exec bash"
        ;;
        "2")
        gnome-terminal --title="multi-camera-node" --tab -- bash -c "cd cameras; source install/setup.bash; ros2 run multi_camera_node multi_camera_node; exec bash"
        ;;
        "3")
        gnome-terminal --title="drivetrain-control" --tab -- bash -c "./kangaStartUp.sh; exec bash"
        ;;
        *)
        echo "Invalid Input"

    esac

done