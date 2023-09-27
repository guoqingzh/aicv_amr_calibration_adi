#!/bin/bash

echo "---------------------------------"
echo "AMR Calibration (recording) tool"
echo "---------------------------------"


####---------------------- Input handling ------------------------------------

# Default variables
output_file=""
sensors=1

# Function to display
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo " -h, --help           :   Display this help message"
    echo " -f, --filename       :   Name of the output rosbag file (without extension) to be saved in /workspace/bags"
    echo " -s, --sensors        :   Recording to be used for calibrating which sensors (default <1>)"
    echo "          1           :   cam2cam  - record ROS topics <IR, RGB>"
    echo "          2           :   cam2imu  - record ROS topics <IR, RGB, IMU>"
    echo "          3           :   cam2odom - record ROS topics <IR, RGB, odom>"
    echo " Sample command       :   $0 --filename outputBag --sensors 1"
}


has_argument() {
    [[ ("$1" == *=* && -n ${1#*=}) || ( ! -z "$2" && "$2" != -*)  ]];
}


extract_argument() {
  echo "${2:-${1#*=}}"
}

# Function to handle options and arguments
handle_options() {
    if [ $# -le 1 ]; then
        usage
        exit 0
    fi
    while [ $# -ge 1 ]; do
        case $1 in
            -h | --help)
                usage
                exit 0
                ;;
            -f | --output_file*)
                if ! has_argument $@; then
                    echo "Error: File not specified" >&2
                    usage
                    exit 1
                fi

                output_file=$(extract_argument $@)

                shift
                ;;
            -s | --sensors)
                if ! has_argument $@; then
                    echo "Error: sensor option not specified" >&2
                    usage
                    exit 1
                fi

                sensors=$(extract_argument $@)

                if [ $sensors -lt 1 ] || [ $sensors -gt 3 ]; then
                    echo "Error: Invalid sensor option specified" >&2
                    usage
                    exit 1
                fi

                shift
                ;;
            *)
                echo "Error: Invalid option selected $1" >&2
                usage
                exit 1
                ;;
        esac
        shift
    done
}


# Main script execution
handle_options "$@"

if [ -n "$output_file" ]; then
    echo "Output file specified: $output_file"
else
    echo "Error: Output filename not specified"
    usage
    exit 1
fi



####---------------------- ROS camera and Rviz launch ------------------------------------

echo ""
echo "Starting realsense cameras"
echo -e "---------------------------- \n"
sleep 2


# Check if the /opt/ros directory exists
if [ -d "/opt/ros" ]; then
    # List the contents of /opt/ros and find the ROS version folder
    ros_version=$(ls /opt/ros)

    if [ -n "$ros_version" ]; then
        echo "Found ROS version: $ros_version"
        # Source the setup.bash file for the detected ROS version
        #source "/opt/ros/$ros_version/setup.bash"
    else
        echo "No ROS version found in /opt/ros."
    fi
else
    echo "/opt/ros directory does not exist. Make sure ROS is installed."
fi


# Check if the RealSense SDK is installed
if ! command -v rs-enumerate-devices &> /dev/null; then
    echo "Intel RealSense SDK is not installed. Please install it before running this script."
    exit 1
fi

# Run rs-enumerate-devices and extract serial numbers using awk
cameraCtr=0
pid=()

while IFS= read -r serial_number; do
    cameraSNo='_'$serial_number
    cameraName='camera'$cameraCtr
    echo "Launching $cameraName: $cameraSNo"
    ros2 launch realsense2_camera rs_launch.py camera_name:=$cameraName serial_no:=$cameraSNo enable_color:=true enable_infra1:=true enable_infra2:=true enable_depth:=true pointcloud.enable:=true enable_gyro:=true enable_accel:=true unite_imu_method:=2&
    pid+=($!)
    cameraCtr=$((cameraCtr+1))
done < <(rs-enumerate-devices -s | awk '/Intel RealSense/ {print $4}')


# Print all stored PIDs
echo "All PIDs: ${pid[@]}"

echo ""
echo "Starting Rviz"
echo -e "---------------------------- \n"
#sleep 2
ros2 run rviz2 rviz2&


rviz_pid=$!

# Wait 10 seconds for all the realsense launch camera logs to be populated on the terminal
sleep 8

# Clear the screen for displaying the next set of instructions to the user
clear >$(tty)


####---------------------- ROSbag recording ------------------------------------
echo ""
echo "Recording rosbag"
echo -e "---------------------------- \n"


rostopic_opts=" /tf_static "
for ((cameraIdx=0; cameraIdx<$cameraCtr; cameraIdx++))
do
    curCamera="camera$cameraIdx"
    case $sensors in

        1)
            rostopic_opts+="/$curCamera/color/camera_info /$curCamera/color/image_raw /$curCamera/infra1/camera_info /$curCamera/infra1/image_rect_raw /$curCamera/infra2/camera_info /$curCamera/infra2/image_rect_raw "
            ;;
        2)
            rostopic_opts+="/$curCamera/color/camera_info /$curCamera/color/image_raw /$curCamera/infra1/camera_info /$curCamera/infra1/image_rect_raw /$curCamera/infra2/camera_info /$curCamera/infra2/image_rect_raw /$curCamera/imu "
            ;;
        3)
            rostopic_opts+="/$curCamera/color/camera_info /$curCamera/color/image_raw /$curCamera/infra1/camera_info /$curCamera/infra1/image_rect_raw /$curCamera/infra2/camera_info /$curCamera/infra2/image_rect_raw /odom "
            ;;
    esac
done
#echo $rostopic_opts

bagRecordPID=0
while true; do
    echo "Press r/R to begin recording. The recording can be stopped by closing the RViz visualization window"
    read -n1 userInput
    output_file_path="/workspace/bags/$output_file"
    if [ "$userInput" == "r" ]; then
        ros2 bag record -o $output_file_path $rostopic_opts&
        echo "Rosbag recording started ..."
        bagRecordPID=($!)
        break
    fi
done


# Check if the RVIZ window is no longer running
while true; do
    if ! ps -p $rviz_pid > /dev/null; then

        echo "Rosbag recording finished."
        # Stop recording the rosbag before killing the remaining processes
        kill -INT $bagRecordPID
        # Terminate previous camera launch processes
        for pid_process in "${pid[@]}"; do
            sleep 2
            kill -9 $pid_process
        done
        # TBD: Another process still running (realsense-camera-node), check how to kill it too.
        break
    fi
    # Sleep for a short interval before checking again
    sleep 3
done

