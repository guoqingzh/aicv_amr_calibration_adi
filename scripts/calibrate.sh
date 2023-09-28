#!/bin/bash

echo "---------------------"
echo "AMR calibration tool"
echo "---------------------"

####---------------------- Input handling ------------------------------------

# Default variables
sensors=1
input_file=""
tf_file=""

# Function to display
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo " -h, --help           :   Display this help message"
    echo " -i, --input          :   Name of the input rosbag file (without extension)"
    echo " -s, --sensors        :   Sensors between which the calibration is to be performed (default <1>)"
    echo "          1           :   cam2cam  - Multi-camera calibration"
    echo "          2           :   cam2imu  - Camera-IMU calibration"
    echo "          3           :   cam2odom - Camera-Odometry calibration"
    echo " -tf, --transform     :   Name of the calibration output (.json) file (present in /workspace/data)"
    echo "                          Generates pointcloud files for verifying cam2cam calibration"
    echo ""
    echo "Sample command (calibration)  :   $0 --input inputRosBag --sensors 1"
    echo "Sample command (pointcloud)   :   $0 --input inputRosBag --transform transform_graph_m600.json"
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
            -i | --input*)
                if ! has_argument $@; then
                    echo "Error: Input file not specified" >&2
                    usage
                    exit 1
                fi

                input_file=$(extract_argument $@)

                shift
                ;;
            -tf | --transform*)
                if ! has_argument $@; then
                    echo "Error: Transform graph file not specified" >&2
                    usage
                    exit 1
                fi

                tf_file=$(extract_argument $@)

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


#if [ -n "$input_file" ]; then
#    echo "Input file specified: $input_file"
#else
#    echo "Error: Input filename not specified"
#    ls /workspace/bags
#    echo ""
#    usage
#    exit 1
#fi


#if [ -n "$tf_file" ]; then
#    echo "Transform file specified: $tf_file"
#else
#    echo "Error: Transform filename not specified"
#    ls /workspace/data
#    echo ""
#    usage
#    exit 1
#fi



# Perform the desired actions based on the provided flags and arguments
dataset_path="/workspace/datasets/$input_file"
echo -e "Input rosbag: $dataset_path"

# First, generate the config file for the given ros2 bag
echo 'Reading ros2 bag metadata, generating config file...'
python3 /home/tools/data-collection/src/data_collection/generate_config.py --bag /workspace/bags/$input_file --ignore-warnings --output /workspace/bags/$input_file_config.yaml

# Second, using the config file convert the ros2 bag to dataset
echo 'Extracting ros2 bag data...'
python3 /home/tools/data-collection/src/data_collection/bag_converter.py --bag /workspace/bags/$input_file --config_file /workspace/bags/$input_file_config.yaml --ignore-warnings --dataset_path $dataset_path --maximum_delay 0.050

if [ -n "$tf_file" ]; then
    # generate pointclouds
    # Create the pointcloud folder for storing the pointcloud results in case it has not been created yet
    mkdir -p pointcloud_op/$input_file

    calib_transform_file="/workspace/data/$tf_file"
    echo -e "Calibration transform: $tf_file \n"

    outputPath="./pointcloud_op/$input_file/"
    python3 extractPCloudcsv.py $dataset_path $calib_transform_file $outputPath
else
    # perform sensor calibration
    # Create the data folder for our calibration results in case it has not been created yet
    mkdir -p /workspace/data

    # Load the dataset and calibrate each camera channel intrinsics
    echo 'Calibrating intrisics'
    python3 /home/tools/scripts/calibrate_intrinsics.py $dataset_path --start 0 --step 10 --camera_model BrownConrady --refine_corners --target_config_file ./config/kalibr_target.yaml

    # Load the dataset, the previous intrinsics and calibrate the camera extrinsics
    echo 'Calibrating extrisics'
    python3 /home/tools/scripts/calibrate_extrinsics.py $dataset_path --start 0 --step 10 --camera_model BrownConrady --load_prior_intrinsics --refine_corners --target_config_file ./config/kalibr_target.yaml
fi
