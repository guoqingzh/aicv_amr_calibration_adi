# IntelLabs AMR calibration

# Get the code and create the docker:
- Clone the repo: $ git clone git@github.com:intel-sandbox/aicv_amr_calibration.git
- Create the docker image and launch it: $ ./launch_calibDocker.sh
## Rosbag recording
- Launch the script and follow the instructions to record the rosbag: $ ./record_rosbag.sh
- Close the visualization window to stop the recording process
- The rosbag is saved in /workspace/bags
## Performing calibration
- Launch the script and follow the instructions to calibrate the appropriate sensors: ./calibrate.sh
- The calibration requies the user to move the calibration board (consisting of AprilTags), in different orientations.
- The calibration output is a transformation graph between the selected sensor nodes that is saved in /workspace/data as a (.json) file.
## Verifying calibration using pointclouds
- The pointcloud data can be extracted from a given rosbag for verification of the multi-camera calibration.
- This can be achieved by passing as argument input rosbag (--input) whose pointcloud data is to be analyzed, and the filepath of the transformation graph (--transform) for a previous calibration done with the same camera setup.
