#!/bin/bash



if [[ -z $ROS_DISTRO ]]; then

  source /opt/ros/humble/setup.bash
  source /home/upxtreme/air_ws/install/setup.bash
  
fi

ldconfig


sysconfdir="/etc/sysconfig"
if [ -e $sysconfdir/ecathardware ]; then
  . $sysconfdir/ecathardware
else
  echo "Error: $sysconfdir/ecathardware not found."
  exit 1
fi

echo "Found configuration file"

# Define the path to the diff_drive_hardware file
DIFF_DRIVE_CONTROLLER="${HARDWARE_EXEC_PATH}/diff_drive_controller"

# Check if the file exists and is executable
if [ -x "$DIFF_DRIVE_CONTROLLER" ]; then
  # Execute the DIFF_DRIVE_CONTROLLER file
  echo "Starting controller"
  .${DIFF_DRIVE_CONTROLLER}
else
  echo "Error: $DIFF_DRIVE_CONTROLLER not found or not executable."
  exit 1
fi

echo "Controller started"
