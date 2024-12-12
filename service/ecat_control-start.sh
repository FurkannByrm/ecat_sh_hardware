#!/bin/bash
set -Eeo pipefail

if [[ -z $ROS_DISTRO ]]; then

  source /opt/ros/humble/setup.bash

fi

sysconfdir="/etc/sysconfig"
if [ -e $sysconfdir/ecathardware ]; then
  . $sysconfdir/ecathardware
else
  echo "Error: $sysconfdir/ecathardware not found."
  exit 1
fi

# Define the path to the diff_drive_hardware file
DIFF_DRIVE_CONTROLLER="${HARDWARE_EXEC_PATH}/diff_drive_controller"

# Check if the file exists and is executable
if [ -x "$DIFF_DRIVE_CONTROLLER" ]; then
  # Execute the DIFF_DRIVE_CONTROLLER file
  ./${DIFF_DRIVE_CONTROLLER}
else
  echo "Error: $DIFF_DRIVE_CONTROLLER not found or not executable."
  exit 1
fi