#!/bin/bash
set -Eeuo pipefail

sysconfdir="/etc/sysconfig"
if [ -e $sysconfdir/ecathardware ]; then
  . $sysconfdir/ecathardware
else
  echo "Error: $sysconfdir/ecathardware not found."
  exit 1
fi

# Define the path to the diff_drive_hardware file
DIFF_DRIVE_HARDWARE="${HARDWARE_EXEC_PATH}/diff_drive_hardware"

# Check if the file exists and is executable
if [ -x "$DIFF_DRIVE_HARDWARE" ]; then
  # Execute the diff_drive_hardware file
  ./${DIFF_DRIVE_HARDWARE}/diff_drive_hardware
else
  echo "Error: $DIFF_DRIVE_HARDWARE not found or not executable."
  exit 1
fi