#!/bin/bash

sysconfdir="/etc/sysconfig"
if [ -e $sysconfdir/ecathardware ]; then
  . $sysconfdir/ecathardware
else
  echo "Error: $sysconfdir/ecathardware not found."
  exit 1
fi

ec_status="$(/etc/init.d/ethercat status)"
#echo $ec_status
if [[ "${ec_status}" != *"running"* ]]; then
/etc/init.d/ethercat start
else
/etc/init.d/ethercat stop
/etc/init.d/ethercat start
fi


#git_branch="$(git branch 2> /dev/null | grep '^*' | colrm 1 2 | xargs -I BRANCH echo -n ""
#
#if [[ "$git_branch" != "ec_io_domain"  ]]; then
#  git checkout ec_io_domain
#fi
num_slaves=$REQUIRED_NUM_SLAVES ## Number of slaves to be configured, must be specfied in the config file in /etc/sysconfig
num_preop_slaves=0

while ((num_preop_slaves != num_slaves))
do

sleep .5

slave_ls="$(ethercat sl)"

if [[ -z "$slave_ls" ]]; then
  continue
fi

if [[ $slave_ls == *\?* ]]; then
  continue;
fi

num_preop_slaves=$(echo "$slave_ls" | tr " " "\n" | grep -c "PREOP")

done

# Define the path to the diff_drive_hardware file
DIFF_DRIVE_HARDWARE="${HARDWARE_EXEC_PATH}/diff_drive_hardware"

# Check if the file exists and is executable
if [ -x "$DIFF_DRIVE_HARDWARE" ]; then
  # Execute the diff_drive_hardware file
  .${DIFF_DRIVE_HARDWARE}
else
  echo "Error: $DIFF_DRIVE_HARDWARE not found or not executable."
  exit 1
fi
