#!/bin/bash

set -e  # Exit on error

echo "Installing EtherCAT Motor Control System dependencies..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root"
    exit 1
fi

# Function to check command success
check_status() {
    if [ $? -ne 0 ]; then
        echo "Error: $1"
        exit 1
    fi
}

# Build the project
echo "Building the project..."
if [ ! -d "build" ]; then
    mkdir build
fi
cd build
cmake ..
cmake --build . --target install
check_status "Failed to build project"

echo "Installation complete!"
