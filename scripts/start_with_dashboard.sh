#!/bin/bash

echo "Starting RoboCup system with dashboard enabled..."

# Set your Mac's IP address here
export DASHBOARD_IP="192.168.4.77"   # Change this to your Mac's IP address
export DASHBOARD_PORT="8080"

echo "Dashboard will send data to: $DASHBOARD_IP:$DASHBOARD_PORT"

# Start the normal robot system
./scripts/start.sh "$@" 