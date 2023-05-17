#!/bin/bash

echo "Setting up botlab lite"

MBOT_USER=pi 
TIMESTAMP=$(date "+%y%m%d_%H%M%S")  # For log file
LOG_DIR=/home/$MBOT_USER/.logs

if [ ! -d $LOG_DIR ]; then
    mkdir $LOG_DIR
fi

echo "Logging to: $LOG_DIR"

echo "Cleaning up any running MBot code."
pkill slam
pkill motion_controll
pkill timesync
pkill rplidar_driver
pkill pico_shim

echo "Launching timesync, motion_controller and shim."
./bin/timesync &> $LOG_DIR/timesync.log &
./bin/rplidar_driver &> $LOG_DIR/rplidar_driver.log &
./bin/pico_shim &> $LOG_DIR/pico_shim.log &
./bin/motion_controller &> $LOG_DIR/motion_controller.log &