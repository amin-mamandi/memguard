#!/bin/bash

# Remove existing module if loaded
sudo rmmod memguard 2>/dev/null || true

cd ../memguard
# Build memguard module
make clean
make

# Load memguard module with parameters
sudo insmod memguard.ko 


sudo insmod memguard.ko 
# Monitor raw events
# watch -n 1 'sudo cat /sys/kernel/debug/memguard/raw_events'


echo 100 | sudo tee /sys/module/memguard/parameters/g_write_events_threshold
echo 10000 | sudo tee /sys/module/memguard/parameters/g_tor_lat_threshold
echo 3000 | sudo tee /sys/module/memguard/parameters/g_wpq_lat_threshold