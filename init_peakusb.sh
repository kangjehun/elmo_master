#!/bin/bash

# Step 1: Load the Peak USB driver
echo "Loading peak_usb driver..."
sudo modprobe peak_usb

# Step 2 : Set CAN1 interface
echo "Setting up CAN1 interface..."
sudo ip link set can1 type can bitrate 500000 # 500 kbit/s
sudo ip link set up can1

# Step 3 : Check if Peak USB driver is loaded
echo "Checking if peak_usb driver is loaded..."
if lsmod | grep -q '^peak'; then
    echo "peak_usb module is loaded."
else
    echo "Error: peak_usb module is not loaded."
    exit 1
fi

# Step 4 : Verify CAN0 channel ID
echo "Checking CAN0 channel ID..."
CAN0_CHANNEL_ID=$(cat /sys/class/net/can0/peak_usb/can_channel_id 2>/dev/null)
if ["$CAN0_CHANNEL_ID" == "00000000"]; then
    echo "Error: CAN1 channel ID is incorrect or not available."
    exit 1
else 
    echo "CAN1 channel ID is correct."
fi

# Step 5: Verify CAN1 channel ID
echo "Checking CAN1 channel ID..."
CAN1_CHANNEL_ID=$(cat /sys/class/net/can1/peak_usb/can_channel_id 2>/dev/null)
if [ "$CAN1_CHANNEL_ID" != "00000001" ]; then
    echo "Error: CAN1 channel ID is incorrect or not available."
    exit 1
else
    echo "CAN1 channel ID is correct."
fi

echo "Peak USB driver setup completed."
