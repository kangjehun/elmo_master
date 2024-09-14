#!/bin/bash

print_error() {
    echo -e "\e[31m\e[1m$1\e[0m"  # Red color, bold font
}
print_success() {
    echo -e "\e[32m\e[1m$1\e[0m"  # Green color, bold font
}

# Step 1 : Check if Peak USB driver is loaded
echo "Checking if peak_usb driver is loaded..."
if lsmod | grep -q '^peak'; then
    echo "peak_usb module is already loaded."
else
    echo "Trying to load peak_usb driver..."
    sudo modprobe peak_usb
    if ! lsmod | grep -q '^peak_usb'; then
        print_error "Error: Failed to load peak_usb driver."
        exit 1
    fi
fi

# # Step 2 : Set CAN0 interface
# echo "Checking if CAN0 interface is already up..."
# if ip link show can0 | grep -q "state UP"; then
#     echo "CAN0 interface is already up."
# else
#     echo "Setting up CAN0 interface..."
#     sudo ip link set can0 up type can bitrate 500000 # 500kbps
# fi

# Step 3 : Set CAN1 interface
echo "Checking if CAN1 interface is already up..."
if ip link show can1 | grep -q "state UP"; then
    echo "CAN1 interface is already up."
else
    echo "Setting up CAN1 interface..."
    sudo ip link set can1 up type can bitrate 500000 # 500kbps
fi

# Step 4 : Verify CAN0 channel ID
# echo "Checking CAN0 channel ID..."
# CAN0_CHANNEL_ID=$(cat /sys/class/net/can0/peak_usb/can_channel_id 2>/dev/null)
# if [ "$CAN0_CHANNEL_ID" != "00000000" ]; then
#     print_error "Error: CAN0 channel ID is incorrect or not available."
#     exit 1
# else 
#     echo "CAN0 channel ID is correct."
# fi

# Step 5: Verify CAN1 channel ID
echo "Checking CAN1 channel ID..."
CAN1_CHANNEL_ID=$(cat /sys/class/net/can1/peak_usb/can_channel_id 2>/dev/null)
if [ "$CAN1_CHANNEL_ID" != "00000001" ]; then
    print_error "Error: CAN1 channel ID is incorrect or not available."
    exit 1
else
    echo "CAN1 channel ID is correct."
fi

print_success "Peak USB driver setup completed."
