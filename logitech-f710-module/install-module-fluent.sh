#!/bin/bash
# Copy the Logitech HID module with F710 support to the lib/module location
# Which device is root mounted?
# We want to check if this is on the eMMC/SD Card (boot device) or if the rootfs has been
# pivoted
echo "Modified for Fluent Robotics Lab. by Jeeho Ahn"
echo "original script from https://github.com/jetsonhacks/logitech-f710-module"
ROOT_DEVICE=$( findmnt -n -o SOURCE --target / )
echo "Installing on root device: $ROOT_DEVICE"

# Load release and revision
JETSON_L4T_RELEASE=$(echo $JETSON_L4T_ARRAY | cut -f 1 -d '.')
JETSON_L4T_REVISION=${JETSON_L4T_ARRAY#"$JETSON_L4T_RELEASE."}
# Write Jetson description
JETSON_L4T="$JETSON_L4T_RELEASE-$JETSON_L4T_REVISION"

JETSON_L4T_FIXED=bin/l4t-32-7-2
echo "Installing from: $JETSON_L4T_FIXED"


INSTALL_DIRECTORY=/lib/modules/$(uname -r)/kernel/drivers/hid


cd $JETSON_L4T_FIXED

sudo cp -v hid-logitech.ko $INSTALL_DIRECTORY
sudo depmod -a
echo "Installed hid-logitech Module"
echo "Please reboot"
