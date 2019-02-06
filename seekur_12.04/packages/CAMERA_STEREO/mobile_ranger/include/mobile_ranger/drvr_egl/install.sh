#!/bin/sh
#****************************************************************************
# Copyright (c) 2009 by Focus Robotics Inc.
#
# All rights reserved. No part of this design may be reproduced stored
# in a retrieval system, or transmitted, in any form or by any means,
# electronic, mechanical, photocopying, recording, or otherwise, without
# prior written permission of Focus Robotics, Inc.
#
# Proprietary and Confidential
#
# Creation_Date :  Mon Mar 23 2009
# Created by    :  Andrew Worcester
#
#*****************************************************************************

# Kernel module install must be run as root!
my_name=`whoami`
if [ $my_name != "root" ] 
then
    echo "Please run install script as root"
    exit
fi

# Build the eagle kernel module
make

# Remove any existing fr3 modules from the kernel if they exist
match=`lsmod | grep fr3`
if [ "$match" != "" ]; then
    rmmod fr3
fi

# Remove installed fr3 modules from /lib/modules
# Could use 'find' for more robustness
rm -f /lib/modules/`uname -r`/kernel/drivers/media/video/fr3.ko

# Copy the eagle module to the /lib/modules tree
cp -f eagle.ko /lib/modules/`uname -r`/kernel/drivers/media/video/

# Update modules dependencies
depmod -ae

# Insert the new module
#/sbin/insmod eagle.ko
modprobe eagle

# Make the eagle device file if it doesn't exist
if [ ! -e /dev/eagle ]; then
    mknod /dev/eagle c 240 0
    chmod a+rw /dev/eagle
fi

# Make sure the eagle device file gets made every time the computer starts
# udev will handle this in the future but it must be handled manually for now
if [ -e "/etc/rc.d/rc.local" ]; then
    local_service_file=/etc/rc.d/rc.local
elif [ -e "/etc/init.d/rc.local" ]; then
    local_service_file=/etc/init.d/rc.local
elif [ -e "/etc/conf.d/local.start" ]; then
    local_service_file=/etc/conf.d/local.start
else
    echo What is the local init file on this system
    exit 3
fi
echo "if [ ! -e /dev/eagle ]; then" >> $local_service_file
echo "    mknod /dev/eagle c 240 0" >> $local_service_file
echo "    chmod a+rw /dev/eagle" >> $local_service_file
echo "fi" >> $local_service_file




