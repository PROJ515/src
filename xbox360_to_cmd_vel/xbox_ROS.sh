#!/bin/bash
#echo "please enter password"
stty_orig=`stty -g` # save original terminal setting.
stty -echo          # turn-off echoing.
#read password         # read the password
#stty $stty_orig     # restore terminal setting.
password="ms-dos"
#echo ${password} | sudo -S xboxdrv --silent -o deadzone=7000 --axismap X1=X2,Y1=Y2,X2=X1,Y2=Y1
echo ${password} | sudo -S xboxdrv --detach-kernel-driver --silent -o deadzone=12000 #--axis-map X1=Y1&
stty $stty_orig
#echo "${OUTPUT}"
