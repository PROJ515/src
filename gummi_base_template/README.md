# gummi_base_template

This replaces the old gummi_description package, now separated into gummi_base_XXX and gummi_ee_XXX for easier change of end effectors.

## Setting up a new base

Fork or clone this repository.

Rename gummi_base_template to gummi_base_YOUR_OWN_BASE

`$ mv gummi_base_template gummi_base_YOUR_OWN_BASE`

Edit the file gummi_base_YOUR_OWN_BASE/set_env_gummi.sh

`$ gedit gummi_base_YOUR_OWN_BASE/set_env_gummi.sh`

And change the line containing "export ROS_GUMMI_BASE=template" to "export ROS_GUMMI_BASE=YOUR_OWN_BASE"

You can now proceed to change gummi_base_YOUR_OWN_BASE/launch/controllers_base.launch to contain the motor drivers that your base has and where they are connected in the bus and the updating the yaml definitions of joints in gummi_base_YOUR_OWN_BASE/dynamixel

## Changing Urdf definitions:

Currently the urdf file is being generated during the catkin_make routine from the files in the xacro directory "gummi_description.urdf.xacro", "gummi_description_base.urdf.xacro" and an external file from the end effector "gummi_description_ee.urdf.xacro".

The simple setup we planned it to have all joint and link definitions for the base in the "gummi_description_base.urdf.xacro" (as well as "gummi_description_ee.urdf.xacro" containing the joints and links for the end effector only) and the file "gummi_description.urdf.xacro" would just call both and include them.

To change this to a new setup, simple either write by hand, or generate it from a solidworks model the URDF file as usual and place the joints and links from gripper up to the elbow joint (inclusive) in the end-effector xacro file and the remainder joints and links, that is, from base to elbow (exclusive) in the base urdf xacro file

TO DO:
 - finish separating yaml files
 - integrate alterations into gummi_interface

## New in this version:

- .yaml files now can be generated for the base+ee pair in build time, but they need to be properly split and the script scripts/make_yaml.sh needs to be changed accordingly

##

<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a>

<br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>.
