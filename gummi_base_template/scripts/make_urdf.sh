#!/bin/bash

## Creates URDF file from gummi_description_base.urdf.xacro and gummi_description_ee.urdf.xacro
# the paths are going to look bad, but that is how I got this working. If you
# want to fiddle with cmake variables and pass the right working directories,
# please do and update all these paths.
# UPDATE: Added part to generate SRDF file as well. Probably should rename separate these files

echo "** Creating URDF file from gummi_description_base.urdf.xacro and gummi_description_ee.urdf.xacro."

. ../../devel/setup.bash

export GUMMIBASEPATH=${PWD}/../gummi_base_${ROS_GUMMI_BASE}
export GUMMIEEPATH=${PWD}/../gummi_ee_${ROS_GUMMI_EE}
rosrun xacro xacro --inorder ${GUMMIBASEPATH}/xacro/gummi_description.urdf.xacro base:=${GUMMIBASEPATH} ee:=${GUMMIEEPATH} > ${GUMMIBASEPATH}/urdf/gummi_description.urdf

echo "** Creating SRDF file from gummi_base.srdf.xacro and gummi_ee.srdf.xacro."

rosrun xacro xacro --inorder ${GUMMIBASEPATH}/xacro/gummi.srdf.xacro base:=${GUMMIBASEPATH} ee:=${GUMMIEEPATH} > ${GUMMIBASEPATH}/config/gummi.srdf
