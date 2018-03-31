#!/bin/bash

## Creates YAML files
echo "** Creating combine YAML files from gummi_ee and gummi_base."

. ../../devel/setup.bash

#echo ${PWD}
#echo -e "\n"
#env | grep fre
#echo -e "\n"
# sleep 1m

export SRCPATH=`echo -e "${ROS_PACKAGE_PATH//:/$'\n'}" | grep /src`
export GUMMIBASEPATH=${SRCPATH}/gummi_base_${ROS_GUMMI_BASE}
export GUMMIEEPATH=${SRCPATH}/gummi_ee_${ROS_GUMMI_EE}

# LIST OF GENERATED YAML FILES AS:
#
# ./scripts/yaml_gen_.py <base.yaml> <ee.yaml> <combined.yaml>
./scripts/yaml_gen_.py ${GUMMIBASEPATH}/config/gummi_base.yaml ${GUMMIEEPATH}/config/gummi_ee.yaml ${GUMMIBASEPATH}/config/gummi.yaml
