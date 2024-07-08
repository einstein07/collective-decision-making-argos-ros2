#!/bin/bash

# The purpose of this script is to create a launch file for a multi-robot
# system by replicating a "group" tag "n" times, then executing the resulting
# launch file.

# The number of robots.  This should match the 'quantity' value in the argos world file (e.g. argos_worlds/demo.argos).
n=3

LAUNCH_FILE=/tmp/argos_interface.launch.xml

echo "<launch>" > $LAUNCH_FILE

for ((i=0; i<n; i++)); do
    namespace="bot$i"
    echo -e "\t<group>"
    echo -e "\t\t<node pkg=\"argos_interface\" exec=\"demo_controller.py\" name=\"demo_controller\" output=\"screen\" namespace=\"$namespace\"/>"
    echo -e "\t</group>"
done >> $LAUNCH_FILE
echo -e "</launch>" >> $LAUNCH_FILE

ros2 launch $LAUNCH_FILE
