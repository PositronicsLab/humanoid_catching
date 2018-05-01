#!/bin/bash
cd ~/.ros

echo "Killing processes"
kill $(ps -ef | grep '/usr/bin/python /opt/ros/groovy/bin/roslaunch' | grep -v grep | awk '{print $2}')

echo "Waiting for processes to die"
# Now wait for the processes to clean up
while ps -ef | grep '/usr/bin/python /opt/ros/groovy/bin/roslaunch' | grep -v grep;
do
  sleep 1;
  echo "Waiting...";
done
