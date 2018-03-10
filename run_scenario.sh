#!/bin/bash
unset DISPLAY
cd ~/.ros

# Create a unique ID for the scenario
SCENARIO_FOLDER=`/bin/date +%s`
mkdir -p results/$SCENARIO_FOLDER
echo "Storing results in results/$SCENARIO_FOLDER"

# Execute control
echo "Executing control scenario"
RESULTS_FOLDER=results/$SCENARIO_FOLDER/control

echo "Creating folder for control results: $RESULTS_FOLDER"
mkdir -p $RESULTS_FOLDER
export RESULTS_FOLDER=$RESULTS_FOLDER

for i in `seq 1 50`;
do
  echo "Executing scenario: $i"
  export i=$i
  # Launch gazebo
  roslaunch pr2_gazebo pr2_empty_world.launch gui:=false  &
  echo "PR2 launched. Waiting 5 seconds for stability"
  sleep 5

  echo "Running set arm positions"
  rosrun gazebo_utils set_arm_positions
  echo "Completed setting arm positions"

  echo "Lauching humanoid catching"
  roslaunch humanoid_catching human_catching_simulated.launch &

  echo "Starting fall"
  rosrun gazebo_utils random_fall_generator

  echo "Killing processes"
  kill $(ps -ef | grep '/usr/bin/python /opt/ros/groovy/bin/roslaunch' | grep -v grep | awk '{print $2}')

  echo "Waiting for processes to die"
  # Now wait for the processes to clean up
  while ps -ef | grep '/usr/bin/python /opt/ros/groovy/bin/roslaunch' | grep -v grep;
  do
    sleep 1;
    echo "Waiting...";
  done
done
