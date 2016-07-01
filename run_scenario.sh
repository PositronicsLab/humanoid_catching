#!/bin/bash

cd ~/.ros

# Create a unique ID for the scenario
SCENARIO_FOLDER=`/bin/date +%s`
mkdir -p results/$SCENARIO_FOLDER

echo "Storing results in results/$SCENARIO_FOLDER"

# Execute baseline
echo "Executing baseline scenario"
RESULTS_FOLDER=results/$SCENARIO_FOLDER/baseline
echo "Creating folder for baseline results: $RESULTS_FOLDER"
mkdir -p $RESULTS_FOLDER
export RESULTS_FOLDER=$RESULTS_FOLDER

for i in `seq 1 1`;
do
  echo "Executing scenario: $i"
  export i=$i
  # Launch gazebo
  roslaunch gazebo_ros empty_world.launch &
  sleep 15
  roslaunch humanoid_catching no_catching_simulated.launch &
  sleep 15
  echo "Starting fall"
  rosrun humanoid_catching fall_starter _random:=true _waitForBalancer:=false
  sleep 30
   kill $(ps -ef | grep '/usr/bin/python /opt/ros/hydro/bin/roslaunch' | grep -v grep | awk '{print $2}')
  sleep 5
done

# Execute control
echo "Executing control scenario"
RESULTS_FOLDER=results/$SCENARIO_FOLDER/control
echo "Creating folder for control results: $RESULTS_FOLDER"
mkdir -p $RESULTS_FOLDER
export RESULTS_FOLDER=$RESULTS_FOLDER

for i in `seq 1 100`;
do
  echo "Executing scenario: $i"
  export i=$i
  # Launch gazebo
  roslaunch pr2_gazebo pr2_empty_world.launch &
  sleep 15
  roslaunch humanoid_catching human_catching_simulated.launch &
  sleep 15
  echo "Starting fall"
  rosrun humanoid_catching fall_starter _random:=true _skipnotify:=false
  sleep 120
  kill $(ps -ef | grep '/usr/bin/python /opt/ros/hydro/bin/roslaunch' | grep -v grep | awk '{print $2}')
  sleep 15
done
