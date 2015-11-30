#!/bin/bash
rosservice call gazebo/delete_model '{model_name: human}'
rosparam load models/human.model human_description
rosrun gazebo_ros spawn_model -param human_description -sdf -model human -x 0.5 -z 0.5

