#!/bin/bash
# to be run from the root dir of this repo
# (should just run the bash script in root instead of running this directly)

cd ../..
source ./devel/setup.bash

export TURTLEBOT3_MODEL=burger

if [ $2 == "nav" ]; then
	if [ $4 == "real" ]; then
		roslaunch project $1ing_$4_env.launch drive_type:=twist
	else
		# roslaunch project $1ing_env.launch drive_type:=twist

		if [ $3 == "-100" ]; then
			roslaunch project $1ing_sim_to_real_env.launch drive_type:=twist
		else
			roslaunch project $1ing_env.launch drive_type:=twist
		fi

	fi
else
	if [ $4 == "real" ]; then
		# roslaunch project $1ing_$4_env.launch run_type:=$1
		roslaunch project $1ing_$4_env.launch drive_type:=twist
	else
		# roslaunch project $1ing_env.launch run_type:=$1

		if [ $3 == "-100" ]; then
			roslaunch project $1ing_sim_to_real_env.launch run_type:=$1
		else
			roslaunch project $1ing_env.launch run_type:=$1
		fi

	fi
fi

while true
do
	:
done
