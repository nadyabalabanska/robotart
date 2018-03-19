#!/bin/bash

source /ws/setup.bash

sim=0

if [[ -z "${REDHAWK_SIMULATION}" ]]; then
    # REDHAWK_SIMULATION is not set
    sim=0
else
    # REDHAWK_SIMULATION is set
    sim=$REDHAWK_SIMULATION
fi

if [ $sim -eq 1 ]; then
    roslaunch redhawk_ctl sim.launch &
else
    roslaunch redhawk_ctl redhawk.launch &
fi

sleep 1
roslaunch redhawk_moveit redhawk_headless.launch
