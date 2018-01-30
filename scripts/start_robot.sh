#!/bin/bash

# This scripts is for bringing up a robot
# if [ "$EUID" -ne 0 ]
#   then echo "please run as root"
#   exit
# fi

function cleanup {

    echo "Stopping procman controller"
    killall procman-controller
    echo "Stopping procman daemon"
    killall procman-daemon

    wait $daemon
    wait $controller

    exit
}
trap cleanup SIGINT

export ROOT=$(readlink -f $(dirname $0)/..)
export MAGIC2="$ROOT/magic2"
echo "Using MAGIC2=$MAGIC2"

export PATH=$PATH:$ROOT/bin
export LCMTYPES=""
export LCMTYPES="$LCMTYPES $ROOT/lcmtypes/"
export LCMTYPES="$LCMTYPES $MAGIC2/lcmtypes/"
export LCMTYPES="$LCMTYPES $MAGIC2/../navboard/lcmtypes/"
export LCMTYPES="$LCMTYPES $MAGIC2/april2/lcmtypes/"
export APRIL2_PATH="$MAGIC2/april2"
export ROBOT_ID="7"
export GROUNDSTATION_IP="10.0.1.1"

# Network setup
#ifconfig eth0 10.0.1.1 netmask 255.255.255.0
#route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0
#route add -net 10.0.0.0 netmask 255.255.0.0 gw 10.0.1.2 dev eth0

# LCM Happiness w/o internet
#ifconfig lo multicast
#route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

procman-daemon &
daemon=$!
procman-controller -c $MAGIC2/config/proc-robot-concat.config &
controller=$!

printf "HomePage: \033[1mhttp://localhost:3912\033[m\n"

wait $daemon
wait $controller

#while true; do
#    : # Do nothing
#done
