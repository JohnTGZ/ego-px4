#!/bin/bash

SESSION="quick_start_sesh"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

# Directories
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PX4_AUTOPILOT_REPO_DIR="$SCRIPT_DIR/../../../../PX4-Autopilot"

# SOURCE_PX4_AUTOPILOT="
# source $PX4_AUTOPILOT_REPO_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash $PX4_AUTOPILOT_REPO_DIR $PX4_AUTOPILOT_REPO_DIR/build/px4_sitl_default &&
# source $SCRIPT_DIR/../../../../devel/setup.bash &&
# "

SOURCE_PX4_AUTOPILOT="
source $SCRIPT_DIR/../../../../devel/setup.bash &&
"

ADD_ROS_PACKAGE_PATH="
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_AUTOPILOT_REPO_DIR:$PX4_AUTOPILOT_REPO_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic &&
"

CMD_0="
roslaunch ego_gz_bridge simulator.launch world_name:=$SCRIPT_DIR/../simulation/worlds/ego_test.world
"
CMD_1="
roslaunch px4 px4.launch
"
CMD_2="roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557""

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_PX4_AUTOPILOT $CMD_0" C-m 
    sleep 3
    tmux send-keys -t $SESSION:0.1 "$ADD_ROS_PACKAGE_PATH $CMD_1" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.2 "$CMD_2" C-m 
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
