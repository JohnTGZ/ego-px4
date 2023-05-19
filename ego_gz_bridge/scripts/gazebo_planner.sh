#!/bin/bash

SESSION="formation_sesh"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

SOURCE_WS="source $SCRIPT_DIR/../../../../devel/setup.bash &&"

CMD_0="roslaunch ego_planner trajectory_server.launch"
CMD_1="roslaunch ego_planner demo.launch"
CMD_2="rostopic pub /traj_server_event std_msgs/Int8 \"data: 0 \" "

# rostopic pub /traj_server_event std_msgs/Int8 "data: 0" 
# rostopic pub /traj_server_event std_msgs/Int8 "data: 2" 

if [ "$SESSIONEXISTS" = "" ]
then 
    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_WS $CMD_0" C-m 
    sleep 2
    tmux send-keys -t $SESSION:0.1 "$SOURCE_WS $CMD_1" C-m 
    tmux send-keys -t $SESSION:0.2 "$SOURCE_WS $CMD_2" 
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
