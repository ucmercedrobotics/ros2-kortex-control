#!/bin/bash
# Leaf Grasping Pipeline - tmux launcher
# Creates a tmux session with all pipeline components

SESSION_NAME="leaf_pipeline"

# Kill existing session if it exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Create new session with first window (MoveIt)
tmux new-session -d -s $SESSION_NAME -n "moveit"
tmux send-keys -t $SESSION_NAME:moveit "make moveit-target" C-m

# Wait for MoveIt to initialize before starting other nodes
sleep 2

# Window 2: Vision camera streams
tmux new-window -t $SESSION_NAME -n "vision"
tmux send-keys -t $SESSION_NAME:vision "make vision" C-m

# Window 3: Leaf segmentation (YOLO)
tmux new-window -t $SESSION_NAME -n "segmentation"
tmux send-keys -t $SESSION_NAME:segmentation "make leaf-segmentation" C-m

# Window 4: Arm control (arm_commander + target_manager)
tmux new-window -t $SESSION_NAME -n "arm-control"
tmux send-keys -t $SESSION_NAME:arm-control "make arm-control" C-m

# Window 5: Trigger (ready for user to run)
tmux new-window -t $SESSION_NAME -n "trigger"
tmux send-keys -t $SESSION_NAME:trigger "# Press Enter to trigger the pipeline:" 
tmux send-keys -t $SESSION_NAME:trigger ""
tmux send-keys -t $SESSION_NAME:trigger "make trigger-segmentation"

# Attach to the session
tmux attach-session -t $SESSION_NAME

