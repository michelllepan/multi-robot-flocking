#!/bin/bash

tmux new-session -d -s redis
tmux send-keys -t redis:0 'redis-cli shutdown; redis-server --protected-mode no' C-m

tmux new-session -d -s driver
tmux send-keys -t driver:0 'ros2 launch launch/main.launch.py' C-m

sleep 3
tmux new-session -d -s state_pub
tmux send-keys -t state_pub:0 'python scripts/run_state_pub.py' C-m

sleep 3
tmux new-session -d -s robot
tmux send-keys -t robot:0 'python scripts/run_robot.py' C-m

tmux new-session -d -s music
tmux send-keys -t music 'python scripts/run_music.py' C-m

tmux attach-session -t robot
