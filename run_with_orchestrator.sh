echo "Starting"
tmux new-session -n 'roscore' -d 'roscore; $SHELL'
sleep 3;
tmux new-window -n 'rosbridge' 'roslaunch speech_test rosbridge_websocket_remap.launch; $SHELL'
tmux new-window -n 'ros_pololu' 'rosrun ros_pololu_servo ros_pololu_servo_node; $SHELL'
tmux new-window -n 'pau2motors' 'rosrun pau2motors pau2motors_node.py cmd_neck_pau:=orch/cmd_neck_pau; $SHELL'
tmux new-window -n 'expr_node' 'rosrun basic_head_api head_ctrl.py point_head:=orch/point_head cmd_neck_pau:=orch/cmd_neck_pau make_face_expr:=orch/make_face_expr cmd_face_pau:=speechtest_face_in; $SHELL'
tmux new-window -n 'speech' 'rosrun speech_test speech_test.py; $SHELL'
tmux new-window -n 'orch' 'rosrun speech_test orchestrator.py; $SHELL'
tmux new-window -n 'webserver' 'python -m SimpleHTTPServer; $SHELL'
tmux attach;
echo "Started"