killall -9 gzserver
killall -9 gzclient
killall -9 rviz
kill -9 $(ps ax | grep $ROBOT_RL_SIM_ROOT/devel/lib/ | fgrep -v grep | awk '{ print $1 }')
kill -9 $(ps ax | grep /opt/ros/noetic/lib/moveit | fgrep -v grep | awk '{ print $1 }')
kill -9 $(ps ax | grep /opt/ros/noetic/lib/robot | fgrep -v grep | awk '{ print $1 }')
kill -9 $(ps ax | grep /opt/ros/noetic/lib/controller | fgrep -v grep | awk '{ print $1 }')
kill -9 $(ps ax | grep /opt/ros/noetic/lib/state | fgrep -v grep | awk '{ print $1 }')
kill -9 $(ps ax | grep /opt/ros/noetic/lib/nodelet | fgrep -v grep | awk '{ print $1 }')
rm -rf /home/$USER/.ros/log
