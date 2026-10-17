#!/bin/bash
# Virtual display -> view-only VNC -> noVNC on port 6080, the watch-only 3D relay
# on port 8080, then the 2016 mission.
# View-only keeps browser clicks from moving models in the running world.
set -e
export DISPLAY=:1
Xvfb :1 -screen 0 1280x800x24 +extension GLX +render -noreset &
sleep 2
x11vnc -display :1 -viewonly -forever -shared -nopw -quiet -localhost -rfbport 5900 &
websockify --web /usr/share/novnc 6080 localhost:5900 &

source /catkin_ws/devel/setup.bash
python /opt/live3d/relay.py &
exec roslaunch robot_control master.launch
