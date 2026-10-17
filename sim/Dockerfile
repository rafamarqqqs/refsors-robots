# ROS Indigo + Gazebo 2 image that builds robot_control and shows the running mission
# as a live 3D page (port 8080) and the Gazebo window through noVNC (port 6080).
# Build context is the repository root: docker build -f sim/Dockerfile -t refsors-sim .
FROM osrf/ros:indigo-desktop-full@sha256:53729ecd137c19e0c82658d1a7468533c3b8e7951254a699bf57e0033ea3ed4e

# The ESM lists need an Ubuntu Pro token; the base and ROS snapshot mirrors are enough.
RUN grep -l esm.ubuntu.com /etc/apt/sources.list.d/*.list | xargs rm -f \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-indigo-husky-gazebo ros-indigo-jackal-description ros-indigo-jackal-control libopencv-dev \
      xvfb x11vnc novnc websockify libav-tools \
 && rm -rf /var/lib/apt/lists/*

# The final Indigo Husky/Jackal packages (2019) hardcode the ROS namespace "/" in
# gazebo_ros_control and the LMS1xx laser, and spawn the Husky as "mobile_base".
# The 2016 launch files namespace each robot and the code looks for the model "husky".
COPY sim/deps.patch /tmp/deps.patch
RUN patch -p1 -d /opt/ros/indigo/share < /tmp/deps.patch

# Jackal accessories are chosen by environment: a front laser (front/scan) and
# a stereo camera pair (front/left/image_raw), the topics topics.h subscribes to.
# laser.cpp reads scans in the beam order of an upside-down laser, which is how
# husky_description mounts the Husky's. The Jackal's laser is turned over the same
# way; the bracket is 0.149 m tall, so the offset keeps the laser where it was.
ENV JACKAL_LASER=1 JACKAL_STEREO_FLEA3=1 JACKAL_LASER_RPY="3.14159 0 0" JACKAL_LASER_OFFSET="0 0 0.298"

# husky_gazebo spawns at ROBOT_INITIAL_POSE, the origin by default, which harbor.world
# fills with container_10. The repository never set a Husky pose; this is the free
# spot closest to the origin.
ENV ROBOT_INITIAL_POSE="-x -2.5 -y 0"

# Copy the package (never bind-mount it): a run writes into its maps/ folder.
WORKDIR /catkin_ws
COPY CMakeLists.txt package.xml /catkin_ws/src/robot_control/
COPY action /catkin_ws/src/robot_control/action
COPY msg /catkin_ws/src/robot_control/msg
COPY srv /catkin_ws/src/robot_control/srv
COPY include /catkin_ws/src/robot_control/include
COPY src /catkin_ws/src/robot_control/src
COPY launch /catkin_ws/src/robot_control/launch
COPY world /catkin_ws/src/robot_control/world
COPY maps /catkin_ws/src/robot_control/maps
# catkin looks up its OpenCV component only under ROS prefixes; Indigo ships it in /usr/share/OpenCV.
RUN bash -c 'source /opt/ros/indigo/setup.bash && catkin_make -DOpenCV_DIR=/usr/share/OpenCV'

# Watch-only 3D page and the relay that streams ROS topics to it.
COPY sim/live3d /opt/live3d
COPY sim/start.sh /start.sh
EXPOSE 8080 6080
CMD ["bash", "/start.sh"]
