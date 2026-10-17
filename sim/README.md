# Simulation in Docker

Runs the 2016 code in its original stack, ROS Indigo with Gazebo 2.2 on Ubuntu
14.04, and shows the Gazebo window in the browser through noVNC.

## Start

From the repository root:

```sh
docker build --platform linux/amd64 -f sim/Dockerfile -t refsors-sim . && docker run --rm --name refsors-sim --platform linux/amd64 -p 127.0.0.1:6080:6080 refsors-sim
```

Open <http://127.0.0.1:6080/vnc_auto.html?host=127.0.0.1&port=6080>.

The port is bound to `127.0.0.1` only, and the view is read-only so browser
clicks cannot move models in the running world. Stop with `Ctrl-C` or
`docker stop refsors-sim`.

The image is amd64 only. On Apple Silicon it runs under Rosetta: Gazebo needs
one to two minutes to spawn both robots and then runs at a real-time factor of
about 0.4. The container needs about 6 GB of memory.

## What runs

[Dockerfile](Dockerfile) copies the package into a catkin workspace, builds it
with `catkin_make` and starts [start.sh](start.sh): a virtual display, a
view-only `x11vnc`, `websockify` with noVNC, and then
`roslaunch robot_control master.launch`.

The package is copied, never mounted. A run writes its maps into the package's
`maps/` folder, and those writes stay inside the container.

## Changes made inside the image

The 2016 sources are compiled and launched unchanged. The image changes only the
environment around them:

| Change | Why |
| --- | --- |
| `catkin_make -DOpenCV_DIR=/usr/share/OpenCV` | `CMakeLists.txt` asks catkin for an `OpenCV` component, and catkin only searches ROS prefixes; Indigo's OpenCV 2.4 is in `/usr/share/OpenCV`. |
| [deps.patch](deps.patch), `gazebo_ros_control` namespace | The last Indigo release of `husky_gazebo` and `jackal_description` (2019) hardcodes the namespace `/`. Under the 2016 namespaced launch, the Husky plugin waits forever for `/robot_description` and freezes the world. Without the hardcoded value, `spawn_model` uses each robot's namespace. |
| [deps.patch](deps.patch), LMS1xx laser namespace | Same issue: the laser would publish `/scan` and `/front/scan` instead of `/larger_robot/scan` and `/smaller_robot/front/scan`, which `topics.h` subscribes to. |
| [deps.patch](deps.patch), Husky model name | `husky_gazebo` spawns the model as `mobile_base`; the localization node looks for `husky` (`MODEL_NAME` in `topics.h`). |
| `JACKAL_LASER=1`, `JACKAL_STEREO_FLEA3=1` | Jackal accessories are chosen by environment. These give the `front/scan` and `front/left/image_raw` topics the code subscribes to. |
| `ROBOT_INITIAL_POSE="-x -2.5 -y 0"` | The Husky spawns at the origin by default, inside `container_10` of `harbor.world`. The repository never set a Husky pose, so this is the free spot closest to the origin. |

The patch applies to `/opt/ros/indigo/share` inside the image only.

## Known log noise

- `Unable to download model manifests`: Gazebo 2 queries an online model
  database that no longer exists. The harbor world needs no downloaded models.
- `Tried to advertise a service that is already advertised` for `/imu/data` and
  `/navsat/fix`, and `TF_OLD_DATA` warnings for `imu_link`: both robots' IMU and
  GPS plugins still use the namespace `/`. The 2016 nodes do not use them.

The 2016 nodes log through `rosout`. To follow the mission:

```sh
docker exec refsors-sim bash -c 'tail -f /root/.ros/log/latest/rosout.log' | grep robot_control
```

## Recording a capture

`libav-tools` is installed so the display can be recorded from inside the
container, for example 60 seconds at 4 frames per second:

```sh
docker exec refsors-sim avconv -f x11grab -r 4 -s 1280x800 -i :1.0 -t 60 -c:v libx264 -pix_fmt yuv420p /tmp/capture.mp4
docker cp refsors-sim:/tmp/capture.mp4 .
```
