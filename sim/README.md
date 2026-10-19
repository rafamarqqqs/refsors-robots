# Simulation in Docker

Runs the 2016 code in its original stack, ROS Indigo with Gazebo 2.2 on Ubuntu
14.04, and shows the running mission in the browser two ways: a live 3D view
you can move around in, and the Gazebo window through noVNC.

## Start

From the repository root:

```sh
docker build --platform linux/amd64 -f sim/Dockerfile -t refsors-sim . && docker run --rm --name refsors-sim --platform linux/amd64 -p 127.0.0.1:8080:8080 -p 127.0.0.1:6080:6080 refsors-sim
```

Then open:

- <http://127.0.0.1:8080>: the live 3D view.
- <http://127.0.0.1:6080/vnc_auto.html?host=127.0.0.1&port=6080>: the Gazebo
  window, view-only.

Both ports are bound to `127.0.0.1` only, and neither view can change the
running world. Stop with `Ctrl-C` or `docker stop refsors-sim`.

The image is amd64 only. On Apple Silicon it runs under Rosetta: Gazebo needs
one to two minutes to spawn both robots and then runs at a real-time factor of
about 0.4. The container needs about 6 GB of memory.

## Live 3D view

The page at port 8080 draws the harbor and follows the mission as it runs:

- The world from `world/harbor.world`: ground, walls, containers and the red
  target, at the poses Gazebo reports.
- Both robots at their live poses from `/gazebo/model_states`, labeled. The
  shapes are simplified boxes and wheels.
- Each robot's live laser scan (`/larger_robot/scan`,
  `/smaller_robot/front/scan`), and every laser hit since the page opened.
- Trails, the current search goal of each robot (from the navigation nodes'
  `Driving to x y` log lines) and A* paths sent to the `driveTo` actions.
- Sim time, real-time factor, each robot's position, heading, speed and goal,
  and the mission log of the 15 nodes in `launch/master.launch`.

Camera: drag to orbit, right-drag, Shift-drag or the arrow keys to pan, scroll
to zoom. The buttons or keys `1` and `2` follow the Husky or the Jackal, `T`
looks straight down, `R` resets the view and `Esc` stops following.

The robots' own occupancy grids and the shared knowledge map stay inside their
nodes: no topic publishes them, and the only way to read them is a service call.
The "laser hits" layer is drawn by the page from the scans instead.

### Watch-only

[live3d/relay.py](live3d/relay.py) is a small ROS node with an HTTP server. It
only subscribes to `/gazebo/model_states`, `/clock`, the two laser scans,
`/rosout_agg` and the two `Navigation/driveTo/goal` topics, and streams them
to the page as Server-Sent Events, a one-way channel. It has no publishers,
service clients or action clients. The HTTP server answers `GET` for the page's
files, the world file and the event stream; any other path is 404 and any other
method is 501. The page cannot publish, call services or move anything.

rosbridge was not used: the last Indigo release (0.7.16) has no option to limit
clients to subscriptions, and always allows publishing and service calls.

### Files

- [live3d/relay.py](live3d/relay.py): the relay, started by [start.sh](start.sh).
- [live3d/index.html](live3d/index.html), [live3d/app.js](live3d/app.js),
  [live3d/style.css](live3d/style.css): the page.
- [live3d/vendor/three](live3d/vendor/three): three.js 0.170.0 under its
  [MIT license](live3d/vendor/three/LICENSE), copied from the npm package
  (`three-0.170.0.tgz`, sha512
  `FQK+LEpYc0fBD+J8g6oSEyyNzjp+Q7Ks1C568WWaoMRLW+TkNNWmenWeGgJjV105Gd+p/2ql1ZcjYvNiPZBhuQ==`):
  `build/three.module.min.js` and `examples/jsm/controls/OrbitControls.js`,
  unchanged. The page loads nothing from outside the container.

## What runs

[Dockerfile](Dockerfile) copies the package into a catkin workspace, builds it
with `catkin_make` and starts [start.sh](start.sh): a virtual display, a
view-only `x11vnc`, `websockify` with noVNC, the 3D view relay, and then
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
| `JACKAL_LASER_RPY="3.14159 0 0"`, `JACKAL_LASER_OFFSET="0 0 0.298"` | A 2026 simulation change. `src/laser.cpp` gives beam 120 the angle +1.57 (left) and beam 599 the angle -1.57 (right), the beam order of an upside-down laser. `husky_description` mounts the Husky's LMS1xx upside down, but `jackal_description` mounts the Jackal's upright, so the Jackal saw its scan mirrored: it steered into `container_23`, climbed its corner and flipped over within the first 80 s of sim time. These variables turn the Jackal's laser over like the Husky's. The offset puts it back at the same point, 0.12 m forward and 0.333 m up from `base_link`. |
| `ROBOT_INITIAL_POSE="-x -2.5 -y 0"` | The Husky spawns at the origin by default, inside `container_10` of `harbor.world`. The repository never set a Husky pose, so this is the free spot closest to the origin. |

The patch applies to `/opt/ros/indigo/share` inside the image only.

## Known behavior in this re-run

- Both robots drive. The Jackal leaves its spawn point (-17, 15) at up to
  about 1.5 m/s for a fraction of a second, then slows down. Its height, roll
  and pitch stay level meanwhile, so this is driving, not the robot being
  thrown.
- Both lasers are upside down (roll 180° in tf), the Husky's by
  `husky_description` and the Jackal's by the variables above. The 3D view
  mirrors both scans accordingly.
- Under Rosetta the container's memory grows by about 15 MB per second of sim
  time, about 0.74 GB per minute on an Apple Silicon Mac. It grows in
  `gzserver` and in every ROS node, in an anonymous region above the x86-64
  user address space that Rosetta uses, not in the nodes' heaps. With the
  7.75 GB Docker Desktop VM used here, the kernel kills `gzserver` after about
  445 s of sim time: three runs on 2026-10-17 ended between sim 12422 and
  12432, with the mission starting at 11977. The 3D view then shows "No
  Gazebo data".
- So far no run reached a detection. In the three recorded runs both robots
  searched until Gazebo was killed, and neither camera saw enough red to
  report the target. The rest of the mission (the A* drive, alignment and
  exit) has not been observed in this re-run. A full mission needs more memory
  for Docker, or a native amd64 host.
  [../viewer/media/sim-mission-2026-10-17.mp4](../viewer/media/sim-mission-2026-10-17.mp4)
  shows one such run.
- In one run the Husky drove nose first into `container_18` and did not move
  for the remaining three minutes of sim time, through a change of goal.
- `gzserver` sometimes dies at start with `X Error of failed request:
  BadDrawable` (twice in ten starts on 2026-10-17, both while the Mac was
  heavily loaded). The 3D view then stays on "Waiting for Gazebo…". Restart the
  container.

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
