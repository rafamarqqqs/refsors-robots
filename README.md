# RefSORS robots

Two simulated robots map a harbor, share what they find and locate a target
together. Undergraduate research at ICMC-USP (CNPq, 2015–2016), built as
services on [RefSORS](https://teses.usp.br/teses/disponiveis/55/55134/tde-27112015-145425/en.html),
the reference architecture for service-oriented robotic systems from
L. B. R. de Oliveira's PhD thesis.

The code is from 2016 and targets ROS Indigo with Gazebo 2. It is kept as it
was, apart from portable map paths and package metadata.

## See it

- **Live simulation.** Build and run the 2016 code in Docker, then watch the
  mission in a 3D view you can orbit, zoom, pan and point at either robot:

  ```sh
  docker build --platform linux/amd64 -f sim/Dockerfile -t refsors-sim . && docker run --rm --name refsors-sim --platform linux/amd64 -p 127.0.0.1:8080:8080 -p 127.0.0.1:6080:6080 refsors-sim
  ```

  Open <http://127.0.0.1:8080>. The Gazebo window itself is at
  <http://127.0.0.1:6080/vnc_auto.html?host=127.0.0.1&port=6080>. Details and
  requirements are in [sim/README.md](sim/README.md).
- **Static viewer.** Open [viewer/index.html](viewer/index.html) for the
  mission, node graph, saved maps, world, history and a recorded run, with no
  ROS or Docker. See [viewer/README.md](viewer/README.md).

## Mission

1. A Husky (larger, Kinect) and a Jackal (smaller, camera) start in a harbor
   world full of containers.
2. Both explore the free areas, each building an occupancy grid from its laser
   and sharing it through a common knowledge service so they do not search the
   same area twice.
3. Each robot watches its camera for the target, a red object found by HSV
   thresholding.
4. The first robot to see it ends the search for both. If it is the Jackal, the
   Husky plans a path to the target with A* while the Jackal leaves. The Husky
   then aligns with the target and both robots exit.

## Services

Each robot runs its own set of nodes, under the `larger_robot` or
`smaller_robot` namespace:

| Node | Role |
| --- | --- |
| `localization` | Publishes the robot's pose from Gazebo's model states |
| `laser` | Turns the laser scan into the measures the other services use |
| `mapping` | Builds the robot's occupancy grid (`createMap` action) |
| `navigation` | Explores with a force-based occupancy grid, drives to points and aligns with the target (`search`, `driveTo`, `alignWithBomb` actions) |
| `image_processing` | Detects the target and reports its displacement (`processImage` action, `getBombDisplacement` service) |
| `control` | Coordinates the robot's services for each mission step (`exit` action) |

Shared by both robots:

| Node | Role |
| --- | --- |
| `knowledge` | Shared map, robot positions and the next unexplored goal (`addToMap`, `getMap`, `getPositions`, `getNewGoal` services) |
| `path_planning` | Global paths with A* (`defineGlobalPath` service) |
| `application` | Runs the mission and hands work from one robot to the other |

Messages, services and actions are in [msg](msg), [srv](srv) and [action](action).
Worlds are in [world](world); maps written during a run go to [maps](maps).

## Run

Needs ROS Indigo (Ubuntu 14.04) with `gazebo_ros`, `husky_gazebo`,
`jackal_description` and `jackal_control`.

```sh
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/rafamarqqqs/refsors-robots.git robot_control
cd ~/catkin_ws && catkin_make && source devel/setup.bash
roslaunch robot_control master.launch
```

## License

[MIT](LICENSE). RefSORS is described in L. B. R. de Oliveira's thesis; this
repository contains only the robot system built on it.
