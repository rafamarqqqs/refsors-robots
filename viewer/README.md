# Static viewer

A local web page that shows the 2016 project without ROS or Docker: the
mission and robots, the node graph, the saved maps, the harbor world, the
commit history and a capture of a 2026 re-run.

## Open

Open [index.html](index.html) in a browser, or serve this folder:

```sh
cd viewer && python3 -m http.server 8000
```

and open <http://127.0.0.1:8000>. The page makes no network requests beyond
its own files.

## Sections and sources

| Section | Read from |
| --- | --- |
| Overview | `README.md`, `launch/*.launch`, `include/robot_control/topics.h` |
| Simulation | [media/sim-2026-10-17.mp4](media/sim-2026-10-17.mp4), recorded with [../sim](../sim) |
| Architecture | `launch/master.launch`, `CMakeLists.txt`, `src/`, `include/robot_control/topics.h`, `msg/`, `srv/`, `action/` |
| Maps | `maps/*.map`, with the file formats from the writers in `src/` |
| World | `world/harbor.world`, the world `master.launch` loads |
| History | `git log`, commits dated 2016 |

The architecture table comes from patterns in the sources (`advertise`,
`subscribe`, `serviceClient`, `advertiseService`, actionlib servers and
clients), with names resolved through `topics.h` and each node's launch
namespace. Each entry links back to its `file:line`.

## Regenerate the data

[extract.py](extract.py) uses only the Python 3 standard library. It reads the
repository and `git log` read-only and writes only [data.js](data.js):

```sh
python3 viewer/extract.py
```

It needs a full Git checkout, since the history and the maps' last commits come
from Git. Running it on an unchanged checkout reproduces `data.js` byte for byte.

## Files

- [index.html](index.html), [style.css](style.css), [app.js](app.js): the page.
- [data.js](data.js): generated data, committed so the page works without Python.
- [extract.py](extract.py): the generator.
- [media/](media): the simulation capture.
