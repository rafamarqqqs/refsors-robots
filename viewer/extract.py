#!/usr/bin/env python3
"""Extract viewer data from the 2016 robot_control package.

Reads the repository read-only and writes only viewer/data.js.
Run from anywhere: python3 viewer/extract.py
"""

import json
import math
import re
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

VIEWER = Path(__file__).resolve().parent
ROOT = VIEWER.parent
OUT = VIEWER / "data.js"

# Size of the mapped area, from MAP_LENGTH / MAP_WIDTH in include/robot_control/resources.h.
MAP_SIDE_M = 40.0


def read(rel):
    return (ROOT / rel).read_text(encoding="utf-8")


def strip_comments(code):
    """Blank out C/C++ comments, keeping line numbers intact."""
    def blank(m):
        return re.sub(r"[^\n]", " ", m.group(0))
    return re.sub(r"/\*.*?\*/|//[^\n]*", blank, code, flags=re.S)


def line_of(text, pos):
    return text.count("\n", 0, pos) + 1


# ---------------------------------------------------------------- README

def readme_sections():
    text = read("README.md")
    parts = re.split(r"^## ", text, flags=re.M)
    head = parts[0].split("\n", 1)
    sections = {"title": head[0].lstrip("# ").strip(), "intro": head[1].strip()}
    for part in parts[1:]:
        name, body = part.split("\n", 1)
        sections[name.strip()] = body.strip()
    return {k: sections[k] for k in ("title", "intro", "Mission", "Services")}


# ---------------------------------------------------------------- interfaces

def parse_interface(path):
    sections = []
    for block in path.read_text(encoding="utf-8").split("---"):
        fields = []
        for line in block.splitlines():
            line = line.split("#", 1)[0].strip()
            if line:
                ftype, fname = line.split()[:2]
                fields.append({"type": ftype, "name": fname})
        sections.append(fields)
    return sections


def interfaces():
    labels = {"srv": ["request", "response"],
              "action": ["goal", "result", "feedback"],
              "msg": ["fields"]}
    result = {}
    for kind, names in labels.items():
        for path in sorted((ROOT / kind).glob("*." + kind)):
            sections = parse_interface(path)
            result[path.stem] = {
                "kind": kind,
                "file": f"{kind}/{path.name}",
                "sections": [{"label": n, "fields": s} for n, s in zip(names, sections)],
            }
    return result


# ---------------------------------------------------------------- topics.h

def topic_macros():
    """Map macro name -> {"larger_robot": name, "smaller_robot": name} or a plain string."""
    text = read("include/robot_control/topics.h").replace("\\\n", " ")
    macros = {}
    for m in re.finditer(r'^#define\s+(\w+)\(type\)\s+\(strcmp\(type, LARGER_ROBOT\) == 0 \? "([^"]+)"\s*:\s*"([^"]+)"\)',
                         text, flags=re.M):
        macros[m.group(1)] = {"larger_robot": m.group(2), "smaller_robot": m.group(3)}
    for m in re.finditer(r'^#define\s+(\w+)\s+"([^"]+)"', text, flags=re.M):
        macros[m.group(1)] = m.group(2)
    return macros


# ---------------------------------------------------------------- build and launch

def executables():
    """Map executable -> source files, from the active add_executable lines."""
    result = {}
    for line in read("CMakeLists.txt").splitlines():
        m = re.match(r"\s*add_executable\((\w+)\s+([^)]*)\)", line)
        if m:
            result[m.group(1)] = m.group(2).split()
    return result


def launch_nodes():
    root = ET.fromstring(read("launch/master.launch"))
    nodes = []
    for parent, ns in [(root, "")] + [(g, g.get("ns")) for g in root.findall("group")]:
        for n in parent.findall("node"):
            nodes.append({
                "ns": ns,
                "name": n.get("name"),
                "executable": n.get("type"),
                "args": n.get("args", ""),
                "id": "/" + (ns + "/" if ns else "") + n.get("name"),
            })
    return nodes


def robots():
    master = ET.fromstring(read("launch/master.launch"))
    world = master.find("include/arg[@name='world_name']").get("value")
    macros = topic_macros()
    result = []
    for group in master.findall("group"):
        ns = group.get("ns")
        launch_file = f"launch/{ns}.launch"
        launch = ET.fromstring(read(launch_file).strip())
        includes = sorted({i.get("file") for i in launch.iter("include")})
        spawn = None
        for n in launch.iter("node"):
            if n.get("type") == "spawn_model":
                args = n.get("args").split()
                spawn = {a.lstrip("-"): float(args[i + 1]) for i, a in enumerate(args)
                         if a in ("-x", "-y", "-Y")}
        result.append({
            "ns": ns,
            "model": macros["MODEL_NAME"][ns],
            "launch": launch_file,
            "includes": includes,
            "launch_args": {a.get("name"): a.get("value") for a in group.findall("include/arg")},
            "scan_topic": macros["SCAN_TOPIC"][ns],
            "camera_topic": macros["CAMERA_TOPIC"][ns],
            "spawn": spawn,
        })
    return {"world": world.replace("$(find robot_control)/", ""), "robots": result}


# ---------------------------------------------------------------- source scan

def resolve(name, ns):
    if name.startswith("/"):
        return name
    return "/" + (ns + "/" if ns else "") + name


def callback_types():
    """Map 'Class::method' -> message type of its first const ...ConstPtr& argument."""
    result = {}
    for path in sorted((ROOT / "src").glob("*.cpp")):
        code = strip_comments(path.read_text(encoding="utf-8"))
        for m in re.finditer(r"(\w+::\w+)\s*\(\s*const\s+([\w:]+?)(?:::ConstPtr|ConstPtr)\s*&", code):
            result[m.group(1)] = m.group(2)
    return result


def scan_source(rel, callbacks):
    """Find ROS endpoints a source file creates. Returns (role, kind, macro, absolute ns, type, line)."""
    code = strip_comments(read(rel))
    found = []
    patterns = [
        ("provides", "topic", r"advertise<([\w:]+)>\(\s*(\w+)"),
        ("uses", "topic", r"\.subscribe\(\s*(\w+)(?:\(\w+\))?\s*,\s*\d+\s*,\s*&(\w+::\w+)"),
        ("uses", "service", r"serviceClient<([\w:]+)>\(\s*(\w+)"),
        ("provides", "service", r"advertiseService\(\s*()(\w+)"),
        ("provides", "action", r"\w+\(\s*n\s*,\s*()(\w+_ACTION)\s*,\s*boost::bind"),
        ("uses", "action", r"\w+\(\s*()(\w+_ACTION)\s*,\s*true\s*\)"),
    ]
    for role, kind, pattern in patterns:
        for m in re.finditer(pattern, code):
            line = line_of(code, m.start())
            if kind == "topic" and role == "uses":
                found.append((role, kind, m.group(1), None, callbacks.get(m.group(2), ""), line))
            else:
                found.append((role, kind, m.group(2), None, m.group(1), line))
    # application.cpp builds absolute action names: name = "/ns/"; name += MACRO;
    for m in re.finditer(r'(\w+)\s*=\s*"/(\w+)/"\s*;\s*\1\s*\+=\s*(\w+_ACTION)\s*;', code):
        found.append(("uses", "action", m.group(3), m.group(2), "", line_of(code, m.start())))
    return found


def architecture():
    macros = topic_macros()
    sources = executables()
    iface = interfaces()
    nodes = launch_nodes()
    callbacks = callback_types()
    endpoints = {}

    for node in nodes:
        robot_type = node["args"] or None
        node["sources"] = sources[node["executable"]]
        for rel in node["sources"]:
            for role, kind, macro, abs_ns, ftype, line in scan_source(rel, callbacks):
                value = macros[macro]
                if isinstance(value, dict):
                    value = value[robot_type]
                name = resolve(value, abs_ns if abs_ns else node["ns"])
                if kind != "topic":
                    ftype = name.rsplit("/", 1)[1]
                    ftype = ftype if ftype in iface else ""
                ep = endpoints.setdefault(name, {"name": name, "kind": kind, "type": "",
                                                 "providers": [], "users": []})
                if ftype and not ep["type"]:
                    ep["type"] = ftype.replace("robot_control::", "")
                ref = {"node": node["id"], "file": rel, "line": line}
                ep["providers" if role == "provides" else "users"].append(ref)

    for ep in endpoints.values():
        for key in ("providers", "users"):
            ep[key].sort(key=lambda r: (r["node"], r["file"], r["line"]))
    order = {"topic": 0, "service": 1, "action": 2}
    return {
        "nodes": nodes,
        "endpoints": sorted(endpoints.values(), key=lambda e: (order[e["kind"]], e["name"])),
        "interfaces": iface,
    }


# ---------------------------------------------------------------- maps

# What each map file is, who writes it and what its characters mean (from src/).
MAP_INFO = [
    ("containers.map", "Shared map from the knowledge service", "Knowledge::writeMap, src/knowledge.cpp",
     {" ": "free (< EMPTY_RANGE)", ":": "uncertain", "#": "occupied (> FULL_RANGE)"}),
    ("areas.map", "Explored areas (8 m squares)", "Knowledge::writeAreas, src/knowledge.cpp",
     {"0": "not occupied", "1": "OCCUPIED"}),
    ("new", "Shared map resampled for path planning", "Knowledge::getMap, src/knowledge.cpp",
     {" ": "free", "#": "occupied"}),
    ("Path_planning_", "A* result over the resampled map", "PathPlanning::writeMap, src/path_planning.cpp",
     {" ": "free (< 30)", ":": "uncertain", "#": "occupied (> 70)", "x": "A* path", "X": "path point kept"}),
    ("containers_path_planning.map", "A* result over the resampled map", "PathPlanning::writeMap, src/path_planning.cpp until 13f5bdd (2016-05-30)",
     {" ": "free (< 30)", ":": "uncertain", "#": "occupied (> 70)", "x": "A* path", "X": "path point kept"}),
    ("occupancy_grid_", "Force-based occupancy grid used for exploration", "OccupancyGrid::writeMap, src/occupancy_grid.cpp",
     None),
]


def map_info(name):
    for prefix, title, writer, legend in MAP_INFO:
        if name == prefix or (not prefix.endswith(".map") and name.startswith(prefix)):
            return title, writer, legend
    raise ValueError(f"no description for maps/{name}")


def last_commit(rel):
    out = subprocess.run(["git", "-C", str(ROOT), "log", "-1", "--format=%H %as", "--", rel],
                         check=True, capture_output=True, text=True).stdout.split()
    return {"sha": out[0][:7], "date": out[1]}


def parse_map(path):
    title, writer, legend = map_info(path.name)
    text = path.read_text(encoding="utf-8")
    if legend is None:
        # " |  val |  val |" rows separated by blank lines; empty cells are zero.
        rows = [[int(c) if c.strip() else 0 for c in line.split("|")[1:-1]]
                for line in text.splitlines() if line.strip()]
        kind = "numeric"
    elif path.name == "areas.map":
        rows = [line.strip("|").split("|") for line in text.splitlines() if line]
        kind = "chars"
    else:
        rows = text.removesuffix("\n").split("\n")
        kind = "chars"
    cols = max(len(r) for r in rows)
    return {
        "file": f"maps/{path.name}",
        "commit": last_commit(f"maps/{path.name}"),
        "title": title,
        "writer": writer,
        "kind": kind,
        "legend": legend,
        "rows": ["".join(r) for r in rows] if kind == "chars" else rows,
        "size": [len(rows), cols],
        "cell_m": MAP_SIDE_M / cols,
    }


def maps():
    return [parse_map(p) for p in sorted((ROOT / "maps").glob("*.map"), key=lambda p: p.name.lower())]


# ---------------------------------------------------------------- world

def pose2d(text):
    v = [float(x) for x in (text or "0 0 0 0 0 0").split()]
    return v[0], v[1], v[5]


def compose(a, b):
    ax, ay, ayaw = a
    bx, by, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return ax + c * bx - s * by, ay + s * bx + c * by, ayaw + byaw


def parse_world(path):
    world = ET.parse(path).getroot().find("world")
    state = world.find("state")
    state_links = {}
    if state is not None:
        for m in state.findall("model"):
            for link in m.findall("link"):
                state_links[(m.get("name"), link.get("name"))] = pose2d(link.findtext("pose"))
    shapes = []
    for model in world.findall("model"):
        mpose = pose2d(model.findtext("pose"))
        for link in model.findall("link"):
            lpose = state_links.get((model.get("name"), link.get("name")),
                                    compose(mpose, pose2d(link.findtext("pose"))))
            for visual in link.findall("visual"):
                geom = visual.find("geometry")
                x, y, yaw = compose(lpose, pose2d(visual.findtext("pose")))
                shape = {"model": model.get("name"), "x": round(x, 4), "y": round(y, 4),
                         "yaw": round(yaw, 4), "material": visual.findtext("material/script/name") or ""}
                if geom.find("box") is not None:
                    sx, sy = [float(v) for v in geom.findtext("box/size").split()[:2]]
                    shape.update(shape_kind="box", sx=sx, sy=sy)
                elif geom.find("cylinder") is not None:
                    shape.update(shape_kind="cylinder", r=float(geom.findtext("cylinder/radius")))
                else:
                    continue
                shapes.append(shape)
    shapes.sort(key=lambda s: (s["model"], s["x"], s["y"]))
    return {"file": f"world/{path.name}", "uses_state": state is not None, "shapes": shapes}


# ---------------------------------------------------------------- history

def history():
    out = subprocess.run(
        ["git", "-C", str(ROOT), "log", "--reverse", "--format=%H%x1f%aI%x1f%an%x1f%s"],
        check=True, capture_output=True, text=True).stdout
    commits = []
    for line in out.splitlines():
        sha, date, author, subject = line.split("\x1f")
        if date.startswith("2016-"):
            commits.append({"sha": sha[:7], "date": date, "author": author, "subject": subject})
    return commits


def main():
    data = {
        "readme": readme_sections(),
        "setup": robots(),
        "architecture": architecture(),
        "maps": maps(),
        "world": parse_world(ROOT / robots()["world"]),
        "history": history(),
    }
    body = json.dumps(data, indent=1, ensure_ascii=False)
    OUT.write_text("// Generated by viewer/extract.py. Do not edit.\nwindow.REFSORS = " + body + ";\n",
                   encoding="utf-8")
    print(f"wrote {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
