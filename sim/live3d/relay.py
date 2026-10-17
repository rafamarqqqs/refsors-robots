#!/usr/bin/env python
"""Watch-only relay for the live 3D view.

Serves the page on port 8080 and streams a few ROS topics to it as
Server-Sent Events. The relay only subscribes: it creates no publishers,
service clients or action clients, and its HTTP server answers GET for a fixed
list of files plus the event stream. Any other method gets 501.
"""
import BaseHTTPServer
import collections
import json
import math
import os
import Queue
import socket
import SocketServer
import sys
import threading
import time

import rospkg
import rospy
from gazebo_msgs.msg import ModelStates
from robot_control.msg import driveToActionGoal
from rosgraph_msgs.msg import Clock, Log
from sensor_msgs.msg import LaserScan

PORT = 8080
FRAME_HZ = 15
SCAN_HZ = 5
LOG_KEEP = 300

HERE = os.path.dirname(os.path.abspath(__file__))
# The world master.launch loads.
WORLD = os.path.join(rospkg.RosPack().get_path('robot_control'), 'world', 'harbor.world')

FILES = {
    '/': (os.path.join(HERE, 'index.html'), 'text/html; charset=utf-8'),
    '/app.js': (os.path.join(HERE, 'app.js'), 'text/javascript; charset=utf-8'),
    '/style.css': (os.path.join(HERE, 'style.css'), 'text/css; charset=utf-8'),
    '/vendor/three/three.module.min.js': (os.path.join(HERE, 'vendor/three/three.module.min.js'),
                                          'text/javascript; charset=utf-8'),
    '/vendor/three/OrbitControls.js': (os.path.join(HERE, 'vendor/three/OrbitControls.js'),
                                       'text/javascript; charset=utf-8'),
    '/vendor/three/LICENSE': (os.path.join(HERE, 'vendor/three/LICENSE'), 'text/plain; charset=utf-8'),
    '/world.sdf': (WORLD, 'application/xml; charset=utf-8'),
}

ROBOTS = ('larger_robot', 'smaller_robot')
SCAN_TOPICS = {'larger_robot': '/larger_robot/scan', 'smaller_robot': '/smaller_robot/front/scan'}
# The 15 nodes of launch/master.launch; the robot drivers' own logs are left out.
MISSION_NODES = set(['/Application', '/Knowledge', '/PathPlanning'] + [
    '/%s/%s' % (robot, node) for robot in ROBOTS
    for node in ('Localization', 'ImageProcessing', 'Laser', 'Mapping', 'Navigation', 'Control')])


class Latest(object):
    """Keeps the newest raw message of a topic and parses it only when read.

    Gazebo publishes model states and the clock at every physics step; parsing
    only what the page needs keeps the relay from slowing the simulation.
    """

    def __init__(self, topic, msg_class):
        self.msg_class = msg_class
        self.msg = None
        self.count = 0
        rospy.Subscriber(topic, rospy.AnyMsg, self.store, queue_size=1)

    def store(self, msg):
        self.msg = msg
        self.count += 1

    def read(self):
        msg = self.msg
        # rospy shares one subscription per topic in a process, and it subscribes to
        # /clock itself, so that topic arrives already parsed.
        if isinstance(msg, rospy.AnyMsg):
            return self.msg_class().deserialize(msg._buff)
        return msg


def r(value, digits):
    # JSON has no Infinity or NaN.
    return round(value, digits) if not (math.isinf(value) or math.isnan(value)) else 0


class Relay(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.clients = set()
        self.logs = collections.deque(maxlen=LOG_KEEP)
        self.log_id = 0
        self.sent_log_id = 0
        self.paths = {}
        self.paths_changed = False
        self.scans = {}
        self.scan_sent = {}
        self.models = None
        self.clock = None
        self.scan_sources = {}

    # ROS side, only subscriptions.

    def start_ros(self):
        rospy.init_node('live3d_relay')
        self.models = Latest('/gazebo/model_states', ModelStates)
        self.clock = Latest('/clock', Clock)
        for robot in ROBOTS:
            self.scan_sources[robot] = Latest(SCAN_TOPICS[robot], LaserScan)
            rospy.Subscriber('/%s/Navigation/driveTo/goal' % robot, driveToActionGoal,
                             self.on_path, callback_args=robot)
        rospy.Subscriber('/rosout_agg', Log, self.on_log, queue_size=100)

    def on_log(self, msg):
        if msg.name not in MISSION_NODES:
            return
        # Stamp entries with the sim clock, the clock the page shows.
        clock = self.clock.read()
        t = clock.clock.to_sec() if clock else msg.header.stamp.to_sec()
        with self.lock:
            self.log_id += 1
            self.logs.append({'id': self.log_id, 'node': msg.name, 'level': msg.level,
                              't': r(t, 2), 'msg': msg.msg})

    def on_path(self, msg, robot):
        goal = msg.goal
        with self.lock:
            self.paths[robot] = {'x': [r(v, 2) for v in goal.x_path], 'y': [r(v, 2) for v in goal.y_path]}
            self.paths_changed = True

    # Frames sent to the page.

    def read_models(self):
        states = self.models.read() if self.models else None
        if states is None:
            return {}
        out = {}
        for name, pose in zip(states.name, states.pose):
            p, q = pose.position, pose.orientation
            out[name] = [r(p.x, 3), r(p.y, 3), r(p.z, 3), r(q.x, 4), r(q.y, 4), r(q.z, 4), r(q.w, 4)]
        return out

    def read_scan(self, robot):
        scan = self.scan_sources[robot].read()
        return {'frame': scan.header.frame_id, 'angle_min': r(scan.angle_min, 5),
                'angle_increment': r(scan.angle_increment, 6), 'range_min': r(scan.range_min, 3),
                'range_max': r(scan.range_max, 3), 'ranges': [r(v, 2) for v in scan.ranges]}

    def update_scans(self, now):
        fresh = {}
        for robot, source in self.scan_sources.items():
            if source.msg is None or self.scan_sent.get(robot, (0, 0))[0] == source.count:
                continue
            if now - self.scan_sent.get(robot, (0, 0))[1] < 1.0 / SCAN_HZ:
                continue
            self.scan_sent[robot] = (source.count, now)
            fresh[robot] = self.read_scan(robot)
        with self.lock:
            self.scans.update(fresh)
        return fresh

    def base_frame(self, now):
        clock = self.clock.read() if self.clock else None
        return {'wall': round(now, 3), 'ros': self.models is not None,
                't': r(clock.clock.to_sec(), 3) if clock else None, 'models': self.read_models()}

    def snapshot(self):
        frame = self.base_frame(time.time())
        with self.lock:
            frame.update(snapshot=True, logs=list(self.logs), paths=dict(self.paths), scans=dict(self.scans))
        return json.dumps(frame, separators=(',', ':'))

    def tick(self):
        now = time.time()
        frame = self.base_frame(now)
        frame['scans'] = self.update_scans(now)
        with self.lock:
            frame['logs'] = [entry for entry in self.logs if entry['id'] > self.sent_log_id]
            self.sent_log_id = self.log_id
            if self.paths_changed:
                frame['paths'] = dict(self.paths)
                self.paths_changed = False
            clients = list(self.clients)
        data = json.dumps(frame, separators=(',', ':'))
        for client in clients:
            try:
                client.put_nowait(data)
            except Queue.Full:
                # A page that falls behind is dropped; it reconnects and gets a fresh snapshot.
                self.disconnect(client)

    def run(self):
        while not rospy.is_shutdown():
            self.tick()
            time.sleep(1.0 / FRAME_HZ)

    def connect(self):
        client = Queue.Queue(maxsize=2 * FRAME_HZ)
        with self.lock:
            self.clients.add(client)
        return client

    def disconnect(self, client):
        with self.lock:
            self.clients.discard(client)

    def connected(self, client):
        with self.lock:
            return client in self.clients


relay = Relay()


class Handler(BaseHTTPServer.BaseHTTPRequestHandler):
    server_version = 'refsors-live3d'

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path == '/events':
            return self.stream()
        if path not in FILES:
            return self.send_error(404)
        filename, content_type = FILES[path]
        with open(filename, 'rb') as f:
            body = f.read()
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(body)

    def stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        client = relay.connect()
        try:
            self.wfile.write('retry: 2000\n\ndata: %s\n\n' % relay.snapshot())
            self.wfile.flush()
            while relay.connected(client):
                try:
                    self.wfile.write('data: %s\n\n' % client.get(timeout=5))
                except Queue.Empty:
                    self.wfile.write(': keepalive\n\n')
                self.wfile.flush()
        except socket.error:
            pass
        finally:
            relay.disconnect(client)

    def log_message(self, *args):
        pass


class Server(SocketServer.ThreadingMixIn, BaseHTTPServer.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def handle_error(self, request, client_address):
        # A page that closes or reloads leaves a broken pipe; nothing to report.
        if not isinstance(sys.exc_info()[1], socket.error):
            BaseHTTPServer.HTTPServer.handle_error(self, request, client_address)


def main():
    server = Server(('0.0.0.0', PORT), Handler)
    http = threading.Thread(target=server.serve_forever, name='http')
    http.daemon = True
    http.start()
    # Blocks until roslaunch has started the ROS master.
    relay.start_ros()
    relay.run()


if __name__ == '__main__':
    main()
