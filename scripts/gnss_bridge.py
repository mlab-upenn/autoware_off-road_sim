#!/usr/bin/env python3
"""GNSS publisher bridge for Isaac Sim (Python 3.10 subprocess).

Isaac Sim runs Python 3.12, but rclpy is compiled only for Python 3.10 on
ROS Humble.  launch_sim.py spawns this script with /usr/bin/python3.10 so
that GNSS messages can be published via rclpy without a version conflict.

Protocol (stdin, one record per line):
  topic<TAB>frame_id<TAB>lat<TAB>lon<TAB>alt<TAB>h_std<TAB>v_std<TAB>stamp_ns

  - lat, lon, alt  : float (degrees / metres)
  - h_std, v_std   : 1-sigma noise in metres (squared to covariance here)
  - stamp_ns       : wall-clock nanoseconds since epoch (int)

Writes "ready\\n" to stdout once the rclpy node is up; the parent reads this
to avoid sending messages before the bridge is initialised.

Exits cleanly when stdin is closed (parent process exits).
"""

import sys
import os
import signal

# Ignore SIGINT — the parent (Isaac Sim) owns the shutdown sequence.
signal.signal(signal.SIGINT, signal.SIG_IGN)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus

rclpy.init()

_node = Node("isaacsim_gnss_bridge")
_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)
_pubs = {}  # topic -> Publisher (created lazily on first message)

# Signal readiness to the parent process.
sys.stdout.write("ready\n")
sys.stdout.flush()

for _line in sys.stdin:
    _line = _line.rstrip("\n")
    if not _line:
        continue
    try:
        _topic, _fid, _lat, _lon, _alt, _hs, _vs, _ts = _line.split("\t")
        _lat, _lon, _alt = float(_lat), float(_lon), float(_alt)
        _ch = float(_hs) ** 2
        _cv = float(_vs) ** 2
        _ts = int(_ts)

        if _topic not in _pubs:
            _pubs[_topic] = _node.create_publisher(NavSatFix, _topic, _qos)

        _msg = NavSatFix()
        _msg.header.frame_id          = _fid
        _msg.header.stamp.sec         = _ts // 1_000_000_000
        _msg.header.stamp.nanosec     = _ts % 1_000_000_000
        _msg.status.status            = NavSatStatus.STATUS_FIX
        _msg.status.service           = NavSatStatus.SERVICE_GPS
        _msg.latitude                 = _lat
        _msg.longitude                = _lon
        _msg.altitude                 = _alt
        _msg.position_covariance      = [_ch, 0.0, 0.0,
                                         0.0, _ch, 0.0,
                                         0.0, 0.0, _cv]
        _msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        _pubs[_topic].publish(_msg)
    except Exception:
        pass

try:
    rclpy.shutdown()
except Exception:
    pass
