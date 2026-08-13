"""The lab panel: what is moving, what finished, and every device's status.

A lightweight, ROS-like, pure-Python layer — one process, many nodes, one web
display on :8770, no external broker. Each instrument in :mod:`tools`
contributes a thin ``node.py`` adapter; this package runs them together and
renders them.

    python -m dashboard              # http://127.0.0.1:8770/

The panel's first job is answering "is anything moving right now?" — it reads
:mod:`tools.occupancy`, the same interlock the arm and the UV-Vis carrier claim
before they actuate. See that module for why the two can never move together.
"""
from tools.node import Node

from .bus import Bus
from .oplog import OpLog
from .registry import Lab
from .server import serve

__all__ = ["Bus", "Lab", "Node", "OpLog", "serve"]
