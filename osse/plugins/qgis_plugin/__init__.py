# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) QGIS Plugin Entry Point
"""

import os
import sys

_PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
if _PLUGIN_DIR not in sys.path:
    sys.path.insert(0, _PLUGIN_DIR)


def classFactory(iface):
    from .plugin import SSEPlugin
    return SSEPlugin(iface)
