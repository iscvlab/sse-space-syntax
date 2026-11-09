# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) QGIS Processing Provider
Registers SSE (Space Syntax Engine) algorithms into the QGIS Processing Toolbox.
"""

import os
import sys
from qgis.core import QgsProcessingProvider
from qgis.PyQt.QtGui import QIcon

_PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
if _PLUGIN_DIR not in sys.path:
    sys.path.insert(0, _PLUGIN_DIR)

from .algorithms.algorithm_asa import SSEASAAlgorithm  # noqa: E402
from .algorithms.algorithm_axial import SSEAxialAlgorithm  # noqa: E402
from .algorithms.algorithm_natural_roads import (  # noqa: E402
    SSENaturalRoadsAlgorithm
)
from .algorithms.algorithm_prepare_osm import (  # noqa: E402
    SSEPrepareOSMAlgorithm
)
from .algorithms.algorithm_step_depth import (  # noqa: E402
    SSEStepDepthAlgorithm
)


class SSEProvider(QgsProcessingProvider):
    def loadAlgorithms(self, *args, **kwargs):
        self.addAlgorithm(SSEASAAlgorithm())
        self.addAlgorithm(SSEStepDepthAlgorithm())
        self.addAlgorithm(SSEAxialAlgorithm())
        self.addAlgorithm(SSENaturalRoadsAlgorithm())
        self.addAlgorithm(SSEPrepareOSMAlgorithm())

    def id(self, *args, **kwargs):
        return "sse"

    def name(self, *args, **kwargs):
        return "SSE — Space Syntax Engine"

    def icon(self):
        icon_path = os.path.join(os.path.dirname(__file__), "icon.png")
        if os.path.exists(icon_path):
            return QIcon(icon_path)
        return QgsProcessingProvider.icon(self)

    def longName(self):
        return self.name()


OpenDepthmapProvider = SSEProvider
