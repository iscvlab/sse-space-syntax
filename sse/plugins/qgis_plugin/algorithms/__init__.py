# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) QGIS Algorithms Package
"""

from .algorithm_asa import SSEASAAlgorithm, OpenDepthmapASAAlgorithm
from .algorithm_axial import SSEAxialAlgorithm, OpenDepthmapAxialAlgorithm
from .algorithm_natural_roads import (
    SSENaturalRoadsAlgorithm,
    OpenDepthmapNaturalRoadsAlgorithm,
)
from .algorithm_prepare_osm import (
    SSEPrepareOSMAlgorithm,
    OpenDepthmapPrepareOSMAlgorithm,
)
from .algorithm_step_depth import (
    SSEStepDepthAlgorithm,
    OpenDepthmapStepDepthAlgorithm,
)

__all__ = [
    "SSEASAAlgorithm",
    "SSEStepDepthAlgorithm",
    "SSEAxialAlgorithm",
    "SSENaturalRoadsAlgorithm",
    "SSEPrepareOSMAlgorithm",
    "OpenDepthmapASAAlgorithm",
    "OpenDepthmapStepDepthAlgorithm",
    "OpenDepthmapAxialAlgorithm",
    "OpenDepthmapNaturalRoadsAlgorithm",
    "OpenDepthmapPrepareOSMAlgorithm",
]
