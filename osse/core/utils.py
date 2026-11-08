# -*- coding: utf-8 -*-
"""
SSE (Space Syntax Engine) Core Utilities
Zero-dependency helper functions.
"""

import math
from typing import List


def parse_radii(radii_str: str) -> List[float]:
    """
    Parses custom comma-separated radii string.
    Supports numbers (e.g. 400, 800, 1200, 2000) and 'n'/'inf' for global radius.
    """
    radii = []
    tokens = [t.strip().lower() for t in radii_str.split(",") if t.strip()]
    for t in tokens:
        if t in ["n", "inf", "global", "all"]:
            radii.append(float("inf"))
        else:
            try:
                val = float(t)
                if val > 0:
                    radii.append(val)
            except ValueError:
                pass
    finite = sorted([r for r in radii if not math.isinf(r)])
    has_inf = any(math.isinf(r) for r in radii)
    return finite + ([float("inf")] if has_inf else [])
