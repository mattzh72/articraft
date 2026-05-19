from __future__ import annotations

# Compatibility facade for the public sdk._core.v0.mesh module.
from ._mesh import *  # noqa: F401,F403
from ._mesh.booleans import (
    _boolean_union_many,
    _geometry_from_manifold,
    _is_expected_boolean_fallback_error,
    _manifold_from_geometry,
)
from ._mesh.common import _ensure_ccw, _profile_points_2d, _triangulate_polygon
from ._mesh.sweeps import _tube_network_visual_mesh_from_paths
