from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple, Union

from .common import AABB, TestFailure, Vec3, _ExactElement
from .core import TestContextCoreMixin
from .expectations import TestContextExpectationMixin
from .model_checks import TestContextModelCheckMixin


@dataclass
class TestContext(TestContextExpectationMixin, TestContextModelCheckMixin, TestContextCoreMixin):
    model: object
    asset_root: Optional[Union[str, Path]] = None
    seed: int = 0

    checks_run: int = 0
    _checks: List[str] = field(default_factory=list)
    _failures: List[TestFailure] = field(default_factory=list)
    _warnings: List[str] = field(default_factory=list)
    _allow_pairs: Dict[Tuple[str, str], str] = field(default_factory=dict)
    _allowances: List[str] = field(default_factory=list)
    _allow_isolated_parts: Dict[str, str] = field(default_factory=dict)
    _allow_elems: List[Tuple[Tuple[str, str], Optional[str], Optional[str], str]] = field(
        default_factory=list, repr=False
    )
    _allow_coplanar_pairs: Dict[Tuple[str, str], str] = field(default_factory=dict)
    _allow_coplanar_elems: List[Tuple[Tuple[str, str], Optional[str], Optional[str], str]] = field(
        default_factory=list, repr=False
    )

    _pose: Dict[str, float] = field(default_factory=dict)
    _world_tfs_cache: Optional[Dict[str, object]] = None
    _parts_by_name_cache: Optional[Dict[str, object]] = field(default=None, repr=False)
    _mesh_aabb_cache: Dict[Path, AABB] = field(default_factory=dict, repr=False)
    _part_local_aabbs_cache: Dict[str, Tuple[AABB, ...]] = field(default_factory=dict, repr=False)
    _part_world_aabb_cache: Dict[str, Optional[AABB]] = field(default_factory=dict, repr=False)
    _exact_compiled_model_cache: Optional[object] = field(default=None, repr=False)
    _exact_local_elements_cache: Dict[str, Tuple[_ExactElement, ...]] = field(
        default_factory=dict, repr=False
    )
    _exact_world_elements_cache: Optional[Dict[str, Tuple[_ExactElement, ...]]] = field(
        default=None, repr=False
    )
    _exact_mesh_cache: Dict[tuple[Path, Optional[Vec3]], object] = field(
        default_factory=dict, repr=False
    )
    _mesh_vertex_cache: Dict[Path, Tuple[Vec3, ...]] = field(default_factory=dict, repr=False)
    _warned_deprecations: Set[str] = field(default_factory=set, repr=False)
    _model_validated_strict: bool = field(default=False, repr=False)
