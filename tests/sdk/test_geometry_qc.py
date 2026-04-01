from __future__ import annotations

import sys
import types
from pathlib import Path

import pytest

from sdk import ArticulatedObject, ArticulationType, Box, Mesh, Origin
from sdk._core.v0 import geometry_qc


def _write_disconnected_boxes_obj(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    centers = ((0.0, 0.0, 0.0), (0.45, 0.0, 0.0))
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    cube_faces = (
        (0, 1, 2),
        (0, 2, 3),
        (4, 6, 5),
        (4, 7, 6),
        (0, 4, 5),
        (0, 5, 1),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 4),
        (3, 4, 0),
    )
    for center_x, center_y, center_z in centers:
        base_index = len(vertices) + 1
        vertices.extend(
            [
                (center_x - 0.05, center_y - 0.05, center_z - 0.05),
                (center_x + 0.05, center_y - 0.05, center_z - 0.05),
                (center_x + 0.05, center_y + 0.05, center_z - 0.05),
                (center_x - 0.05, center_y + 0.05, center_z - 0.05),
                (center_x - 0.05, center_y - 0.05, center_z + 0.05),
                (center_x + 0.05, center_y - 0.05, center_z + 0.05),
                (center_x + 0.05, center_y + 0.05, center_z + 0.05),
                (center_x - 0.05, center_y + 0.05, center_z + 0.05),
            ]
        )
        faces.extend(tuple(base_index + idx for idx in face) for face in cube_faces)
    lines = [f"v {x} {y} {z}" for x, y, z in vertices]
    lines.extend(f"f {a} {b} {c}" for a, b, c in faces)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_single_box_obj(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    vertices = [
        (-0.05, -0.05, -0.05),
        (0.05, -0.05, -0.05),
        (0.05, 0.05, -0.05),
        (-0.05, 0.05, -0.05),
        (-0.05, -0.05, 0.05),
        (0.05, -0.05, 0.05),
        (0.05, 0.05, 0.05),
        (-0.05, 0.05, 0.05),
    ]
    faces = (
        (1, 2, 3),
        (1, 3, 4),
        (5, 7, 6),
        (5, 8, 7),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 8),
        (3, 8, 4),
        (4, 8, 5),
        (4, 5, 1),
    )
    lines = [f"v {x} {y} {z}" for x, y, z in vertices]
    lines.extend(f"f {a} {b} {c}" for a, b, c in faces)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _build_disjoint_collision_model() -> ArticulatedObject:
    model = ArticulatedObject(name="disjoint_collision_parts")

    left = model.part("left")
    left.visual(
        Box((0.1, 0.1, 0.1)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        name="left_box",
    )

    right = model.part("right")
    right.visual(
        Box((0.1, 0.1, 0.1)),
        origin=Origin(xyz=(1.0, 0.0, 0.05)),
        name="right_box",
    )

    return model


def test_collision_overlap_qc_prunes_disjoint_aabbs_before_fcl(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls = {"collide": 0}

    class FakeCollisionRequest:
        pass

    class FakeCollisionResult:
        pass

    def fake_collide(*_args: object, **_kwargs: object) -> int:
        calls["collide"] += 1
        raise AssertionError("fcl.collide should not run when AABB overlap thresholds fail")

    fake_fcl = types.SimpleNamespace(
        CollisionRequest=FakeCollisionRequest,
        CollisionResult=FakeCollisionResult,
        collide=fake_collide,
    )

    monkeypatch.setitem(sys.modules, "fcl", fake_fcl)
    monkeypatch.setattr(
        geometry_qc,
        "_collision_object_from_geometry",
        lambda geometry, **kwargs: object(),
    )

    overlaps = geometry_qc.find_geometry_overlaps(
        _build_disjoint_collision_model(),
        max_pose_samples=1,
        overlap_tol=1e-3,
        overlap_volume_tol=0.0,
    )

    assert overlaps == []
    assert calls["collide"] == 0


def test_load_fcl_mesh_rejects_obj_without_triangles(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    mesh_path = tmp_path / "empty_faces.obj"
    mesh_path.write_text(
        "\n".join(
            [
                "v 0 0 0",
                "v 1 0 0",
                "v 0 1 0",
                "",
            ]
        ),
        encoding="utf-8",
    )

    fake_fcl = types.SimpleNamespace(BVHModel=lambda: object())
    monkeypatch.setitem(sys.modules, "fcl", fake_fcl)

    class FakeTrimesh:
        def __init__(self) -> None:
            self.vertices = types.SimpleNamespace(size=9)
            self.faces = types.SimpleNamespace(size=0)

    fake_trimesh = types.SimpleNamespace(
        Trimesh=FakeTrimesh,
        load_mesh=lambda path, force="mesh": FakeTrimesh(),
    )
    monkeypatch.setitem(sys.modules, "trimesh", fake_trimesh)

    with pytest.raises(geometry_qc.ValidationError, match="no triangles"):
        geometry_qc._load_fcl_mesh(mesh_path, cache={}, scale=None)


def test_collision_object_from_geometry_supports_scaled_meshes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    mesh_relpath = "assets/meshes/scaled.obj"
    mesh_path = tmp_path / mesh_relpath
    mesh_path.parent.mkdir(parents=True, exist_ok=True)
    mesh_path.write_text("# placeholder\n", encoding="utf-8")

    applied_scales: list[tuple[float, float, float]] = []

    class FakeArray:
        def __init__(self, values: list[tuple[float, float, float]]) -> None:
            self._values = values
            self.size = len(values) * 3

        def __len__(self) -> int:
            return len(self._values)

    class FakeTrimesh:
        def __init__(self) -> None:
            self.vertices = FakeArray([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)])
            self.faces = FakeArray([(0, 1, 2)])

        def copy(self) -> "FakeTrimesh":
            return FakeTrimesh()

        def apply_scale(self, scale: tuple[float, float, float]) -> None:
            applied_scales.append(tuple(float(v) for v in scale))

    class FakeBVHModel:
        def beginModel(self, *_args: object) -> None:
            return None

        def addSubModel(self, *_args: object) -> None:
            return None

        def endModel(self) -> None:
            return None

    class FakeTransform:
        def __init__(self, rot: object, trans: object) -> None:
            self.rot = rot
            self.trans = trans

    class FakeCollisionObject:
        def __init__(self, shape: object, transform: object) -> None:
            self.shape = shape
            self.transform = transform

    fake_fcl = types.SimpleNamespace(
        BVHModel=FakeBVHModel,
        CollisionObject=FakeCollisionObject,
        Transform=FakeTransform,
    )
    fake_trimesh = types.SimpleNamespace(
        Trimesh=FakeTrimesh,
        load_mesh=lambda path, force="mesh": FakeTrimesh(),
    )

    monkeypatch.setitem(sys.modules, "fcl", fake_fcl)
    monkeypatch.setitem(sys.modules, "trimesh", fake_trimesh)

    collision_object = geometry_qc._collision_object_from_geometry(
        Mesh(filename=mesh_relpath, scale=(2.0, 3.0, 4.0)),
        elem_tf=(
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        ),
        asset_root=tmp_path,
        mesh_cache={},
    )

    assert isinstance(collision_object, FakeCollisionObject)
    assert applied_scales == [(2.0, 3.0, 4.0)]


def test_find_part_geometry_connectivity_findings_splits_mesh_components(tmp_path: Path) -> None:
    mesh_path = tmp_path / "assets" / "meshes" / "frame.obj"
    _write_disconnected_boxes_obj(mesh_path)

    model = ArticulatedObject(name="mesh_connectivity")
    frame = model.part("frame")
    frame.visual(Mesh(filename=mesh_path.as_posix()), origin=Origin(), name="frame_body")

    findings = geometry_qc.find_part_geometry_connectivity_findings(model)

    assert len(findings) == 1
    finding = findings[0]
    assert finding.part == "frame"
    assert finding.connected == 1
    assert finding.total == 2
    assert finding.disconnected == ("frame_body__component_002:Mesh",)


def test_find_geometry_overlaps_reports_mesh_component_name(tmp_path: Path) -> None:
    mesh_path = tmp_path / "assets" / "meshes" / "frame.obj"
    _write_disconnected_boxes_obj(mesh_path)

    model = ArticulatedObject(name="mesh_overlap")
    frame = model.part("frame")
    frame.visual(Mesh(filename=mesh_path.as_posix()), origin=Origin(), name="frame_body")

    blocker = model.part("blocker")
    blocker.visual(
        Box((0.12, 0.12, 0.12)),
        origin=Origin(xyz=(0.45, 0.0, 0.0)),
        name="blocker_box",
    )

    overlaps = geometry_qc.find_geometry_overlaps(
        model,
        max_pose_samples=1,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )

    assert len(overlaps) == 1
    overlap = overlaps[0]
    assert overlap.link_a == "blocker"
    assert overlap.link_b == "frame"
    assert overlap.elem_a_name == "blocker_box"
    assert overlap.elem_b_name == "frame_body__component_002"


def test_find_unsupported_parts_keeps_single_mesh_behavior(tmp_path: Path) -> None:
    mesh_path = tmp_path / "assets" / "meshes" / "child.obj"
    _write_single_box_obj(mesh_path)

    model = ArticulatedObject(name="single_mesh_isolated_part")
    base = model.part("base")
    base.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))

    child = model.part("child")
    child.visual(Mesh(filename=mesh_path.as_posix()), origin=Origin(), name="child_mesh")

    model.articulation(
        "base_to_child",
        ArticulationType.FIXED,
        parent=base,
        child=child,
        origin=Origin(xyz=(0.7, 0.0, 0.0)),
    )

    findings = geometry_qc.find_unsupported_parts(model, max_pose_samples=1)

    assert len(findings) == 1
    assert findings[0].part == "child"


def test_compute_part_world_transforms_applies_floating_origin_pose() -> None:
    model = ArticulatedObject(name="floating_pose_world_tf")

    base = model.part("base")
    base.visual(Box((0.2, 0.2, 0.2)), origin=Origin(xyz=(0.0, 0.0, 0.1)))

    payload = model.part("payload")
    payload.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))

    free_joint = model.articulation(
        "base_to_payload",
        ArticulationType.FLOATING,
        parent=base,
        child=payload,
        origin=Origin(xyz=(1.0, 0.0, 0.0)),
    )

    world = geometry_qc.compute_part_world_transforms(
        model,
        {free_joint.name: Origin(xyz=(0.25, -0.5, 0.75))},
    )

    payload_tf = world["payload"]
    assert payload_tf[0][3] == pytest.approx(1.25)
    assert payload_tf[1][3] == pytest.approx(-0.5)
    assert payload_tf[2][3] == pytest.approx(0.75)


def test_generate_pose_samples_keeps_floating_origin_values() -> None:
    model = ArticulatedObject(name="floating_pose_samples")
    base = model.part("base")
    payload = model.part("payload")
    joint = model.articulation(
        "base_to_payload",
        ArticulationType.FLOATING,
        parent=base,
        child=payload,
        meta={"qc_samples": [Origin(xyz=(0.1, 0.2, 0.3)), Origin(rpy=(0.0, 0.0, 0.4))]},
    )

    poses = geometry_qc.generate_pose_samples(model, max_samples=8, seed=0)

    assert poses[0][joint.name] == Origin()
    assert any(pose[joint.name] == Origin(xyz=(0.1, 0.2, 0.3)) for pose in poses)
    assert any(pose[joint.name] == Origin(rpy=(0.0, 0.0, 0.4)) for pose in poses)
