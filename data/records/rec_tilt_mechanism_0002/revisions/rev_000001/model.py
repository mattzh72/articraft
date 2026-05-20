from __future__ import annotations

from collections import Counter

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
BASE_X = 0.22
BASE_Y = 0.12
BASE_Z = 0.012

PLATE_T = 0.01
PLATE_Y = 0.085
PLATE_Z = 0.16
PLATE_CENTER_X = 0.075
YOKE_TIE_X = 0.15
YOKE_TIE_Y = 0.038
YOKE_TIE_Z = 0.014
PIVOT_Z_LOCAL = 0.083
PIVOT_ROD_RADIUS = 0.006
PIVOT_ROD_LENGTH = 0.158

FRAME_T = 0.012
FRAME_OUTER_Y = 0.13
FRAME_OUTER_Z = 0.09
FRAME_RAIL = 0.016
FRAME_OPEN_Y = FRAME_OUTER_Y - 2.0 * FRAME_RAIL
FRAME_OPEN_Z = FRAME_OUTER_Z - 2.0 * FRAME_RAIL
FRAME_CROSSBAR_Y = 0.086
FRAME_CROSSBAR_Z = 0.012
FRAME_CENTER_Z = 0.018
FRAME_CROSSBAR_CENTER_Z = FRAME_CENTER_Z + 0.014
FRAME_SLEEVE_R_OUTER = 0.012
FRAME_SLEEVE_R_INNER = 0.0072
FRAME_SLEEVE_LENGTH = 0.13

LOWER_TILT = -0.95
UPPER_TILT = 0.95


def _base_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BASE_X, BASE_Y, BASE_Z)
        .edges("|Z")
        .fillet(0.005)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.07, 0.0), (0.07, 0.0)])
        .slot2D(0.024, 0.008, 90)
        .cutThruAll()
    )


def _support_yoke_shape() -> cq.Workplane:
    lower_tie = (
        cq.Workplane("XY")
        .box(YOKE_TIE_X, YOKE_TIE_Y, YOKE_TIE_Z)
        .translate((0.0, 0.0, YOKE_TIE_Z / 2.0))
        .edges("|Z")
        .fillet(0.003)
    )
    left_plate = (
        cq.Workplane("XY")
        .box(PLATE_T, PLATE_Y, PLATE_Z)
        .translate((-PLATE_CENTER_X, 0.0, PLATE_Z / 2.0))
        .edges("|Z")
        .fillet(0.004)
    )
    right_plate = (
        cq.Workplane("XY")
        .box(PLATE_T, PLATE_Y, PLATE_Z)
        .translate((PLATE_CENTER_X, 0.0, PLATE_Z / 2.0))
        .edges("|Z")
        .fillet(0.004)
    )
    pivot_rod = (
        cq.Workplane("YZ")
        .circle(PIVOT_ROD_RADIUS)
        .extrude(PIVOT_ROD_LENGTH)
        .translate((-PIVOT_ROD_LENGTH / 2.0, 0.0, PIVOT_Z_LOCAL))
    )
    return lower_tie.union(left_plate).union(right_plate).union(pivot_rod)


def _payload_frame_shape() -> cq.Workplane:
    ring = (
        cq.Workplane("XY")
        .box(FRAME_T, FRAME_OUTER_Y, FRAME_OUTER_Z)
        .translate((0.0, 0.0, FRAME_CENTER_Z))
    )
    opening = (
        cq.Workplane("XY")
        .box(FRAME_T + 0.004, FRAME_OPEN_Y, FRAME_OPEN_Z)
        .translate((0.0, 0.0, FRAME_CENTER_Z))
    )
    crossbar = (
        cq.Workplane("XY")
        .box(FRAME_T, FRAME_CROSSBAR_Y, FRAME_CROSSBAR_Z)
        .translate((0.0, 0.0, FRAME_CROSSBAR_CENTER_Z))
    )
    sleeve = (
        cq.Workplane("YZ")
        .circle(FRAME_SLEEVE_R_OUTER)
        .extrude(FRAME_SLEEVE_LENGTH)
        .translate((-FRAME_SLEEVE_LENGTH / 2.0, 0.0, 0.0))
    )
    sleeve_bore = (
        cq.Workplane("YZ")
        .circle(FRAME_SLEEVE_R_INNER)
        .extrude(FRAME_SLEEVE_LENGTH + 0.004)
        .translate((-(FRAME_SLEEVE_LENGTH + 0.004) / 2.0, 0.0, 0.0))
    )
    return ring.cut(opening).union(crossbar).union(sleeve.cut(sleeve_bore))


def _load_welded_obj(path):
    vertices: list[tuple[float, float, float]] = []
    raw_triangles: list[tuple[int, int, int]] = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            if line.startswith("v "):
                _, xs, ys, zs, *_ = line.split()
                vertices.append((float(xs), float(ys), float(zs)))
            elif line.startswith("f "):
                face_indices: list[int] = []
                for token in line.split()[1:]:
                    index_text = token.split("/")[0]
                    if not index_text:
                        continue
                    index = int(index_text)
                    if index < 0:
                        index = len(vertices) + index
                    else:
                        index -= 1
                    face_indices.append(index)
                if len(face_indices) < 3:
                    continue
                anchor = face_indices[0]
                for i in range(1, len(face_indices) - 1):
                    raw_triangles.append((anchor, face_indices[i], face_indices[i + 1]))

    welded_vertices: list[tuple[float, float, float]] = []
    welded_map: dict[tuple[int, int, int], int] = {}

    def weld_index(vertex_index: int) -> int:
        vx, vy, vz = vertices[vertex_index]
        key = (round(vx * 1_000_000_000), round(vy * 1_000_000_000), round(vz * 1_000_000_000))
        if key not in welded_map:
            welded_map[key] = len(welded_vertices)
            welded_vertices.append((vx, vy, vz))
        return welded_map[key]

    welded_triangles = [
        tuple(weld_index(index) for index in triangle) for triangle in raw_triangles
    ]
    return welded_vertices, welded_triangles


def _mesh_extents(path) -> tuple[float, float, float]:
    vertices, _ = _load_welded_obj(path)
    xs = [vertex[0] for vertex in vertices]
    ys = [vertex[1] for vertex in vertices]
    zs = [vertex[2] for vertex in vertices]
    return max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)


def _assert_closed_mesh(path, label: str) -> None:
    vertices, triangles = _load_welded_obj(path)
    edge_counts: Counter[tuple[int, int]] = Counter()
    for triangle in triangles:
        if len(set(triangle)) != 3:
            raise AssertionError(f"{label} mesh contains a welded degenerate triangle.")
        i0, i1, i2 = triangle
        edge_counts[tuple(sorted((i0, i1)))] += 1
        edge_counts[tuple(sorted((i1, i2)))] += 1
        edge_counts[tuple(sorted((i2, i0)))] += 1

    bad_edges = [edge for edge, count in edge_counts.items() if count != 2]
    if bad_edges:
        raise AssertionError(f"{label} mesh has {len(bad_edges)} welded non-manifold/open edges.")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_bracket_mechanism", assets=ASSETS)
    model.material("powdercoat_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("anodized_aluminum", rgba=(0.72, 0.74, 0.78, 1.0))

    base_plate = model.part("base_plate")
    base_mesh = mesh_from_cadquery(_base_plate_shape(), "base_plate.obj", assets=ASSETS)
    base_plate.visual(base_mesh, material="powdercoat_dark")

    base_plate.inertial = Inertial.from_geometry(
        Box((BASE_X, BASE_Y, BASE_Z)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, BASE_Z / 2.0)),
    )

    support_yoke = model.part("support_yoke")
    yoke_mesh = mesh_from_cadquery(_support_yoke_shape(), "support_yoke.obj", assets=ASSETS)
    support_yoke.visual(yoke_mesh, material="powdercoat_dark")




    support_yoke.inertial = Inertial.from_geometry(
        Box((0.16, PLATE_Y, PLATE_Z)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, PLATE_Z / 2.0)),
    )

    payload_frame = model.part("payload_frame")
    payload_mesh = mesh_from_cadquery(_payload_frame_shape(), "payload_frame.obj", assets=ASSETS)
    payload_frame.visual(payload_mesh, material="anodized_aluminum")






    payload_frame.inertial = Inertial.from_geometry(
        Box((FRAME_T, FRAME_OUTER_Y, FRAME_OUTER_Z)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, FRAME_CENTER_Z / 2.0)),
    )

    model.articulation(
        "base_to_yoke",
        ArticulationType.FIXED,
        parent=base_plate,
        child=support_yoke,
        origin=Origin(xyz=(0.0, 0.0, BASE_Z)),
    )
    model.articulation(
        "yoke_to_payload",
        ArticulationType.REVOLUTE,
        parent=support_yoke,
        child=payload_frame,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z_LOCAL)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=LOWER_TILT,
            upper=UPPER_TILT,
            effort=8.0,
            velocity=1.5,
        ),
    )

    model.meta["mesh_files"] = {
        "base_plate": MESH_DIR / "base_plate.obj",
        "support_yoke": MESH_DIR / "support_yoke.obj",
        "payload_frame": MESH_DIR / "payload_frame.obj",
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("support_yoke", "base_plate", axes="xy", min_overlap=0.03)
    ctx.expect_aabb_overlap("payload_frame", "support_yoke", axes="xy", min_overlap=0.05)
    ctx.expect_origin_distance("payload_frame", "support_yoke", axes="xy", max_dist=0.005)
    ctx.expect_origin_gap("payload_frame", "base_plate", axis="z", min_gap=0.06)
    ctx.expect_aabb_gap("payload_frame", "base_plate", axis="z", max_gap=0.06, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "yoke_to_payload",
        "payload_frame",
        world_axis="y",
        direction="negative",
        min_delta=0.02,
    )

    with ctx.pose(yoke_to_payload=LOWER_TILT):
        ctx.expect_origin_distance("payload_frame", "support_yoke", axes="xy", max_dist=0.005)
        ctx.expect_aabb_gap("payload_frame", "base_plate", axis="z", max_gap=0.017, max_penetration=0.0)
        ctx.expect_aabb_overlap("payload_frame", "support_yoke", axes="xy", min_overlap=0.05)

    with ctx.pose(yoke_to_payload=UPPER_TILT):
        ctx.expect_origin_distance("payload_frame", "support_yoke", axes="xy", max_dist=0.005)
        ctx.expect_aabb_gap("payload_frame", "base_plate", axis="z", max_gap=0.017, max_penetration=0.0)
        ctx.expect_aabb_overlap("payload_frame", "support_yoke", axes="xy", min_overlap=0.05)

    revolute = object_model.get_articulation("yoke_to_payload")
    fixed_mount = object_model.get_articulation("base_to_yoke")
    assert revolute.axis == (1.0, 0.0, 0.0)
    assert revolute.motion_limits is not None
    assert revolute.motion_limits.lower == LOWER_TILT
    assert revolute.motion_limits.upper == UPPER_TILT
    assert fixed_mount.articulation_type == ArticulationType.FIXED

    base_dx, base_dy, base_dz = _mesh_extents(object_model.meta["mesh_files"]["base_plate"])
    yoke_dx, yoke_dy, yoke_dz = _mesh_extents(object_model.meta["mesh_files"]["support_yoke"])
    payload_dx, payload_dy, payload_dz = _mesh_extents(
        object_model.meta["mesh_files"]["payload_frame"]
    )

    assert 0.21 < base_dx < 0.23
    assert 0.11 < base_dy < 0.13
    assert 0.011 < base_dz < 0.0135
    assert 0.15 < yoke_dx < 0.17
    assert 0.08 < yoke_dy < 0.09
    assert 0.15 < yoke_dz < 0.17
    assert 0.125 < payload_dx < 0.135
    assert 0.125 < payload_dy < 0.135
    assert 0.089 < payload_dz < 0.095

    _assert_closed_mesh(object_model.meta["mesh_files"]["base_plate"], "base_plate")
    _assert_closed_mesh(object_model.meta["mesh_files"]["support_yoke"], "support_yoke")
    _assert_closed_mesh(object_model.meta["mesh_files"]["payload_frame"], "payload_frame")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
