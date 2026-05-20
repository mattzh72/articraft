from __future__ import annotations

from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
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
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)
# >>> USER_CODE_START
# In sdk_hybrid, author visual meshes with cadquery + mesh_from_cadquery.
import cadquery as cq
import math


POST_SIZE = 0.09
POST_HEIGHT = 1.45
POST_CAP_HEIGHT = 0.02
LEFT_POST_CENTER_X = -0.55
RIGHT_POST_CENTER_X = 0.55

HINGE_AXIS_X = -0.455
HINGE_AXIS_Y = 0.065
HINGE_ORIGIN_Z = 0.30

GATE_WIDTH = 0.94
GATE_HEIGHT = 1.20
GATE_THICKNESS = 0.04
GATE_BOTTOM_Z = 0.06
FRAME_SECTION = 0.075
SLAT_COUNT = 5
SLAT_THICKNESS = 0.016
SLAT_PROUD = 0.006

LOWER_HINGE_LOCAL_Z = 0.0
UPPER_HINGE_LOCAL_Z = 0.62
LATCH_LOCAL_Z = 0.54
LEAF_CENTER_LOCAL_Z = GATE_BOTTOM_Z + (GATE_HEIGHT / 2.0) - HINGE_ORIGIN_Z


def _require_cadquery():
    return cq


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    cq_mod = _require_cadquery()
    return cq_mod.Workplane("XY").box(*size).translate(center)


def _cq_cylinder(radius: float, height: float, center: tuple[float, float, float]):
    cq_mod = _require_cadquery()
    return (
        cq_mod.Workplane("XY")
        .circle(radius)
        .extrude(height)
        .translate((center[0], center[1], center[2] - (height / 2.0)))
    )


def _mesh_bounds(
    path: Path,
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.startswith("v "):
                continue
            _, sx, sy, sz = line.split()[:4]
            xs.append(float(sx))
            ys.append(float(sy))
            zs.append(float(sz))
    if not xs:
        raise AssertionError(f"{path.name} did not contain any OBJ vertices")
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


def _make_frame_posts_shape():
    shape = _cq_box(
        (POST_SIZE, POST_SIZE, POST_HEIGHT), (LEFT_POST_CENTER_X, 0.0, POST_HEIGHT / 2.0)
    )
    shape = shape.union(
        _cq_box((POST_SIZE, POST_SIZE, POST_HEIGHT), (RIGHT_POST_CENTER_X, 0.0, POST_HEIGHT / 2.0))
    )
    shape = shape.union(
        _cq_box(
            (POST_SIZE + 0.018, POST_SIZE + 0.018, POST_CAP_HEIGHT),
            (LEFT_POST_CENTER_X, 0.0, POST_HEIGHT + (POST_CAP_HEIGHT / 2.0)),
        )
    )
    shape = shape.union(
        _cq_box(
            (POST_SIZE + 0.018, POST_SIZE + 0.018, POST_CAP_HEIGHT),
            (RIGHT_POST_CENTER_X, 0.0, POST_HEIGHT + (POST_CAP_HEIGHT / 2.0)),
        )
    )
    return shape


def _make_frame_hardware_shape():
    left_post_inner_face_x = LEFT_POST_CENTER_X + (POST_SIZE / 2.0)
    hinge_bridge_x = (left_post_inner_face_x + HINGE_AXIS_X) / 2.0
    hinge_bridge_len = HINGE_AXIS_X - left_post_inner_face_x

    lower_strap = _cq_box((hinge_bridge_len, 0.008, 0.055), (hinge_bridge_x, 0.053, HINGE_ORIGIN_Z))
    upper_strap = _cq_box(
        (hinge_bridge_len, 0.008, 0.055),
        (hinge_bridge_x, 0.053, HINGE_ORIGIN_Z + UPPER_HINGE_LOCAL_Z),
    )
    lower_pintle = _cq_cylinder(0.012, 0.065, (HINGE_AXIS_X, HINGE_AXIS_Y, HINGE_ORIGIN_Z))
    upper_pintle = _cq_cylinder(
        0.012,
        0.065,
        (HINGE_AXIS_X, HINGE_AXIS_Y, HINGE_ORIGIN_Z + UPPER_HINGE_LOCAL_Z),
    )

    keeper = _cq_box(
        (0.034, 0.026, 0.11),
        (
            RIGHT_POST_CENTER_X - (POST_SIZE / 2.0) - 0.017,
            HINGE_AXIS_Y,
            HINGE_ORIGIN_Z + LATCH_LOCAL_Z,
        ),
    )
    keeper_lip = _cq_box(
        (0.010, 0.012, 0.08),
        (
            RIGHT_POST_CENTER_X - (POST_SIZE / 2.0) - 0.005,
            HINGE_AXIS_Y - 0.010,
            HINGE_ORIGIN_Z + LATCH_LOCAL_Z,
        ),
    )

    shape = lower_strap.union(upper_strap)
    shape = shape.union(lower_pintle)
    shape = shape.union(upper_pintle)
    shape = shape.union(keeper)
    shape = shape.union(keeper_lip)
    return shape


def _make_leaf_wood_shape():
    inner_width = GATE_WIDTH - (2.0 * FRAME_SECTION)
    inner_height = GATE_HEIGHT - (2.0 * FRAME_SECTION)

    left_stile = _cq_box(
        (FRAME_SECTION, GATE_THICKNESS, GATE_HEIGHT),
        (FRAME_SECTION / 2.0, 0.0, LEAF_CENTER_LOCAL_Z),
    )
    right_stile = _cq_box(
        (FRAME_SECTION, GATE_THICKNESS, GATE_HEIGHT),
        (GATE_WIDTH - (FRAME_SECTION / 2.0), 0.0, LEAF_CENTER_LOCAL_Z),
    )
    bottom_rail = _cq_box(
        (inner_width, GATE_THICKNESS, FRAME_SECTION),
        (GATE_WIDTH / 2.0, 0.0, (GATE_BOTTOM_Z - HINGE_ORIGIN_Z) + (FRAME_SECTION / 2.0)),
    )
    top_rail = _cq_box(
        (inner_width, GATE_THICKNESS, FRAME_SECTION),
        (
            GATE_WIDTH / 2.0,
            0.0,
            (GATE_BOTTOM_Z - HINGE_ORIGIN_Z) + GATE_HEIGHT - (FRAME_SECTION / 2.0),
        ),
    )

    brace_length = math.sqrt((inner_width * inner_width) + (inner_height * inner_height))
    brace_angle_deg = math.degrees(math.atan2(inner_height, inner_width))
    brace = (
        _require_cadquery()
        .Workplane("XY")
        .box(brace_length, GATE_THICKNESS * 0.80, 0.03)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), brace_angle_deg)
        .translate((FRAME_SECTION + (inner_width / 2.0), 0.0, LEAF_CENTER_LOCAL_Z))
    )

    shape = left_stile.union(right_stile)
    shape = shape.union(bottom_rail)
    shape = shape.union(top_rail)
    shape = shape.union(brace)

    slat_clear_width = inner_width - (SLAT_COUNT * 0.085)
    slat_gap = slat_clear_width / (SLAT_COUNT + 1.0)
    slat_height = GATE_HEIGHT - (2.0 * FRAME_SECTION) - 0.02
    slat_center_z = LEAF_CENTER_LOCAL_Z
    for index in range(SLAT_COUNT):
        slat_center_x = FRAME_SECTION + slat_gap + 0.0425 + (index * (0.085 + slat_gap))
        slat = _cq_box(
            (0.085, SLAT_THICKNESS, slat_height),
            (slat_center_x, SLAT_PROUD / 2.0, slat_center_z),
        )
        shape = shape.union(slat)

    return shape


def _make_leaf_hardware_shape():
    lower_strap = _cq_box(
        (0.14, 0.008, 0.055), (0.08, (GATE_THICKNESS / 2.0) + 0.004, LOWER_HINGE_LOCAL_Z)
    )
    upper_strap = _cq_box(
        (0.14, 0.008, 0.055),
        (0.08, (GATE_THICKNESS / 2.0) + 0.004, UPPER_HINGE_LOCAL_Z),
    )
    lower_barrel = _cq_cylinder(0.016, 0.085, (0.0, 0.0, LOWER_HINGE_LOCAL_Z))
    upper_barrel = _cq_cylinder(0.016, 0.085, (0.0, 0.0, UPPER_HINGE_LOCAL_Z))

    latch_box = _cq_box(
        (0.075, 0.03, 0.14),
        (GATE_WIDTH - 0.040, (GATE_THICKNESS / 2.0) + 0.008, LATCH_LOCAL_Z),
    )
    latch_tongue = _cq_box((0.012, 0.012, 0.04), (GATE_WIDTH + 0.006, 0.0, LATCH_LOCAL_Z))
    latch_pull = _cq_box((0.012, 0.026, 0.08), (GATE_WIDTH - 0.056, 0.0, LATCH_LOCAL_Z))

    shape = lower_strap.union(upper_strap)
    shape = shape.union(lower_barrel)
    shape = shape.union(upper_barrel)
    shape = shape.union(latch_box)
    shape = shape.union(latch_tongue)
    shape = shape.union(latch_pull)
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garden_gate", assets=ASSETS)

    model.material("painted_wood", rgba=(0.58, 0.42, 0.28, 1.0))
    model.material("galvanized_steel", rgba=(0.45, 0.48, 0.52, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_make_frame_posts_shape(), MESH_DIR / "frame_posts.obj"),
        material="painted_wood",
    )
    frame.visual(
        mesh_from_cadquery(_make_frame_hardware_shape(), MESH_DIR / "frame_hardware.obj"),
        material="galvanized_steel",
    )



    frame.inertial = Inertial.from_geometry(
        Box((1.19, 0.11, POST_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, POST_HEIGHT / 2.0)),
    )

    leaf = model.part("leaf")
    leaf.visual(
        mesh_from_cadquery(_make_leaf_wood_shape(), MESH_DIR / "leaf_wood.obj"),
        material="painted_wood",
    )
    leaf.visual(
        mesh_from_cadquery(_make_leaf_hardware_shape(), MESH_DIR / "leaf_hardware.obj"),
        material="galvanized_steel",
    )


    leaf.inertial = Inertial.from_geometry(
        Box((GATE_WIDTH, GATE_THICKNESS, GATE_HEIGHT)),
        mass=11.0,
        origin=Origin(xyz=(GATE_WIDTH / 2.0, 0.0, LEAF_CENTER_LOCAL_Z)),
    )

    model.articulation(
        "frame_to_leaf",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="leaf",
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, HINGE_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=25.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_joint_motion_axis(
        "frame_to_leaf",
        "leaf",
        world_axis="y",
        direction="positive",
        min_delta=0.10,
    )

    frame_posts_bounds = _mesh_bounds(MESH_DIR / "frame_posts.obj")
    leaf_wood_bounds = _mesh_bounds(MESH_DIR / "leaf_wood.obj")
    leaf_hardware_bounds = _mesh_bounds(MESH_DIR / "leaf_hardware.obj")
    frame_hardware_bounds = _mesh_bounds(MESH_DIR / "frame_hardware.obj")

    if frame_posts_bounds[0][0] > LEFT_POST_CENTER_X - (POST_SIZE / 2.0) + 0.005:
        raise AssertionError("Frame posts mesh is missing the left support post.")
    if frame_posts_bounds[0][1] < RIGHT_POST_CENTER_X + (POST_SIZE / 2.0) - 0.005:
        raise AssertionError("Frame posts mesh is missing the right support post.")
    if frame_posts_bounds[2][1] < POST_HEIGHT + POST_CAP_HEIGHT - 0.01:
        raise AssertionError("Frame posts mesh is missing the post-cap height detail.")
    if leaf_wood_bounds[0][0] > 0.01:
        raise AssertionError("Leaf wood mesh does not start at the hinge-side edge.")
    if leaf_wood_bounds[0][1] < GATE_WIDTH - 0.01:
        raise AssertionError("Leaf wood mesh does not reach the latch-side edge.")
    if leaf_wood_bounds[2][0] > (GATE_BOTTOM_Z - HINGE_ORIGIN_Z) + 0.01:
        raise AssertionError("Leaf wood mesh bottom sits too high above the lower rail.")
    if leaf_wood_bounds[2][1] < (GATE_BOTTOM_Z - HINGE_ORIGIN_Z + GATE_HEIGHT) - 0.01:
        raise AssertionError("Leaf wood mesh top is shorter than the intended rectangular gate.")

    if leaf_hardware_bounds[0][0] > 0.01:
        raise AssertionError("Leaf hardware mesh is missing hinge barrels at the joint axis.")
    if leaf_hardware_bounds[0][1] < GATE_WIDTH:
        raise AssertionError("Leaf hardware mesh is missing the latch tongue overhang.")
    if leaf_hardware_bounds[1][1] < (GATE_THICKNESS / 2.0) + 0.01:
        raise AssertionError("Leaf hardware mesh is missing front-mounted strap or latch depth.")
    if frame_hardware_bounds[0][0] > HINGE_AXIS_X + 0.01:
        raise AssertionError("Frame hardware mesh is missing hinge-side pintle detail.")
    if frame_hardware_bounds[0][1] < RIGHT_POST_CENTER_X - (POST_SIZE / 2.0):
        raise AssertionError("Frame hardware mesh is missing the latch keeper detail.")
    if frame_hardware_bounds[1][1] < HINGE_AXIS_Y + 0.01:
        raise AssertionError(
            "Frame hardware mesh is missing forward-projecting hinge or keeper detail."
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
