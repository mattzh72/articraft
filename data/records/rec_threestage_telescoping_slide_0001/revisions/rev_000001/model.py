from __future__ import annotations

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
MODEL_NAME = "long_reach_linear_slide"

BASE_LENGTH = 0.90
BASE_WIDTH = 0.100
BASE_HEIGHT = 0.070
BASE_WALL = 0.006
BASE_FLOOR = 0.008

MID_LENGTH = 0.72
MID_WIDTH = 0.084
MID_HEIGHT = 0.054
MID_WALL = 0.005
MID_FLOOR = 0.006

INNER_LENGTH = 0.54
INNER_WIDTH = 0.068
INNER_HEIGHT = 0.042
INNER_WALL = 0.0045
INNER_FLOOR = 0.005

BASE_TO_MID_ORIGIN = (0.050, 0.0, 0.010)
MID_TO_INNER_ORIGIN = (0.060, 0.0, 0.008)
BASE_TO_MID_UPPER = 0.42
MID_TO_INNER_UPPER = 0.32


def _channel_body(
    length: float,
    outer_width: float,
    outer_height: float,
    wall: float,
    floor: float,
):
    outer = (
        cq.Workplane("XY")
        .box(length, outer_width, outer_height)
        .translate((length / 2.0, 0.0, outer_height / 2.0))
    )
    cavity = (
        cq.Workplane("XY")
        .box(length + 0.004, outer_width - 2.0 * wall, outer_height - floor)
        .translate((length / 2.0, 0.0, floor + (outer_height - floor) / 2.0))
    )
    return outer.cut(cavity)


def _cut_side_windows(
    shape,
    *,
    length: float,
    outer_width: float,
    wall: float,
    slot_length: float,
    slot_height: float,
    slot_base_z: float,
):
    for sign in (-1.0, 1.0):
        cutter = (
            cq.Workplane("XY")
            .box(slot_length, wall + 0.003, slot_height)
            .translate(
                (
                    length / 2.0,
                    sign * (outer_width / 2.0 - wall / 2.0),
                    slot_base_z + slot_height / 2.0,
                )
            )
        )
        shape = shape.cut(cutter)
    return shape


def _cut_floor_holes(shape, *, x_positions: tuple[float, ...], diameter: float, depth: float):
    if not x_positions:
        return shape
    holes = (
        cq.Workplane("XY")
        .pushPoints([(x, 0.0) for x in x_positions])
        .circle(diameter / 2.0)
        .extrude(depth)
    )
    return shape.cut(holes)


def _add_slide_collisions(
    part,
    *,
    length: float,
    outer_width: float,
    outer_height: float,
    wall: float,
    floor: float,
):
    wall_y = (outer_width - wall) / 2.0





def _make_base_shape():
    shape = _channel_body(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_HEIGHT,
        BASE_WALL,
        BASE_FLOOR,
    )
    shape = _cut_side_windows(
        shape,
        length=BASE_LENGTH,
        outer_width=BASE_WIDTH,
        wall=BASE_WALL,
        slot_length=0.56,
        slot_height=0.020,
        slot_base_z=0.018,
    )
    shape = _cut_floor_holes(
        shape,
        x_positions=(0.12, 0.34, 0.56, 0.78),
        diameter=0.009,
        depth=BASE_HEIGHT,
    )

    rear_pad = cq.Workplane("XY").box(0.040, 0.062, 0.020).translate((0.020, 0.0, 0.010))
    front_stop = (
        cq.Workplane("XY").box(0.024, 0.054, 0.016).translate((BASE_LENGTH - 0.012, 0.0, 0.008))
    )
    upper_rib_left = (
        cq.Workplane("XY")
        .box(0.42, 0.008, 0.008)
        .translate((0.50, -(BASE_WIDTH / 2.0 - 0.004), BASE_HEIGHT + 0.004))
    )
    upper_rib_right = upper_rib_left.mirror(mirrorPlane="XZ")

    return (
        shape.union(rear_pad).union(front_stop).union(upper_rib_left).union(upper_rib_right).val()
    )


def _make_mid_shape():
    shape = _channel_body(
        MID_LENGTH,
        MID_WIDTH,
        MID_HEIGHT,
        MID_WALL,
        MID_FLOOR,
    )
    shape = _cut_side_windows(
        shape,
        length=MID_LENGTH,
        outer_width=MID_WIDTH,
        wall=MID_WALL,
        slot_length=0.44,
        slot_height=0.016,
        slot_base_z=0.016,
    )
    shape = _cut_floor_holes(
        shape,
        x_positions=(0.10, 0.28, 0.46, 0.62),
        diameter=0.007,
        depth=MID_HEIGHT,
    )

    rear_bearing_block = cq.Workplane("XY").box(0.032, 0.052, 0.018).translate((0.016, 0.0, 0.009))
    deck = (
        cq.Workplane("XY")
        .box(0.220, 0.054, 0.010)
        .translate((MID_LENGTH - 0.160, 0.0, MID_HEIGHT + 0.005))
    )
    left_cheek = (
        cq.Workplane("XY")
        .box(0.080, 0.008, 0.026)
        .translate((MID_LENGTH - 0.090, -(0.027), MID_HEIGHT + 0.013))
    )
    right_cheek = left_cheek.mirror(mirrorPlane="XZ")

    return shape.union(rear_bearing_block).union(deck).union(left_cheek).union(right_cheek).val()


def _make_inner_shape():
    shape = _channel_body(
        INNER_LENGTH,
        INNER_WIDTH,
        INNER_HEIGHT,
        INNER_WALL,
        INNER_FLOOR,
    )
    shape = _cut_side_windows(
        shape,
        length=INNER_LENGTH,
        outer_width=INNER_WIDTH,
        wall=INNER_WALL,
        slot_length=0.34,
        slot_height=0.014,
        slot_base_z=0.014,
    )
    shape = _cut_floor_holes(
        shape,
        x_positions=(0.10, 0.24, 0.38),
        diameter=0.006,
        depth=INNER_HEIGHT,
    )

    top_plate = (
        cq.Workplane("XY")
        .box(0.280, 0.090, 0.010)
        .translate((INNER_LENGTH - 0.150, 0.0, INNER_HEIGHT + 0.005))
    )
    front_plate = (
        cq.Workplane("XY").box(0.016, 0.100, 0.072).translate((INNER_LENGTH - 0.008, 0.0, 0.036))
    )
    top_brace = (
        cq.Workplane("XY")
        .box(0.050, 0.090, 0.018)
        .translate((INNER_LENGTH - 0.040, 0.0, INNER_HEIGHT + 0.019))
    )

    front_holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-0.028, 0.020),
                (0.028, 0.020),
                (-0.028, 0.052),
                (0.028, 0.052),
            ]
        )
        .circle(0.004)
        .extrude(0.024)
        .translate((INNER_LENGTH - 0.020, 0.0, 0.0))
    )

    shape = shape.union(top_plate).union(front_plate).union(top_brace)
    return shape.cut(front_holes).val()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name=MODEL_NAME, assets=ASSETS)

    model.material("rail_oxide", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("ground_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("carriage_silver", rgba=(0.80, 0.82, 0.85, 1.0))
    model.material("polymer_black", rgba=(0.09, 0.09, 0.10, 1.0))

    base = model.part("base_section")
    base_shape = _make_base_shape()
    base.visual(
        mesh_from_cadquery(base_shape, "base_section.obj", assets=ASSETS), material="rail_oxide"
    )
    base.visual(
        Box((0.016, 0.050, 0.010)),
        origin=Origin(xyz=(BASE_LENGTH - 0.010, 0.0, 0.005)),
        material="polymer_black",
    )
    _add_slide_collisions(
        base,
        length=BASE_LENGTH,
        outer_width=BASE_WIDTH,
        outer_height=BASE_HEIGHT,
        wall=BASE_WALL,
        floor=BASE_FLOOR,
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)),
        mass=6.8,
        origin=Origin(xyz=(BASE_LENGTH / 2.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    mid = model.part("mid_section")
    mid_shape = _make_mid_shape()
    mid.visual(
        mesh_from_cadquery(mid_shape, "mid_section.obj", assets=ASSETS), material="ground_steel"
    )
    mid.visual(
        Box((0.014, 0.042, 0.008)),
        origin=Origin(xyz=(0.012, 0.0, 0.004)),
        material="polymer_black",
    )
    _add_slide_collisions(
        mid,
        length=MID_LENGTH,
        outer_width=MID_WIDTH,
        outer_height=MID_HEIGHT,
        wall=MID_WALL,
        floor=MID_FLOOR,
    )
    mid.inertial = Inertial.from_geometry(
        Box((MID_LENGTH, MID_WIDTH, MID_HEIGHT)),
        mass=4.1,
        origin=Origin(xyz=(MID_LENGTH / 2.0, 0.0, MID_HEIGHT / 2.0)),
    )

    inner = model.part("inner_section")
    inner_shape = _make_inner_shape()
    inner.visual(
        mesh_from_cadquery(inner_shape, "inner_section.obj", assets=ASSETS),
        material="carriage_silver",
    )
    inner.visual(
        Box((0.014, 0.086, 0.010)),
        origin=Origin(xyz=(INNER_LENGTH - 0.010, 0.0, INNER_HEIGHT + 0.005)),
        material="polymer_black",
    )
    _add_slide_collisions(
        inner,
        length=INNER_LENGTH,
        outer_width=INNER_WIDTH,
        outer_height=INNER_HEIGHT,
        wall=INNER_WALL,
        floor=INNER_FLOOR,
    )
    inner.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT + 0.040)),
        mass=2.7,
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, (INNER_HEIGHT + 0.040) / 2.0)),
    )

    model.articulation(
        "base_to_mid",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mid,
        origin=Origin(xyz=BASE_TO_MID_ORIGIN),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.40,
            lower=0.0,
            upper=BASE_TO_MID_UPPER,
        ),
    )
    model.articulation(
        "mid_to_inner",
        ArticulationType.PRISMATIC,
        parent=mid,
        child=inner,
        origin=Origin(xyz=MID_TO_INNER_ORIGIN),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.45,
            lower=0.0,
            upper=MID_TO_INNER_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("mid_section", "base_section", axes="xy", min_overlap=0.06)
    ctx.expect_aabb_overlap("inner_section", "mid_section", axes="xy", min_overlap=0.05)
    ctx.expect_aabb_overlap("inner_section", "base_section", axes="xy", min_overlap=0.04)
    ctx.expect_origin_distance("mid_section", "base_section", axes="xy", max_dist=0.05)
    ctx.expect_origin_distance("inner_section", "mid_section", axes="xy", max_dist=0.07)
    ctx.expect_origin_distance("inner_section", "base_section", axes="xy", max_dist=0.12)
    ctx.expect_joint_motion_axis(
        "base_to_mid",
        "mid_section",
        world_axis="x",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "mid_to_inner",
        "inner_section",
        world_axis="x",
        direction="positive",
        min_delta=0.05,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
