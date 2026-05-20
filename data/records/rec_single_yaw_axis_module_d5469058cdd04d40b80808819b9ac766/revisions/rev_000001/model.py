from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, sqrt

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.240
BASE_WIDTH = 0.160
BASE_HEIGHT = 0.042
BASE_EDGE_RADIUS = 0.006
TOP_RECESS_DEPTH = 0.005
MOUNT_HOLE_RADIUS = 0.005

BEARING_COLLAR_RADIUS = 0.042
BEARING_COLLAR_HEIGHT = 0.006

PLATTER_RADIUS = 0.072
PLATTER_THICKNESS = 0.012
PLATTER_GROOVE_INNER_RADIUS = 0.041
PLATTER_GROOVE_OUTER_RADIUS = 0.050
PLATTER_GROOVE_DEPTH = 0.0015

WORK_PAD_SIZE = 0.056
WORK_PAD_THICKNESS = 0.008
WORK_PAD_SLOT_WIDTH = 0.008
WORK_PAD_SLOT_DEPTH = 0.0016


def _build_base_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .rect(BASE_LENGTH, BASE_WIDTH)
        .extrude(BASE_HEIGHT)
    )

    top_recess = (
        cq.Workplane("XY")
        .box(BASE_LENGTH - 0.040, BASE_WIDTH - 0.030, TOP_RECESS_DEPTH)
        .translate((0.0, 0.0, BASE_HEIGHT - TOP_RECESS_DEPTH / 2.0))
    )

    mount_holes = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (BASE_LENGTH / 2.0 - 0.030, BASE_WIDTH / 2.0 - 0.024),
                (BASE_LENGTH / 2.0 - 0.030, -BASE_WIDTH / 2.0 + 0.024),
                (-BASE_LENGTH / 2.0 + 0.030, BASE_WIDTH / 2.0 - 0.024),
                (-BASE_LENGTH / 2.0 + 0.030, -BASE_WIDTH / 2.0 + 0.024),
            ]
        )
        .circle(MOUNT_HOLE_RADIUS)
        .extrude(BASE_HEIGHT + BEARING_COLLAR_HEIGHT + 0.004)
        .translate((0.0, 0.0, -0.002))
    )

    bearing_collar = (
        cq.Workplane("XY")
        .circle(BEARING_COLLAR_RADIUS)
        .extrude(TOP_RECESS_DEPTH + BEARING_COLLAR_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT - TOP_RECESS_DEPTH))
    )

    return body.cut(top_recess).cut(mount_holes).union(bearing_collar)


def _build_platter_shape() -> cq.Workplane:
    disk = cq.Workplane("XY").circle(PLATTER_RADIUS).extrude(PLATTER_THICKNESS)
    annular_groove = (
        cq.Workplane("XY")
        .circle(PLATTER_GROOVE_OUTER_RADIUS)
        .circle(PLATTER_GROOVE_INNER_RADIUS)
        .extrude(PLATTER_GROOVE_DEPTH)
        .translate((0.0, 0.0, PLATTER_THICKNESS - PLATTER_GROOVE_DEPTH))
    )
    return disk.cut(annular_groove)


def _build_work_pad_shape() -> cq.Workplane:
    pad = (
        cq.Workplane("XY")
        .rect(WORK_PAD_SIZE, WORK_PAD_SIZE)
        .extrude(WORK_PAD_THICKNESS)
    )

    cross_slot_x = (
        cq.Workplane("XY")
        .box(WORK_PAD_SIZE * 0.68, WORK_PAD_SLOT_WIDTH, WORK_PAD_SLOT_DEPTH)
        .translate(
            (
                0.0,
                0.0,
                WORK_PAD_THICKNESS - WORK_PAD_SLOT_DEPTH / 2.0,
            )
        )
    )
    cross_slot_y = (
        cq.Workplane("XY")
        .box(WORK_PAD_SLOT_WIDTH, WORK_PAD_SIZE * 0.68, WORK_PAD_SLOT_DEPTH)
        .translate(
            (
                0.0,
                0.0,
                WORK_PAD_THICKNESS - WORK_PAD_SLOT_DEPTH / 2.0,
            )
        )
    )

    return pad.cut(cross_slot_x).cut(cross_slot_y)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="indexing_platter_module")

    model.material("base_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("platter_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("work_pad_black", rgba=(0.16, 0.16, 0.17, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "indexer_base"),
        material="base_charcoal",
        name="base_housing",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT + BEARING_COLLAR_HEIGHT)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, (BASE_HEIGHT + BEARING_COLLAR_HEIGHT) / 2.0)),
    )

    platter = model.part("platter")
    platter.visual(
        mesh_from_cadquery(_build_platter_shape(), "rotary_platter"),
        material="platter_aluminum",
        name="platter_disk",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=PLATTER_RADIUS, length=PLATTER_THICKNESS),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, PLATTER_THICKNESS / 2.0)),
    )

    work_pad = model.part("work_pad")
    work_pad.visual(
        mesh_from_cadquery(_build_work_pad_shape(), "square_work_pad"),
        material="work_pad_black",
        name="pad_surface",
    )
    work_pad.inertial = Inertial.from_geometry(
        Box((WORK_PAD_SIZE, WORK_PAD_SIZE, WORK_PAD_THICKNESS)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, WORK_PAD_THICKNESS / 2.0)),
    )

    model.articulation(
        "base_to_platter",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + BEARING_COLLAR_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-pi,
            upper=pi,
            effort=18.0,
            velocity=2.4,
        ),
    )
    model.articulation(
        "platter_to_work_pad",
        ArticulationType.FIXED,
        parent=platter,
        child=work_pad,
        origin=Origin(xyz=(0.0, 0.0, PLATTER_THICKNESS)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    platter = object_model.get_part("platter")
    work_pad = object_model.get_part("work_pad")
    platter_joint = object_model.get_articulation("base_to_platter")
    pad_mount = object_model.get_articulation("platter_to_work_pad")

    ctx.check("base part exists", base is not None)
    ctx.check("platter part exists", platter is not None)
    ctx.check("work pad part exists", work_pad is not None)

    ctx.check(
        "platter joint is vertical revolute",
        platter_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(platter_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={platter_joint.articulation_type}, axis={platter_joint.axis}",
    )
    ctx.check(
        "work pad is fixed to platter",
        pad_mount.articulation_type == ArticulationType.FIXED,
        details=f"type={pad_mount.articulation_type}",
    )

    with ctx.pose({platter_joint: 0.0}):
        ctx.expect_gap(
            platter,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="platter seats on top of base collar",
        )
        ctx.expect_gap(
            work_pad,
            platter,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="work pad sits directly on platter",
        )
        ctx.expect_within(
            work_pad,
            platter,
            axes="xy",
            margin=0.0,
            name="work pad stays within platter footprint",
        )

        pad_rest_aabb = ctx.part_world_aabb(work_pad)
        rest_origin = ctx.part_world_position(work_pad)

    with ctx.pose({platter_joint: pi / 4.0}):
        pad_rotated_aabb = ctx.part_world_aabb(work_pad)
        rotated_origin = ctx.part_world_position(work_pad)
        ctx.expect_gap(
            work_pad,
            platter,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="work pad remains seated when platter rotates",
        )

    rest_span_x = None
    rotated_span_x = None
    expected_rotated_span = WORK_PAD_SIZE * sqrt(2.0)
    if pad_rest_aabb is not None:
        rest_span_x = pad_rest_aabb[1][0] - pad_rest_aabb[0][0]
    if pad_rotated_aabb is not None:
        rotated_span_x = pad_rotated_aabb[1][0] - pad_rotated_aabb[0][0]

    ctx.check(
        "work pad footprint rotates with platter",
        rest_span_x is not None
        and rotated_span_x is not None
        and rotated_span_x > rest_span_x + 0.015
        and abs(rotated_span_x - expected_rotated_span) < 0.008,
        details=(
            f"rest_span_x={rest_span_x}, rotated_span_x={rotated_span_x}, "
            f"expected_about={expected_rotated_span}"
        ),
    )
    ctx.check(
        "work pad stays centered on rotary axis",
        rest_origin is not None
        and rotated_origin is not None
        and abs(rest_origin[0] - rotated_origin[0]) < 1e-6
        and abs(rest_origin[1] - rotated_origin[1]) < 1e-6,
        details=f"rest_origin={rest_origin}, rotated_origin={rotated_origin}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
