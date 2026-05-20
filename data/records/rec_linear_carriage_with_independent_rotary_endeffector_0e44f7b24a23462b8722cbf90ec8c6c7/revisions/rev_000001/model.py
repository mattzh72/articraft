from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


GUIDE_LENGTH = 0.68
GUIDE_WIDTH = 0.16
GUIDE_BASE_THICKNESS = 0.018
GUIDE_RAIL_LENGTH = 0.60
GUIDE_RAIL_WIDTH = 0.056
GUIDE_RAIL_HEIGHT = 0.042
GUIDE_END_BLOCK_LENGTH = 0.036
GUIDE_HOLE_DIAMETER = 0.014

SLIDE_AXIS_Z = GUIDE_BASE_THICKNESS + GUIDE_RAIL_HEIGHT
SLIDE_TRAVEL = 0.18

CARRIAGE_LENGTH = 0.16
CARRIAGE_WIDTH = 0.136
CARRIAGE_SADDLE_HEIGHT = 0.052
CARRIAGE_AXIS_Z = 0.086

HEAD_JOURNAL_RADIUS = 0.029
HEAD_JOURNAL_LENGTH = 0.08
HEAD_BODY_RADIUS = 0.06
HEAD_BODY_LENGTH = 0.086
HEAD_FACEPLATE_RADIUS = 0.066
HEAD_FACEPLATE_THICKNESS = 0.014


def _guide_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_BASE_THICKNESS)
        .translate((0.0, 0.0, GUIDE_BASE_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.004)
    )

    hole_points = [
        (-0.24, -0.05),
        (-0.24, 0.05),
        (0.24, -0.05),
        (0.24, 0.05),
    ]
    base = (
        base.faces(">Z")
        .workplane()
        .pushPoints(hole_points)
        .hole(GUIDE_HOLE_DIAMETER)
    )

    return base


def _carriage_shape() -> cq.Workplane:
    upper_body = (
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_SADDLE_HEIGHT)
        .translate((-0.08, 0.0, 0.036))
    )

    guide_foot = (
        cq.Workplane("XY")
        .box(0.118, 0.052, 0.01)
        .translate((-0.06, 0.0, 0.005))
    )

    support_block = (
        cq.Workplane("XY")
        .box(0.078, 0.118, 0.088)
        .translate((-0.039, 0.0, 0.054))
    )

    bearing_boss = (
        cq.Workplane("YZ")
        .circle(0.05)
        .extrude(0.022)
        .translate((-0.022, 0.0, CARRIAGE_AXIS_Z))
    )

    lower_web = (
        cq.Workplane("XY")
        .box(0.06, 0.084, 0.036)
        .translate((-0.04, 0.0, 0.028))
    )

    carriage = upper_body.union(guide_foot).union(support_block).union(bearing_boss).union(lower_web)

    spindle_bore = (
        cq.Workplane("YZ")
        .circle(HEAD_JOURNAL_RADIUS + 0.0015)
        .extrude(0.14)
        .translate((-0.112, 0.0, CARRIAGE_AXIS_Z))
    )
    underside_relief = (
        cq.Workplane("XY")
        .box(0.078, 0.09, 0.014)
        .translate((-0.028, 0.0, 0.007))
    )

    return carriage.cut(spindle_bore).cut(underside_relief)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machine_slide_rotary_head")

    model.material("guide_steel", rgba=(0.34, 0.37, 0.40, 1.0))
    model.material("carriage_gray", rgba=(0.64, 0.66, 0.69, 1.0))
    model.material("machined_face", rgba=(0.79, 0.81, 0.84, 1.0))
    model.material("dark_service", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("zinc_fastener", rgba=(0.70, 0.72, 0.75, 1.0))

    guide = model.part("guide")
    guide.visual(
        mesh_from_cadquery(_guide_shape(), "guide_body"),
        material="guide_steel",
        name="guide_body",
    )
    guide.visual(
        Box((GUIDE_RAIL_LENGTH, GUIDE_RAIL_WIDTH, GUIDE_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, GUIDE_BASE_THICKNESS + (GUIDE_RAIL_HEIGHT / 2.0))
        ),
        material="machined_face",
        name="guide_rail",
    )
    guide.visual(
        Box((GUIDE_END_BLOCK_LENGTH, 0.092, 0.026)),
        origin=Origin(
            xyz=(
                -(GUIDE_RAIL_LENGTH / 2.0) + (GUIDE_END_BLOCK_LENGTH / 2.0),
                0.0,
                GUIDE_BASE_THICKNESS + 0.013,
            )
        ),
        material="guide_steel",
        name="guide_left_stop",
    )
    guide.visual(
        Box((GUIDE_END_BLOCK_LENGTH, 0.092, 0.026)),
        origin=Origin(
            xyz=(
                (GUIDE_RAIL_LENGTH / 2.0) - (GUIDE_END_BLOCK_LENGTH / 2.0),
                0.0,
                GUIDE_BASE_THICKNESS + 0.013,
            )
        ),
        material="guide_steel",
        name="guide_right_stop",
    )
    for x_pos in (-0.24, 0.24):
        for y_pos in (-0.05, 0.05):
            guide.visual(
                Cylinder(radius=0.011, length=0.003),
                origin=Origin(xyz=(x_pos, y_pos, GUIDE_BASE_THICKNESS + 0.0015)),
                material="zinc_fastener",
                name=f"mount_pad_{'l' if x_pos < 0 else 'r'}_{'n' if y_pos < 0 else 'p'}",
            )
    guide.inertial = Inertial.from_geometry(
        Box((GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_BASE_THICKNESS + GUIDE_RAIL_HEIGHT)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage_block"),
        material="carriage_gray",
        name="carriage_block",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, 0.12)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )

    head = model.part("rotary_head")
    head.visual(
        Cylinder(radius=HEAD_JOURNAL_RADIUS, length=HEAD_JOURNAL_LENGTH),
        origin=Origin(xyz=(-0.03, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_face",
        name="head_journal",
    )
    head.visual(
        Cylinder(radius=0.05, length=0.01),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_face",
        name="head_mount_flange",
    )
    head.visual(
        Cylinder(radius=HEAD_BODY_RADIUS, length=HEAD_BODY_LENGTH),
        origin=Origin(
            xyz=(0.01 + (HEAD_BODY_LENGTH / 2.0), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="machined_face",
        name="head_body",
    )
    head.visual(
        Cylinder(radius=HEAD_FACEPLATE_RADIUS, length=HEAD_FACEPLATE_THICKNESS),
        origin=Origin(
            xyz=(
                0.01 + HEAD_BODY_LENGTH + (HEAD_FACEPLATE_THICKNESS / 2.0),
                0.0,
                0.0,
            ),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="machined_face",
        name="head_faceplate",
    )
    head.visual(
        Box((0.04, 0.048, 0.034)),
        origin=Origin(xyz=(0.05, 0.0, 0.066)),
        material="dark_service",
        name="head_encoder",
    )
    head.inertial = Inertial.from_geometry(
        Cylinder(radius=HEAD_BODY_RADIUS, length=0.11),
        mass=6.5,
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "guide_slide",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, SLIDE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
            effort=350.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "carriage_head_rotation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.35,
            upper=1.35,
            effort=60.0,
            velocity=2.0,
        ),
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
    guide = object_model.get_part("guide")
    carriage = object_model.get_part("carriage")
    head = object_model.get_part("rotary_head")
    slide = object_model.get_articulation("guide_slide")
    rotary = object_model.get_articulation("carriage_head_rotation")
    encoder = head.get_visual("head_encoder")

    ctx.expect_contact(
        carriage,
        guide,
        name="carriage bears on the linear guide",
    )
    ctx.expect_overlap(
        carriage,
        guide,
        axes="xy",
        min_overlap=0.05,
        name="carriage footprint stays over the guide rail",
    )
    ctx.expect_gap(
        head,
        guide,
        axis="z",
        min_gap=0.018,
        name="rotary head clears the fixed guide vertically",
    )
    ctx.expect_contact(
        head,
        carriage,
        name="rotary head is supported by the carriage housing",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        extended_carriage = ctx.part_world_position(carriage)
    ctx.check(
        "prismatic joint advances carriage along the guide",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + (SLIDE_TRAVEL * 0.9),
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    rest_encoder_center = _aabb_center(ctx.part_element_world_aabb(head, elem=encoder))
    with ctx.pose({rotary: 1.2}):
        turned_encoder_center = _aabb_center(ctx.part_element_world_aabb(head, elem=encoder))
    ctx.check(
        "rotary head turns about its own axis",
        rest_encoder_center is not None
        and turned_encoder_center is not None
        and abs(turned_encoder_center[1] - rest_encoder_center[1]) > 0.04
        and abs(turned_encoder_center[2] - rest_encoder_center[2]) > 0.04,
        details=f"rest={rest_encoder_center}, turned={turned_encoder_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
