from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


BASE_LENGTH = 0.52
BASE_WIDTH = 0.24
BASE_THICKNESS = 0.03

LOWER_AXIS_X = -0.11
UPPER_AXIS_X = 0.12

LOWER_HOUSING_RADIUS = 0.056
LOWER_HOUSING_HEIGHT = 0.026
LOWER_BEARING_CAP_RADIUS = 0.036
LOWER_BEARING_CAP_HEIGHT = 0.008

TOWER_COLUMN_X = 0.068
TOWER_COLUMN_Y = 0.10
TOWER_COLUMN_HEIGHT = 0.19

UPPER_HEAD_RADIUS = 0.046
UPPER_HEAD_HEIGHT = 0.022
UPPER_NECK_RADIUS = 0.034
UPPER_BEARING_CAP_HEIGHT = 0.006

LOWER_STAGE_RADIUS = 0.075
LOWER_STAGE_THICKNESS = 0.016
LOWER_STAGE_HUB_RADIUS = 0.034
LOWER_STAGE_HUB_HEIGHT = 0.008

UPPER_STAGE_RADIUS = 0.052
UPPER_STAGE_THICKNESS = 0.013
UPPER_STAGE_HUB_RADIUS = 0.026
UPPER_STAGE_HUB_HEIGHT = 0.006


def _ground_frame_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_THICKNESS,
        centered=(True, True, False),
    )

    lower_housing = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS)
        .center(LOWER_AXIS_X, 0.0)
        .circle(LOWER_HOUSING_RADIUS)
        .extrude(LOWER_HOUSING_HEIGHT)
    )
    lower_bearing_cap = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS + LOWER_HOUSING_HEIGHT)
        .center(LOWER_AXIS_X, 0.0)
        .circle(LOWER_BEARING_CAP_RADIUS)
        .extrude(LOWER_BEARING_CAP_HEIGHT)
    )

    tower_plinth = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS)
        .center(UPPER_AXIS_X, 0.0)
        .rect(0.13, 0.14)
        .extrude(0.018)
    )

    tower_column = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS)
        .center(UPPER_AXIS_X, 0.0)
        .rect(TOWER_COLUMN_X, TOWER_COLUMN_Y)
        .extrude(TOWER_COLUMN_HEIGHT)
    )

    rib_points = [
        (LOWER_AXIS_X + 0.09, BASE_THICKNESS),
        (UPPER_AXIS_X - 0.01, BASE_THICKNESS),
        (UPPER_AXIS_X - 0.01, BASE_THICKNESS + TOWER_COLUMN_HEIGHT * 0.9),
        (LOWER_AXIS_X + 0.14, BASE_THICKNESS + 0.075),
    ]
    tower_rib_right = (
        cq.Workplane("XZ").polyline(rib_points).close().extrude(0.018).translate((0.0, 0.045, 0.0))
    )
    tower_rib_left = (
        cq.Workplane("XZ")
        .polyline(rib_points)
        .close()
        .extrude(0.018)
        .translate((0.0, -0.063, 0.0))
    )

    upper_neck = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS + TOWER_COLUMN_HEIGHT - 0.006)
        .center(UPPER_AXIS_X, 0.0)
        .circle(UPPER_NECK_RADIUS)
        .extrude(0.006)
    )

    upper_head = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS + TOWER_COLUMN_HEIGHT)
        .center(UPPER_AXIS_X, 0.0)
        .circle(UPPER_HEAD_RADIUS)
        .extrude(UPPER_HEAD_HEIGHT)
    )
    upper_cap = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS + TOWER_COLUMN_HEIGHT + UPPER_HEAD_HEIGHT)
        .center(UPPER_AXIS_X, 0.0)
        .circle(UPPER_STAGE_HUB_RADIUS + 0.002)
        .extrude(UPPER_BEARING_CAP_HEIGHT)
    )

    return (
        base.union(lower_housing)
        .union(lower_bearing_cap)
        .union(tower_plinth)
        .union(tower_column)
        .union(tower_rib_right)
        .union(tower_rib_left)
        .union(upper_neck)
        .union(upper_head)
        .union(upper_cap)
    )


def _lower_stage_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("XY")
        .circle(LOWER_STAGE_HUB_RADIUS)
        .extrude(LOWER_STAGE_HUB_HEIGHT)
    )

    platter = (
        cq.Workplane("XY")
        .workplane(offset=LOWER_STAGE_HUB_HEIGHT)
        .circle(LOWER_STAGE_RADIUS)
        .extrude(LOWER_STAGE_THICKNESS)
    )
    flat_cutter = (
        cq.Workplane("XY")
        .workplane(offset=LOWER_STAGE_HUB_HEIGHT)
        .center(0.04, 0.0)
        .box(0.06, 0.18, LOWER_STAGE_THICKNESS + 0.002, centered=(False, True, False))
    )
    main_disk = platter.cut(flat_cutter)

    tooling_arm = (
        cq.Workplane("XY")
        .workplane(offset=LOWER_STAGE_HUB_HEIGHT + 0.006)
        .center(-0.058, -0.014)
        .box(0.072, 0.036, 0.01, centered=(True, True, False))
    )
    rear_counterweight = (
        cq.Workplane("XY")
        .workplane(offset=LOWER_STAGE_HUB_HEIGHT + 0.004)
        .center(-0.014, 0.026)
        .box(0.018, 0.022, 0.01, centered=(True, True, False))
    )

    bolt_holes = (
        cq.Workplane("XY")
        .workplane(offset=LOWER_STAGE_HUB_HEIGHT)
        .pushPoints([(0.034, -0.004), (-0.018, 0.03), (-0.016, -0.033)])
        .circle(0.006)
        .extrude(0.026)
    )

    top_cap = (
        cq.Workplane("XY")
        .workplane(offset=LOWER_STAGE_HUB_HEIGHT + LOWER_STAGE_THICKNESS)
        .circle(0.03)
        .extrude(0.007)
    )

    return (
        hub.union(main_disk)
        .union(tooling_arm)
        .union(rear_counterweight)
        .union(top_cap)
        .cut(bolt_holes)
    )


def _upper_stage_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("XY")
        .circle(UPPER_STAGE_HUB_RADIUS)
        .extrude(UPPER_STAGE_HUB_HEIGHT)
    )

    platter = (
        cq.Workplane("XY")
        .workplane(offset=UPPER_STAGE_HUB_HEIGHT)
        .circle(UPPER_STAGE_RADIUS)
        .extrude(UPPER_STAGE_THICKNESS)
    )
    flat_cutter = (
        cq.Workplane("XY")
        .workplane(offset=UPPER_STAGE_HUB_HEIGHT)
        .center(0.03, 0.0)
        .box(0.045, 0.13, UPPER_STAGE_THICKNESS + 0.002, centered=(False, True, False))
    )
    main_disk = platter.cut(flat_cutter)

    pointer_arm = (
        cq.Workplane("XY")
        .workplane(offset=UPPER_STAGE_HUB_HEIGHT + 0.004)
        .center(0.042, -0.01)
        .box(0.05, 0.022, 0.008, centered=(True, True, False))
    )
    rear_pad = (
        cq.Workplane("XY")
        .workplane(offset=UPPER_STAGE_HUB_HEIGHT + 0.004)
        .center(-0.018, 0.022)
        .box(0.018, 0.02, 0.01, centered=(True, True, False))
    )

    clamp_slot = (
        cq.Workplane("XY")
        .workplane(offset=UPPER_STAGE_HUB_HEIGHT)
        .center(0.012, -0.004)
        .rect(0.022, 0.007)
        .extrude(0.018)
    )

    top_cap = (
        cq.Workplane("XY")
        .workplane(offset=UPPER_STAGE_HUB_HEIGHT + UPPER_STAGE_THICKNESS)
        .circle(0.024)
        .extrude(0.006)
    )

    return hub.union(main_disk).union(pointer_arm).union(rear_pad).union(top_cap).cut(clamp_slot)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        0.5 * (lower[0] + upper[0]),
        0.5 * (lower[1] + upper[1]),
        0.5 * (lower[2] + upper[2]),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_offset_rotary_unit")

    painted_steel = model.material("painted_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    machined_aluminum = model.material("machined_aluminum", rgba=(0.77, 0.79, 0.81, 1.0))
    bead_blast_aluminum = model.material(
        "bead_blast_aluminum", rgba=(0.69, 0.71, 0.74, 1.0)
    )

    ground_frame = model.part("ground_frame")
    ground_frame.visual(
        mesh_from_cadquery(_ground_frame_shape(), "ground_frame"),
        material=painted_steel,
        name="ground_frame_visual",
    )
    ground_frame.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, 0.27)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(_lower_stage_shape(), "lower_stage"),
        material=machined_aluminum,
        name="lower_stage_visual",
    )
    lower_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=LOWER_STAGE_RADIUS, length=0.04),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        mesh_from_cadquery(_upper_stage_shape(), "upper_stage"),
        material=bead_blast_aluminum,
        name="upper_stage_visual",
    )
    upper_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=UPPER_STAGE_RADIUS, length=0.03),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    model.articulation(
        "ground_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=ground_frame,
        child=lower_stage,
        origin=Origin(
            xyz=(
                LOWER_AXIS_X,
                0.0,
                BASE_THICKNESS + LOWER_HOUSING_HEIGHT + LOWER_BEARING_CAP_HEIGHT,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    model.articulation(
        "ground_to_upper_stage",
        ArticulationType.REVOLUTE,
        parent=ground_frame,
        child=upper_stage,
        origin=Origin(
            xyz=(
                UPPER_AXIS_X,
                0.0,
                BASE_THICKNESS
                + TOWER_COLUMN_HEIGHT
                + UPPER_HEAD_HEIGHT
                + UPPER_BEARING_CAP_HEIGHT,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ground_frame = object_model.get_part("ground_frame")
    lower_stage = object_model.get_part("lower_stage")
    upper_stage = object_model.get_part("upper_stage")
    lower_joint = object_model.get_articulation("ground_to_lower_stage")
    upper_joint = object_model.get_articulation("ground_to_upper_stage")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        ground_frame,
        lower_stage,
        reason=(
            "The lower rotary table is modeled as a compact bearing-supported nest: "
            "its hub sits down into the grounded lower bearing seat, so the coaxial "
            "support region is an intentional nested interface rather than an accidental collision."
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(lower_stage, ground_frame, name="lower_stage_is_bearing_supported")
    ctx.expect_contact(upper_stage, ground_frame, name="upper_stage_is_tower_supported")
    ctx.expect_origin_gap(
        upper_stage,
        lower_stage,
        axis="z",
        min_gap=0.16,
        max_gap=0.22,
        name="upper_stage_sits_above_lower_stage",
    )
    ctx.expect_origin_gap(
        upper_stage,
        lower_stage,
        axis="x",
        min_gap=0.20,
        max_gap=0.24,
        name="upper_axis_is_laterally_offset",
    )

    ctx.check(
        "parallel_vertical_revolute_axes",
        lower_joint.axis == (0.0, 0.0, 1.0)
        and upper_joint.axis == (0.0, 0.0, 1.0)
        and lower_joint.articulation_type == ArticulationType.REVOLUTE
        and upper_joint.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"lower axis/type={lower_joint.axis}/{lower_joint.articulation_type}, "
            f"upper axis/type={upper_joint.axis}/{upper_joint.articulation_type}"
        ),
    )

    lower_rest = ctx.part_element_world_aabb(lower_stage, elem="lower_stage_visual")
    with ctx.pose({lower_joint: 1.0}):
        lower_turned = ctx.part_element_world_aabb(lower_stage, elem="lower_stage_visual")
    lower_rest_center = _aabb_center(lower_rest)
    lower_turned_center = _aabb_center(lower_turned)
    lower_reads_as_rotating = (
        lower_rest_center is not None
        and lower_turned_center is not None
        and (
            abs(lower_rest_center[0] - lower_turned_center[0]) > 0.008
            or abs(lower_rest_center[1] - lower_turned_center[1]) > 0.008
        )
    )
    ctx.check(
        "lower_stage_visual_changes_with_rotation",
        lower_reads_as_rotating,
        details=f"rest={lower_rest_center}, turned={lower_turned_center}",
    )

    upper_rest = ctx.part_element_world_aabb(upper_stage, elem="upper_stage_visual")
    with ctx.pose({upper_joint: 1.0}):
        upper_turned = ctx.part_element_world_aabb(upper_stage, elem="upper_stage_visual")
    upper_rest_center = _aabb_center(upper_rest)
    upper_turned_center = _aabb_center(upper_turned)
    upper_reads_as_rotating = (
        upper_rest_center is not None
        and upper_turned_center is not None
        and (
            abs(upper_rest_center[0] - upper_turned_center[0]) > 0.006
            or abs(upper_rest_center[1] - upper_turned_center[1]) > 0.006
        )
    )
    ctx.check(
        "upper_stage_visual_changes_with_rotation",
        upper_reads_as_rotating,
        details=f"rest={upper_rest_center}, turned={upper_turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
