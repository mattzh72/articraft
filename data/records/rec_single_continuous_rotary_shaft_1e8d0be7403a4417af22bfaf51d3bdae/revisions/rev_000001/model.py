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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.18
BASE_WIDTH = 0.075
BASE_THICKNESS = 0.012
SUPPORT_CENTER_SPAN = 0.104
SUPPORT_THICKNESS = 0.018
SUPPORT_WIDTH = 0.056
AXIS_HEIGHT = 0.048

SHAFT_LENGTH = 0.154
SHAFT_RADIUS = 0.0075
JOURNAL_RADIUS = 0.0095
BEARING_WIDTH = 0.024

COLLAR_RADIUS = 0.014
COLLAR_WIDTH = 0.014
PLATE_SIZE = 0.042
PLATE_THICKNESS = 0.006
PLATE_CENTER_X = SUPPORT_CENTER_SPAN / 2.0 + 0.023
PLATE_HUB_RADIUS = 0.012
PLATE_HUB_LENGTH = 0.014

PAD_THICKNESS = 0.004
PAD_INTERLOCK = 0.0005
CHEEK_DEPTH = 0.012
SUPPORT_TOP_CAP_THICKNESS = 0.014
BEARING_CLEARANCE = 0.0008
BEARING_OUTER_RADIUS = JOURNAL_RADIUS + PAD_THICKNESS + 0.0012
LOWER_BLOCK_TOP_Z = AXIS_HEIGHT - JOURNAL_RADIUS - PAD_THICKNESS + PAD_INTERLOCK
TOP_CAP_BOTTOM_Z = AXIS_HEIGHT + JOURNAL_RADIUS + PAD_THICKNESS - PAD_INTERLOCK
SUPPORT_TOP_Z = TOP_CAP_BOTTOM_Z + SUPPORT_TOP_CAP_THICKNESS
SHAFT_ALONG_X = Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _bearing_half_shell(center_x: float, *, upper: bool) -> cq.Workplane:
    ring = (
        cq.Workplane("YZ")
        .circle(BEARING_OUTER_RADIUS)
        .circle(JOURNAL_RADIUS + BEARING_CLEARANCE)
        .extrude(BEARING_WIDTH)
        .translate((center_x - BEARING_WIDTH / 2.0, 0.0, AXIS_HEIGHT))
    )
    clip = (
        cq.Workplane("XY")
        .box(
            BEARING_WIDTH + 0.004,
            2.0 * (BEARING_OUTER_RADIUS + 0.004),
            BEARING_OUTER_RADIUS + 0.004,
        )
        .translate(
            (
                center_x,
                0.0,
                AXIS_HEIGHT
                + (BEARING_OUTER_RADIUS + 0.004) / 2.0 * (1.0 if upper else -1.0),
            )
        )
    )
    return ring.intersect(clip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_shaft_fixture")

    model.material("frame_paint", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("dark_steel", rgba=(0.56, 0.58, 0.61, 1.0))

    frame = model.part("frame")
    left_x = -SUPPORT_CENTER_SPAN / 2.0
    right_x = SUPPORT_CENTER_SPAN / 2.0

    frame.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="frame_paint",
        name="base_rail",
    )

    for side_name, center_x in (("left", left_x), ("right", right_x)):
        frame.visual(
            Box((SUPPORT_THICKNESS, SUPPORT_WIDTH, LOWER_BLOCK_TOP_Z - BASE_THICKNESS)),
            origin=Origin(
                xyz=(
                    center_x,
                    0.0,
                    BASE_THICKNESS + (LOWER_BLOCK_TOP_Z - BASE_THICKNESS) / 2.0,
                )
            ),
            material="frame_paint",
            name=f"{side_name}_lower_block",
        )
        frame.visual(
            Box((SUPPORT_THICKNESS, SUPPORT_WIDTH, SUPPORT_TOP_CAP_THICKNESS)),
            origin=Origin(
                xyz=(center_x, 0.0, TOP_CAP_BOTTOM_Z + SUPPORT_TOP_CAP_THICKNESS / 2.0)
            ),
            material="frame_paint",
            name=f"{side_name}_top_cap",
        )
        for cheek_name, cheek_y in (
            ("front", (SUPPORT_WIDTH - CHEEK_DEPTH) / 2.0),
            ("rear", -(SUPPORT_WIDTH - CHEEK_DEPTH) / 2.0),
        ):
            frame.visual(
                Box((SUPPORT_THICKNESS, CHEEK_DEPTH, SUPPORT_TOP_Z - BASE_THICKNESS)),
                origin=Origin(
                    xyz=(center_x, cheek_y, BASE_THICKNESS + (SUPPORT_TOP_Z - BASE_THICKNESS) / 2.0)
                ),
                material="frame_paint",
                name=f"{side_name}_{cheek_name}_cheek",
            )

        frame.visual(
            mesh_from_cadquery(_bearing_half_shell(center_x, upper=True), f"{side_name}_bearing_upper"),
            material="dark_steel",
            name=f"{side_name}_bearing_top",
        )
        frame.visual(
            mesh_from_cadquery(_bearing_half_shell(center_x, upper=False), f"{side_name}_bearing_lower"),
            material="dark_steel",
            name=f"{side_name}_bearing_bottom",
        )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(rpy=SHAFT_ALONG_X.rpy),
        material="steel",
        name="main_shaft",
    )
    shaft.visual(
        Cylinder(radius=JOURNAL_RADIUS, length=BEARING_WIDTH),
        origin=Origin(xyz=(left_x, 0.0, 0.0), rpy=SHAFT_ALONG_X.rpy),
        material="steel",
        name="left_journal",
    )
    shaft.visual(
        Cylinder(radius=JOURNAL_RADIUS, length=BEARING_WIDTH),
        origin=Origin(xyz=(right_x, 0.0, 0.0), rpy=SHAFT_ALONG_X.rpy),
        material="steel",
        name="right_journal",
    )
    shaft.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_WIDTH),
        origin=Origin(rpy=SHAFT_ALONG_X.rpy),
        material="dark_steel",
        name="midspan_collar",
    )
    shaft.visual(
        Cylinder(radius=PLATE_HUB_RADIUS, length=PLATE_HUB_LENGTH),
        origin=Origin(xyz=(PLATE_CENTER_X - PLATE_HUB_LENGTH / 2.0, 0.0, 0.0), rpy=SHAFT_ALONG_X.rpy),
        material="dark_steel",
        name="drive_hub",
    )
    shaft.visual(
        Box((PLATE_THICKNESS, PLATE_SIZE, PLATE_SIZE)),
        origin=Origin(xyz=(PLATE_CENTER_X, 0.0, 0.0)),
        material="steel",
        name="drive_plate",
    )

    model.articulation(
        "frame_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, AXIS_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=15.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    shaft = object_model.get_part("shaft")
    spindle = object_model.get_articulation("frame_to_shaft")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "shaft_joint_is_continuous",
        spindle.joint_type == ArticulationType.CONTINUOUS,
        details=f"expected continuous joint, got {spindle.joint_type}",
    )
    ctx.check(
        "shaft_axis_is_x",
        tuple(spindle.axis) == (1.0, 0.0, 0.0),
        details=f"expected x-axis rotation, got {spindle.axis}",
    )
    ctx.check(
        "continuous_joint_has_no_position_limits",
        spindle.motion_limits is not None
        and spindle.motion_limits.lower is None
        and spindle.motion_limits.upper is None,
        details="continuous joint should not clamp lower or upper angle limits",
    )

    ctx.expect_contact(shaft, frame, name="shaft_journals_are_supported")

    with ctx.pose({spindle: math.pi / 4.0}):
        ctx.expect_contact(shaft, frame, name="shaft_remains_supported_when_rotated")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
