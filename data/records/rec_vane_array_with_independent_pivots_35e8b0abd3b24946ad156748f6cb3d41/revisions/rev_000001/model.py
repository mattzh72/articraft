from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

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


OUTER_WIDTH = 0.62
OUTER_HEIGHT = 0.46
FRAME_DEPTH = 0.05
STILE_WIDTH = 0.055
RAIL_HEIGHT = 0.045
FLANGE_MARGIN = 0.028
FLANGE_THICKNESS = 0.005

INNER_WIDTH = OUTER_WIDTH - 2.0 * STILE_WIDTH
INNER_HEIGHT = OUTER_HEIGHT - 2.0 * RAIL_HEIGHT

VANE_COUNT = 5
VANE_CHORD = 0.048
VANE_THICKNESS = 0.009
SUPPORT_PAD_LEN = 0.018
SUPPORT_PAD_DEPTH = 0.03
SUPPORT_PAD_HEIGHT = 0.028
BOSS_RADIUS = 0.008
BOSS_LENGTH = 0.014
COLLAR_LENGTH = 0.004
COLLAR_RADIUS = 0.010
VANE_HALF_LENGTH = (
    INNER_WIDTH / 2.0 - SUPPORT_PAD_LEN - BOSS_LENGTH - COLLAR_LENGTH
)
PAD_CENTER_OFFSET = INNER_WIDTH / 2.0 - SUPPORT_PAD_LEN / 2.0
COLLAR_CENTER_OFFSET = VANE_HALF_LENGTH + COLLAR_LENGTH / 2.0
BOSS_CENTER_OFFSET = VANE_HALF_LENGTH + COLLAR_LENGTH + BOSS_LENGTH / 2.0
VANE_ZS = tuple(-0.14 + i * 0.07 for i in range(VANE_COUNT))
VANE_LIMIT = 0.55


def _make_vane_blade() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .moveTo(-VANE_CHORD / 2.0, 0.0)
        .threePointArc((0.0, VANE_THICKNESS / 2.0), (VANE_CHORD / 2.0, 0.0))
        .threePointArc((0.0, -VANE_THICKNESS / 2.0), (-VANE_CHORD / 2.0, 0.0))
        .close()
        .extrude(VANE_HALF_LENGTH, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_louver_module")

    frame_color = model.material("frame_powder_coat", rgba=(0.33, 0.36, 0.39, 1.0))
    vane_color = model.material("vane_aluminum", rgba=(0.74, 0.76, 0.78, 1.0))
    cylinder_rpy = (0.0, pi / 2.0, 0.0)

    frame = model.part("frame")
    frame.visual(
        Box((STILE_WIDTH, FRAME_DEPTH, OUTER_HEIGHT)),
        origin=Origin(xyz=(-OUTER_WIDTH / 2.0 + STILE_WIDTH / 2.0, 0.0, 0.0)),
        material=frame_color,
        name="left_stile",
    )
    frame.visual(
        Box((STILE_WIDTH, FRAME_DEPTH, OUTER_HEIGHT)),
        origin=Origin(xyz=(OUTER_WIDTH / 2.0 - STILE_WIDTH / 2.0, 0.0, 0.0)),
        material=frame_color,
        name="right_stile",
    )
    frame.visual(
        Box((OUTER_WIDTH, FRAME_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, OUTER_HEIGHT / 2.0 - RAIL_HEIGHT / 2.0)),
        material=frame_color,
        name="top_rail",
    )
    frame.visual(
        Box((OUTER_WIDTH, FRAME_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -OUTER_HEIGHT / 2.0 + RAIL_HEIGHT / 2.0)),
        material=frame_color,
        name="bottom_rail",
    )
    frame.visual(
        Box((STILE_WIDTH, FLANGE_THICKNESS, OUTER_HEIGHT + 2.0 * FLANGE_MARGIN)),
        origin=Origin(
            xyz=(
                -OUTER_WIDTH / 2.0 + STILE_WIDTH / 2.0,
                FRAME_DEPTH / 2.0 - FLANGE_THICKNESS / 2.0,
                0.0,
            )
        ),
        material=frame_color,
        name="left_flange",
    )
    frame.visual(
        Box((STILE_WIDTH, FLANGE_THICKNESS, OUTER_HEIGHT + 2.0 * FLANGE_MARGIN)),
        origin=Origin(
            xyz=(
                OUTER_WIDTH / 2.0 - STILE_WIDTH / 2.0,
                FRAME_DEPTH / 2.0 - FLANGE_THICKNESS / 2.0,
                0.0,
            )
        ),
        material=frame_color,
        name="right_flange",
    )
    frame.visual(
        Box((OUTER_WIDTH + 2.0 * FLANGE_MARGIN, FLANGE_THICKNESS, RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                FRAME_DEPTH / 2.0 - FLANGE_THICKNESS / 2.0,
                OUTER_HEIGHT / 2.0 - RAIL_HEIGHT / 2.0,
            )
        ),
        material=frame_color,
        name="top_flange",
    )
    frame.visual(
        Box((OUTER_WIDTH + 2.0 * FLANGE_MARGIN, FLANGE_THICKNESS, RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                FRAME_DEPTH / 2.0 - FLANGE_THICKNESS / 2.0,
                -OUTER_HEIGHT / 2.0 + RAIL_HEIGHT / 2.0,
            )
        ),
        material=frame_color,
        name="bottom_flange",
    )

    blade_shape = _make_vane_blade()
    for index, z_pos in enumerate(VANE_ZS, start=1):
        frame.visual(
            Box((SUPPORT_PAD_LEN, SUPPORT_PAD_DEPTH, SUPPORT_PAD_HEIGHT)),
            origin=Origin(xyz=(-PAD_CENTER_OFFSET, 0.0, z_pos)),
            material=frame_color,
            name=f"left_pad_{index}",
        )
        frame.visual(
            Box((SUPPORT_PAD_LEN, SUPPORT_PAD_DEPTH, SUPPORT_PAD_HEIGHT)),
            origin=Origin(xyz=(PAD_CENTER_OFFSET, 0.0, z_pos)),
            material=frame_color,
            name=f"right_pad_{index}",
        )

        vane = model.part(f"vane_{index}")
        vane.visual(
            mesh_from_cadquery(blade_shape, f"vane_blade_{index}"),
            material=vane_color,
            name="blade",
        )
        vane.visual(
            Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
            origin=Origin(xyz=(-COLLAR_CENTER_OFFSET, 0.0, 0.0), rpy=cylinder_rpy),
            material=vane_color,
            name="left_collar",
        )
        vane.visual(
            Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
            origin=Origin(xyz=(COLLAR_CENTER_OFFSET, 0.0, 0.0), rpy=cylinder_rpy),
            material=vane_color,
            name="right_collar",
        )
        vane.visual(
            Cylinder(radius=BOSS_RADIUS, length=BOSS_LENGTH),
            origin=Origin(xyz=(-BOSS_CENTER_OFFSET, 0.0, 0.0), rpy=cylinder_rpy),
            material=vane_color,
            name="left_boss",
        )
        vane.visual(
            Cylinder(radius=BOSS_RADIUS, length=BOSS_LENGTH),
            origin=Origin(xyz=(BOSS_CENTER_OFFSET, 0.0, 0.0), rpy=cylinder_rpy),
            material=vane_color,
            name="right_boss",
        )
        model.articulation(
            f"frame_to_vane_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=1.5,
                lower=-VANE_LIMIT,
                upper=VANE_LIMIT,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    vane_parts = [
        object_model.get_part(f"vane_{index}") for index in range(1, VANE_COUNT + 1)
    ]
    vane_joints = [
        object_model.get_articulation(f"frame_to_vane_{index}")
        for index in range(1, VANE_COUNT + 1)
    ]

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

    for index, (vane, joint) in enumerate(zip(vane_parts, vane_joints), start=1):
        ctx.check(
            f"vane_{index}_hinge_axis_parallel",
            tuple(joint.axis) == (1.0, 0.0, 0.0),
            details=f"axis={joint.axis}",
        )
        ctx.check(
            f"vane_{index}_has_symmetric_motion_limits",
            joint.motion_limits is not None
            and joint.motion_limits.lower == -VANE_LIMIT
            and joint.motion_limits.upper == VANE_LIMIT,
            details=f"limits={joint.motion_limits}",
        )
        ctx.expect_contact(
            vane,
            frame,
            name=f"vane_{index}_supported_by_frame",
        )
        ctx.expect_within(
            vane,
            frame,
            axes="xz",
            margin=0.0,
            name=f"vane_{index}_contained_by_frame_outline",
        )

    for lower, upper, index in zip(vane_parts, vane_parts[1:], range(1, VANE_COUNT)):
        ctx.expect_origin_gap(
            upper,
            lower,
            axis="z",
            min_gap=0.06,
            max_gap=0.08,
            name=f"vane_axis_spacing_{index}_to_{index + 1}",
        )

    open_pose = {joint: VANE_LIMIT for joint in vane_joints}
    closed_pose = {joint: -VANE_LIMIT for joint in vane_joints}
    with ctx.pose(open_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_all_vanes_open")
        ctx.expect_contact(
            vane_parts[2],
            frame,
            name="center_vane_remains_supported_open",
        )
    with ctx.pose(closed_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_all_vanes_closed")
        ctx.expect_contact(
            vane_parts[2],
            frame,
            name="center_vane_remains_supported_closed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
