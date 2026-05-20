from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_pitch_roll_unit")

    pedestal_mat = model.material("pedestal_paint", rgba=(0.18, 0.19, 0.21, 1.0))
    outer_mat = model.material("outer_frame_finish", rgba=(0.52, 0.55, 0.58, 1.0))
    inner_mat = model.material("inner_cradle_finish", rgba=(0.28, 0.3, 0.33, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.16, 0.14, 0.02)),
        material=pedestal_mat,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="pedestal_base",
    )
    pedestal.visual(
        Box((0.07, 0.06, 0.11)),
        material=pedestal_mat,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        name="pedestal_column",
    )
    pedestal.visual(
        Box((0.082, 0.05, 0.02)),
        material=pedestal_mat,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        name="pedestal_neck",
    )
    pedestal.visual(
        Box((0.018, 0.044, 0.05)),
        material=pedestal_mat,
        origin=Origin(xyz=(-0.046, 0.0, 0.175)),
        name="left_cheek",
    )
    pedestal.visual(
        Box((0.018, 0.044, 0.05)),
        material=pedestal_mat,
        origin=Origin(xyz=(0.046, 0.0, 0.175)),
        name="right_cheek",
    )

    roll_frame = model.part("roll_frame")
    roll_frame.visual(
        Cylinder(radius=0.006, length=0.074),
        material=outer_mat,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        name="roll_axle",
    )
    roll_frame.visual(
        Box((0.014, 0.074, 0.016)),
        material=outer_mat,
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
        name="rear_bridge",
    )
    roll_frame.visual(
        Box((0.012, 0.012, 0.062)),
        material=outer_mat,
        origin=Origin(xyz=(0.014, -0.031, 0.031)),
        name="left_side_plate",
    )
    roll_frame.visual(
        Box((0.012, 0.012, 0.062)),
        material=outer_mat,
        origin=Origin(xyz=(0.014, 0.031, 0.031)),
        name="right_side_plate",
    )
    roll_frame.visual(
        Box((0.012, 0.074, 0.012)),
        material=outer_mat,
        origin=Origin(xyz=(0.014, 0.0, 0.068)),
        name="top_bridge",
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(radius=0.0055, length=0.05),
        material=inner_mat,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        name="pitch_trunnion",
    )
    pitch_cradle.visual(
        Box((0.01, 0.012, 0.01)),
        material=inner_mat,
        origin=Origin(xyz=(0.004, 0.0, 0.001)),
        name="rear_mount",
    )
    pitch_cradle.visual(
        Box((0.016, 0.016, 0.014)),
        material=inner_mat,
        origin=Origin(xyz=(0.01, 0.0, 0.004)),
        name="cradle_body",
    )
    pitch_cradle.visual(
        Box((0.022, 0.022, 0.006)),
        material=inner_mat,
        origin=Origin(xyz=(0.012, 0.0, 0.013)),
        name="top_plate",
    )
    pitch_cradle.visual(
        Box((0.006, 0.014, 0.01)),
        material=inner_mat,
        origin=Origin(xyz=(0.019, 0.0, 0.001)),
        name="front_lip",
    )

    model.articulation(
        "pedestal_to_roll",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=roll_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-0.9, upper=0.9),
    )

    model.articulation(
        "roll_to_pitch",
        ArticulationType.REVOLUTE,
        parent=roll_frame,
        child=pitch_cradle,
        origin=Origin(xyz=(0.014, 0.0, 0.024)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.65, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    roll_frame = object_model.get_part("roll_frame")
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll_joint = object_model.get_articulation("pedestal_to_roll")
    pitch_joint = object_model.get_articulation("roll_to_pitch")

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

    ctx.expect_contact(roll_frame, pedestal, name="roll_frame_supported_by_pedestal")
    ctx.expect_contact(pitch_cradle, roll_frame, name="pitch_cradle_supported_by_roll_frame")
    ctx.expect_origin_gap(
        roll_frame,
        pedestal,
        axis="z",
        min_gap=0.16,
        max_gap=0.19,
        name="head_sits_above_short_pedestal",
    )
    ctx.expect_within(
        pitch_cradle,
        roll_frame,
        axes="yz",
        margin=0.0,
        name="inner_cradle_nests_inside_outer_member",
    )

    ctx.check(
        "roll_axis_is_lateral",
        tuple(round(v, 6) for v in roll_joint.axis) == (1.0, 0.0, 0.0),
        f"expected roll axis (1,0,0), got {roll_joint.axis}",
    )
    ctx.check(
        "pitch_axis_is_perpendicular",
        tuple(round(v, 6) for v in pitch_joint.axis) == (0.0, -1.0, 0.0),
        f"expected pitch axis (0,-1,0), got {pitch_joint.axis}",
    )

    roll_aabb = ctx.part_world_aabb(roll_frame)
    cradle_aabb = ctx.part_world_aabb(pitch_cradle)
    if roll_aabb is not None and cradle_aabb is not None:
        roll_size = tuple(roll_aabb[1][i] - roll_aabb[0][i] for i in range(3))
        cradle_size = tuple(cradle_aabb[1][i] - cradle_aabb[0][i] for i in range(3))
        ctx.check(
            "outer_member_visibly_larger_than_inner_cradle",
            roll_size[1] > cradle_size[1] and roll_size[2] > cradle_size[2],
            f"outer size={roll_size}, inner size={cradle_size}",
        )
    else:
        ctx.fail("outer_member_visibly_larger_than_inner_cradle", "missing part AABB for size comparison")

    with ctx.pose({pitch_joint: pitch_joint.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_max_pitch")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
