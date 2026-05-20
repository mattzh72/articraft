from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
)


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gantry_robot_axis")

    model.material("frame_aluminum", rgba=(0.79, 0.80, 0.82, 1.0))
    model.material("rider_dark", rgba=(0.19, 0.21, 0.24, 1.0))
    model.material("stage_silver", rgba=(0.90, 0.91, 0.92, 1.0))
    model.material("steel", rgba=(0.36, 0.38, 0.42, 1.0))

    bridge = model.part("bridge")
    _add_box(
        bridge,
        (0.24, 0.26, 0.06),
        (-0.54, 0.0, 0.03),
        "frame_aluminum",
        "left_foot",
    )
    _add_box(
        bridge,
        (0.24, 0.26, 0.06),
        (0.54, 0.0, 0.03),
        "frame_aluminum",
        "right_foot",
    )
    _add_box(
        bridge,
        (0.10, 0.18, 0.78),
        (-0.54, 0.0, 0.45),
        "frame_aluminum",
        "left_upright",
    )
    _add_box(
        bridge,
        (0.10, 0.18, 0.78),
        (0.54, 0.0, 0.45),
        "frame_aluminum",
        "right_upright",
    )
    _add_box(
        bridge,
        (1.20, 0.16, 0.12),
        (0.0, 0.0, 0.84),
        "frame_aluminum",
        "bridge_beam",
    )
    _add_box(
        bridge,
        (0.94, 0.02, 0.02),
        (0.0, 0.07, 0.88),
        "steel",
        "beam_way_strip",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((1.32, 0.26, 0.90)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
    )

    rider = model.part("rider")
    _add_box(
        rider,
        (0.18, 0.14, 0.13),
        (0.0, 0.07, -0.005),
        "rider_dark",
        "rider_body",
    )
    _add_box(
        rider,
        (0.20, 0.11, 0.04),
        (0.0, -0.035, 0.08),
        "rider_dark",
        "top_saddle",
    )
    _add_box(
        rider,
        (0.13, 0.03, 0.03),
        (0.0, 0.125, 0.03),
        "steel",
        "stage_crosshead",
    )
    _add_box(
        rider,
        (0.022, 0.03, 0.23),
        (-0.058, 0.125, -0.085),
        "steel",
        "left_stage_rail",
    )
    _add_box(
        rider,
        (0.022, 0.03, 0.23),
        (0.058, 0.125, -0.085),
        "steel",
        "right_stage_rail",
    )
    rider.inertial = Inertial.from_geometry(
        Box((0.20, 0.20, 0.24)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.04, -0.02)),
    )

    z_stage = model.part("z_stage")
    _add_box(
        z_stage,
        (0.088, 0.045, 0.24),
        (0.0, 0.0225, -0.12),
        "stage_silver",
        "carriage_plate",
    )
    _add_box(
        z_stage,
        (0.014, 0.03, 0.10),
        (-0.032, 0.015, -0.05),
        "steel",
        "left_runner",
    )
    _add_box(
        z_stage,
        (0.014, 0.03, 0.10),
        (0.032, 0.015, -0.05),
        "steel",
        "right_runner",
    )
    _add_box(
        z_stage,
        (0.10, 0.055, 0.06),
        (0.0, 0.0275, -0.27),
        "rider_dark",
        "tool_block",
    )
    _add_cylinder(
        z_stage,
        radius=0.02,
        length=0.04,
        xyz=(0.0, 0.075, -0.27),
        material="steel",
        name="tool_nose",
        rpy=(pi / 2.0, 0.0, 0.0),
    )
    z_stage.inertial = Inertial.from_geometry(
        Box((0.10, 0.08, 0.31)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.03, -0.155)),
    )

    model.articulation(
        "bridge_to_rider",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=rider,
        origin=Origin(xyz=(0.0, 0.08, 0.84)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=1.0,
            lower=-0.30,
            upper=0.30,
        ),
    )
    model.articulation(
        "rider_to_z_stage",
        ArticulationType.PRISMATIC,
        parent=rider,
        child=z_stage,
        origin=Origin(xyz=(0.0, 0.14, 0.015)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.6,
            lower=0.0,
            upper=0.26,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge = object_model.get_part("bridge")
    rider = object_model.get_part("rider")
    z_stage = object_model.get_part("z_stage")
    left_upright = bridge.get_visual("left_upright")
    right_upright = bridge.get_visual("right_upright")
    x_axis = object_model.get_articulation("bridge_to_rider")
    z_axis = object_model.get_articulation("rider_to_z_stage")

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

    ctx.expect_contact(rider, bridge, name="rider_is_supported_by_bridge")
    ctx.expect_contact(z_stage, rider, name="z_stage_is_supported_by_rider")
    ctx.expect_within(
        z_stage,
        rider,
        axes="x",
        margin=0.006,
        name="z_stage_stays_nested_between_rider_guides",
    )

    with ctx.pose({x_axis: x_axis.motion_limits.lower}):
        rider_left = ctx.part_world_position(rider)
        ctx.expect_gap(
            rider,
            bridge,
            axis="x",
            min_gap=0.025,
            negative_elem=left_upright,
            name="left_travel_clears_left_upright",
        )
    with ctx.pose({x_axis: x_axis.motion_limits.upper}):
        rider_right = ctx.part_world_position(rider)
        ctx.expect_gap(
            bridge,
            rider,
            axis="x",
            min_gap=0.025,
            positive_elem=right_upright,
            name="right_travel_clears_right_upright",
        )
    ctx.check(
        "rider_moves_across_beam_in_positive_x",
        rider_left is not None
        and rider_right is not None
        and rider_right[0] > rider_left[0] + 0.55,
        details=f"left={rider_left}, right={rider_right}",
    )

    with ctx.pose({z_axis: z_axis.motion_limits.lower}):
        stage_high = ctx.part_world_position(z_stage)
    with ctx.pose({z_axis: z_axis.motion_limits.upper}):
        stage_low = ctx.part_world_position(z_stage)
    ctx.check(
        "z_stage_moves_downward",
        stage_high is not None
        and stage_low is not None
        and stage_low[2] < stage_high[2] - 0.20,
        details=f"high={stage_high}, low={stage_low}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
