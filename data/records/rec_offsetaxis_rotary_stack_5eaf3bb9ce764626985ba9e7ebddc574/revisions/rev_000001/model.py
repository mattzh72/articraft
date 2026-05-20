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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_rotary_fixture_stack")

    dark_base = model.material("dark_base", color=(0.18, 0.18, 0.20))
    steel = model.material("steel", color=(0.73, 0.75, 0.78))
    plate = model.material("plate", color=(0.62, 0.66, 0.70))
    flange_accent = model.material("flange_accent", color=(0.46, 0.53, 0.62))

    base = model.part("base_frame")
    base.visual(
        Cylinder(radius=0.15, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_base,
        name="base_foot",
    )
    base.visual(
        Cylinder(radius=0.11, length=0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_base,
        name="base_step",
    )
    base.visual(
        Cylinder(radius=0.048, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=steel,
        name="base_pedestal",
    )

    lower = model.part("lower_stage")
    lower.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=steel,
        name="rotor_spigot",
    )
    lower.visual(
        Cylinder(radius=0.12, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=plate,
        name="rotor_platter",
    )
    lower.visual(
        Box((0.03, 0.06, 0.104)),
        origin=Origin(xyz=(0.125, 0.0, 0.098)),
        material=plate,
        name="side_upright",
    )
    lower.visual(
        Box((0.07, 0.06, 0.02)),
        origin=Origin(xyz=(0.105, 0.0, 0.16)),
        material=plate,
        name="side_bridge",
    )
    lower.visual(
        Cylinder(radius=0.05, length=0.02),
        origin=Origin(xyz=(0.095, 0.0, 0.18)),
        material=steel,
        name="upper_support",
    )

    upper = model.part("upper_stage")
    upper.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=steel,
        name="upper_spigot",
    )
    upper.visual(
        Cylinder(radius=0.055, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=plate,
        name="upper_rotor",
    )
    upper.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=flange_accent,
        name="top_flange",
    )

    model.articulation(
        "base_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.0,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        "lower_to_upper_stage",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.095, 0.0, 0.19)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=3.0,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    lower = object_model.get_part("lower_stage")
    upper = object_model.get_part("upper_stage")

    lower_joint = object_model.get_articulation("base_to_lower_stage")
    upper_joint = object_model.get_articulation("lower_to_upper_stage")

    base_pedestal = base.get_visual("base_pedestal")
    rotor_spigot = lower.get_visual("rotor_spigot")
    rotor_platter = lower.get_visual("rotor_platter")
    upper_support = lower.get_visual("upper_support")
    upper_spigot = upper.get_visual("upper_spigot")
    upper_rotor = upper.get_visual("upper_rotor")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "lower_joint_is_vertical_revolute",
        lower_joint.joint_type == ArticulationType.REVOLUTE
        and tuple(lower_joint.axis) == (0.0, 0.0, 1.0),
        details=f"lower joint type={lower_joint.joint_type}, axis={lower_joint.axis}",
    )
    ctx.check(
        "upper_joint_is_vertical_revolute",
        upper_joint.joint_type == ArticulationType.REVOLUTE
        and tuple(upper_joint.axis) == (0.0, 0.0, 1.0),
        details=f"upper joint type={upper_joint.joint_type}, axis={upper_joint.axis}",
    )
    ctx.check(
        "upper_axis_is_offset_from_base_axis",
        upper_joint.origin.xyz[0] > 0.08 and abs(upper_joint.origin.xyz[1]) < 1e-9,
        details=f"upper joint origin={upper_joint.origin.xyz}",
    )

    with ctx.pose({lower_joint: 0.0, upper_joint: 0.0}):
        ctx.expect_contact(
            lower,
            base,
            elem_a=rotor_spigot,
            elem_b=base_pedestal,
            name="lower_stage_bearing_contact",
        )
        ctx.expect_within(
            lower,
            base,
            axes="xy",
            inner_elem=rotor_spigot,
            outer_elem=base_pedestal,
            margin=0.0,
            name="lower_stage_bearing_nested_in_pedestal",
        )
        ctx.expect_contact(
            upper,
            lower,
            elem_a=upper_spigot,
            elem_b=upper_support,
            name="upper_stage_bearing_contact",
        )
        ctx.expect_within(
            upper,
            lower,
            axes="xy",
            inner_elem=upper_spigot,
            outer_elem=upper_support,
            margin=0.0,
            name="upper_stage_bearing_nested_in_support",
        )
        ctx.expect_origin_distance(
            upper,
            lower,
            axes="xy",
            min_dist=0.094,
            max_dist=0.096,
            name="upper_axis_xy_offset",
        )
        ctx.expect_origin_gap(
            upper,
            lower,
            axis="z",
            min_gap=0.189,
            max_gap=0.191,
            name="upper_axis_height_above_turntable_axis",
        )
        ctx.expect_gap(
            lower,
            base,
            axis="z",
            positive_elem=rotor_platter,
            negative_elem=base_pedestal,
            min_gap=0.017,
            max_gap=0.019,
            name="turntable_platter_clears_base_pedestal",
        )
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            positive_elem=upper_rotor,
            negative_elem=rotor_platter,
            min_gap=0.14,
            max_gap=0.18,
            name="upper_stage_is_raised_above_lower_platter",
        )

    with ctx.pose({lower_joint: pi / 2, upper_joint: -pi / 2}):
        ctx.expect_contact(
            lower,
            base,
            elem_a=rotor_spigot,
            elem_b=base_pedestal,
            name="lower_stage_contact_in_rotated_pose",
        )
        ctx.expect_contact(
            upper,
            lower,
            elem_a=upper_spigot,
            elem_b=upper_support,
            name="upper_stage_contact_in_rotated_pose",
        )
        ctx.expect_origin_distance(
            upper,
            lower,
            axes="xy",
            min_dist=0.094,
            max_dist=0.096,
            name="upper_axis_offset_persists_when_rotated",
        )

    for joint, label in (
        (lower_joint, "lower_stage"),
        (upper_joint, "upper_stage"),
    ):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{label}_lower_limit_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{label}_lower_limit_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{label}_upper_limit_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{label}_upper_limit_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
